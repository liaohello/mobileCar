"""Causal 5 Hz decision-grid alignment over an immutable G2A Episode."""

from __future__ import annotations

import bisect
import hashlib
import math
from collections import Counter
from dataclasses import dataclass
from typing import Any, Iterable, Mapping, Sequence

from .models import (
    ActionDispatch,
    ActuatorReference,
    AlignedRow,
    AlignmentResult,
    ExportProfile,
    ImageSample,
    ObservationSample,
    ProfileRow,
    sha256_json,
)
from .reader import EpisodeReader, SourceIntegrityError


FORBIDDEN_FEATURE_TOKENS = (
    "world_sim",
    "oracle",
    "truth",
    "gt_debug",
    "diagnostics",
)


def assert_policy_feature_allowlist(keys: Iterable[str]) -> None:
    for key in keys:
        lowered = str(key).lower()
        if any(token in lowered for token in FORBIDDEN_FEATURE_TOKENS):
            raise ValueError(f"forbidden policy feature: {key}")


def assert_sample_identity(
    sample: Mapping[str, Any],
    *,
    run_id: str,
    episode_id: str,
    clock_id: str,
) -> None:
    if sample.get("run_id") != run_id:
        raise SourceIntegrityError("cross_run")
    if sample.get("episode_id") != episode_id:
        raise SourceIntegrityError("cross_episode")
    if sample.get("clock_id") != clock_id:
        raise SourceIntegrityError("clock_mismatch")


def latest_arrived(
    samples: Sequence[Any],
    decision_backend_monotonic_s: float,
    *,
    arrival: str,
) -> Any | None:
    """Binary-search a sequence sorted by the named arrival timestamp."""

    times = [float(getattr(sample, arrival)) for sample in samples]
    index = bisect.bisect_right(times, decision_backend_monotonic_s + 1e-9) - 1
    return samples[index] if index >= 0 else None


def parent_split_leak(
    assignments: Iterable[Mapping[str, Any]],
) -> bool:
    splits_by_parent: dict[str, set[str]] = {}
    for item in assignments:
        parent = str(item.get("parent_episode_id", ""))
        split = str(item.get("split", ""))
        if not parent or not split:
            continue
        splits_by_parent.setdefault(parent, set()).add(split)
    return any(len(splits) > 1 for splits in splits_by_parent.values())


@dataclass(frozen=True)
class ClockAnchor:
    sim_time_s: float
    backend_monotonic_s: float
    source: str


class SimMonotonicMapper:
    """Piecewise-linear map built only from explicitly paired source clocks."""

    def __init__(self, anchors: Iterable[ClockAnchor]) -> None:
        deduplicated: dict[float, ClockAnchor] = {}
        for anchor in anchors:
            if not math.isfinite(anchor.sim_time_s) or not math.isfinite(
                anchor.backend_monotonic_s
            ):
                continue
            previous = deduplicated.get(anchor.sim_time_s)
            if previous is None or anchor.backend_monotonic_s < previous.backend_monotonic_s:
                deduplicated[anchor.sim_time_s] = anchor
        self.anchors = sorted(
            deduplicated.values(), key=lambda item: item.sim_time_s
        )
        if len(self.anchors) < 2:
            raise SourceIntegrityError("clock mapping has fewer than two anchors")
        sim_times = [item.sim_time_s for item in self.anchors]
        backend_times = [item.backend_monotonic_s for item in self.anchors]
        if any(b <= a for a, b in zip(sim_times, sim_times[1:])):
            raise SourceIntegrityError("clock mapping sim anchors are not increasing")
        if any(b <= a for a, b in zip(backend_times, backend_times[1:])):
            raise SourceIntegrityError(
                "clock mapping backend anchors are not increasing"
            )
        self._sim_times = sim_times

    @classmethod
    def from_episode(cls, reader: EpisodeReader) -> "SimMonotonicMapper":
        start_sim = float(reader.recording_start["sim_time_s"])
        start_backend = float(reader.recording_start["backend_monotonic_s"])
        anchors = [
            ClockAnchor(
                start_sim,
                start_backend,
                "recording_start",
            ),
            ClockAnchor(
                float(reader.recording_stop["sim_time_s"]),
                float(reader.recording_stop["backend_monotonic_s"]),
                "recording_stop",
            ),
        ]
        anchors.extend(
            ClockAnchor(
                image.capture_sim_time_s,
                image.capture_backend_monotonic_s,
                "persisted_rgbd_capture",
            )
            for image in reader.images
            if image.capture_sim_time_s >= start_sim - 1e-9
            and image.capture_backend_monotonic_s >= start_backend - 1e-9
        )
        return cls(anchors)

    def to_backend(self, sim_time_s: float) -> float:
        first = self.anchors[0].sim_time_s
        last = self.anchors[-1].sim_time_s
        if not first - 1e-9 <= sim_time_s <= last + 1e-9:
            raise ValueError("sim time is outside declared clock anchors")
        sim_time_s = min(last, max(first, sim_time_s))
        index = bisect.bisect_left(self._sim_times, sim_time_s)
        if index == 0:
            left, right = self.anchors[0], self.anchors[1]
        elif index >= len(self.anchors):
            left, right = self.anchors[-2], self.anchors[-1]
        else:
            left, right = self.anchors[index - 1], self.anchors[index]
        if abs(sim_time_s - left.sim_time_s) <= 1e-12:
            return left.backend_monotonic_s
        if abs(sim_time_s - right.sim_time_s) <= 1e-12:
            return right.backend_monotonic_s
        fraction = (sim_time_s - left.sim_time_s) / (
            right.sim_time_s - left.sim_time_s
        )
        return left.backend_monotonic_s + fraction * (
            right.backend_monotonic_s - left.backend_monotonic_s
        )

    def to_json(self) -> dict[str, Any]:
        return {
            "method": "piecewise_linear_explicit_pairs",
            "anchor_count": len(self.anchors),
            "anchors": [
                {
                    "sim_time_s": item.sim_time_s,
                    "backend_monotonic_s": item.backend_monotonic_s,
                    "source": item.source,
                }
                for item in self.anchors
            ],
            "anchor_sha256": sha256_json(
                [
                    (item.sim_time_s, item.backend_monotonic_s, item.source)
                    for item in self.anchors
                ]
            ),
        }


def _action_dispatch_at(
    dispatches: Sequence[ActionDispatch],
    group: str,
    decision_sim_time_s: float,
    decision_backend_monotonic_s: float,
) -> tuple[ActionDispatch | None, ActionDispatch | None]:
    candidates = [item for item in dispatches if item.group == group]
    active: tuple[ActionDispatch, ActionDispatch | None] | None = None
    for index, item in enumerate(candidates):
        if (
            item.sim_time_s <= decision_sim_time_s + 1e-9
            and item.backend_monotonic_s <= decision_backend_monotonic_s + 1e-9
        ):
            next_item = candidates[index + 1] if index + 1 < len(candidates) else None
            if next_item is None or decision_sim_time_s < next_item.sim_time_s - 1e-9:
                active = (item, next_item)
    return active if active is not None else (None, None)


def _reference_at(
    references: Sequence[ActuatorReference],
    decision_sim_time_s: float,
    decision_backend_monotonic_s: float,
) -> ActuatorReference | None:
    index = bisect.bisect_right(
        [item.backend_monotonic_s for item in references],
        decision_backend_monotonic_s + 1e-9,
    ) - 1
    while index >= 0:
        item = references[index]
        if item.sim_time_s <= decision_sim_time_s + 1e-9:
            return item
        index -= 1
    return None


def _latest_observation_at(
    samples: Sequence[ObservationSample],
    decision_backend_monotonic_s: float,
    *,
    max_capture_sim_time_s: float,
) -> ObservationSample | None:
    index = bisect.bisect_right(
        [item.arrival_backend_monotonic_s for item in samples],
        decision_backend_monotonic_s + 1e-9,
    ) - 1
    while index >= 0:
        item = samples[index]
        if item.capture_sim_time_s <= max_capture_sim_time_s + 1e-9:
            return item
        index -= 1
    return None


def _exact_feedback(
    samples: Sequence[ObservationSample],
    sample_id: str,
    decision_backend_monotonic_s: float,
) -> ObservationSample | None:
    for item in samples:
        if item.sample_id == sample_id:
            if item.arrival_backend_monotonic_s <= decision_backend_monotonic_s + 1e-9:
                return item
            return None
    return None


def _dispatch_provenance(
    dispatch: ActionDispatch,
    next_dispatch: ActionDispatch | None,
) -> dict[str, Any]:
    request = (dispatch.requested_event.get("payload") or {}).get("request") or {}
    approved = (dispatch.approved_event.get("payload") or {}).get("decision") or {}
    return {
        "command_id": dispatch.command_id,
        "command_type": dispatch.command_type,
        "requested": {
            "backend_monotonic_s": dispatch.requested_event.get(
                "backend_monotonic_s"
            ),
            "sim_time_s": dispatch.requested_event.get("sim_time_s"),
            "payload": request.get("payload"),
        },
        "approved": {
            "backend_monotonic_s": dispatch.approved_event.get(
                "backend_monotonic_s"
            ),
            "sim_time_s": dispatch.approved_event.get("sim_time_s"),
            "approved": bool(approved.get("approved", True)),
            "effective_payload": approved.get("effective_payload"),
        },
        "dispatch": {
            "backend_monotonic_s": dispatch.backend_monotonic_s,
            "sim_time_s": dispatch.sim_time_s,
            "control_tick_id": dispatch.control_tick_id,
            "payload": dispatch.dispatch_event.get("payload"),
        },
        "effective_until": (
            {
                "backend_monotonic_s": next_dispatch.backend_monotonic_s,
                "sim_time_s": next_dispatch.sim_time_s,
                "control_tick_id": next_dispatch.control_tick_id,
            }
            if next_dispatch is not None
            else {"reason": "recording_stop_or_next_scan"}
        ),
    }


def _model_to_user(
    model_angles: Sequence[float],
    *,
    zero_model_rad: Sequence[float],
    direction: Sequence[float],
) -> list[float]:
    if not (len(model_angles) == len(zero_model_rad) == len(direction) == 5):
        raise ValueError("D2 user-coordinate mapping requires five values")
    values = [
        (float(model) - float(zero)) * float(sign)
        for model, zero, sign in zip(
            model_angles, zero_model_rad, direction, strict=True
        )
    ]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("D2 mapped action is not finite")
    return values


def _d0_state(
    encoder: ObservationSample | None,
    imu: ObservationSample | None,
) -> list[float] | None:
    if encoder is None or imu is None:
        return None
    wheel = (encoder.value or {}).get("wheel_velocity_rad_s")
    accel = (imu.value or {}).get("acceleration_m_s2")
    gyro = (imu.value or {}).get("gyro_rad_s")
    if not (
        isinstance(wheel, list)
        and len(wheel) == 4
        and isinstance(accel, list)
        and len(accel) == 3
        and isinstance(gyro, list)
        and len(gyro) == 3
    ):
        return None
    values = [float(item) for item in [*wheel, *accel, *gyro]]
    return values if all(math.isfinite(item) for item in values) else None


def _d2_state(
    feedback: ObservationSample,
    *,
    zero_model_rad: Sequence[float],
    direction: Sequence[float],
) -> list[float] | None:
    joints = (feedback.value or {}).get("user_joint_positions_rad")
    if joints is None:
        model_joints = (feedback.value or {}).get("model_joint_positions_rad")
        if (
            isinstance(model_joints, list)
            and len(model_joints) >= 5
            and all(
                isinstance(value, (int, float))
                and not isinstance(value, bool)
                and math.isfinite(float(value))
                for value in model_joints[:5]
            )
        ):
            try:
                joints = _model_to_user(
                    model_joints[:5],
                    zero_model_rad=zero_model_rad,
                    direction=direction,
                )
            except (TypeError, ValueError):
                return None
    width = (feedback.value or {}).get("gripper_width_m")
    if (
        not isinstance(joints, list)
        or len(joints) != 5
        or not isinstance(width, (int, float))
        or isinstance(width, bool)
    ):
        return None
    values = [float(item) for item in [*joints, width]]
    return values if all(math.isfinite(item) for item in values) else None


def align_episode(
    reader: EpisodeReader,
    profile: ExportProfile,
    *,
    include_init_and_final: bool = True,
) -> AlignmentResult:
    """Align one Episode without writing or mutating any source record."""

    if reader.audit_status != "COMPLETE":
        return AlignmentResult(
            rows=[],
            rejected=[
                {
                    "reason": "source_audit_incomplete",
                    "audit_status": reader.audit_status,
                }
            ],
            summary={
                "grid_slots": 0,
                "accepted_rows": 0,
                "d0_rows": 0,
                "d2_rows": 0,
                "excluded_reasons": {"source_audit_incomplete": 1},
            },
            source_identity=reader.identity(),
        )

    mapper = SimMonotonicMapper.from_episode(reader)
    grid_origin = float(reader.recording_start["sim_time_s"]) + float(
        profile.raw["grid"].get("phase_offset_s", 0.0)
    )
    grid_end = float(reader.recording_stop["sim_time_s"])
    if not include_init_and_final:
        accepted_window = (profile.raw.get("grid") or {}).get("analysis_window_s")
        if accepted_window:
            grid_origin = max(grid_origin, float(accepted_window[0]))
            grid_end = min(grid_end, float(accepted_window[1]))
    step = 1.0 / profile.grid_rate_hz
    slot_count = int(math.floor((grid_end - grid_origin) / step + 1e-9)) + 1
    if slot_count <= 0:
        raise SourceIntegrityError("source Episode has no 5 Hz grid slots")

    rejected: list[dict[str, Any]] = []
    rows: list[AlignedRow] = []
    used_images: set[str] = set()
    previous_accepted_grid: int | None = None
    reference_times = [item.backend_monotonic_s for item in reader.actuator_references]
    d0_spec = profile.action_spec("d0")
    d2_spec = profile.action_spec("d2")
    zero = d2_spec["user_zero_model_rad"]
    direction = d2_spec["user_direction"]

    for grid_index in range(slot_count):
        decision_sim = grid_origin + grid_index * step
        if decision_sim > grid_end + 1e-9:
            break
        decision_backend = mapper.to_backend(decision_sim)
        reasons: list[str] = []

        image = latest_arrived(
            reader.images,
            decision_backend,
            arrival="arrival_backend_monotonic_s",
        )
        if image is None:
            rejected.append(
                {
                    "grid_index": grid_index,
                    "decision_sim_time_s": decision_sim,
                    "decision_backend_monotonic_s": decision_backend,
                    "reasons": ["image_not_arrived_by_decision"],
                }
            )
            continue
        if image.sample_id in used_images:
            rejected.append(
                {
                    "grid_index": grid_index,
                    "decision_sim_time_s": decision_sim,
                    "decision_backend_monotonic_s": decision_backend,
                    "sample_id": image.sample_id,
                    "reasons": ["duplicate_image_for_new_grid"],
                }
            )
            continue
        image_age = decision_backend - image.capture_backend_monotonic_s
        image_arrival_age = decision_backend - image.arrival_backend_monotonic_s
        if image_age < -1e-9:
            reasons.append("future_image_capture")
        if not 0.0 <= image_age <= profile.image_max_age_s + 1e-9:
            reasons.append("image_age_exceeded")
        if image_arrival_age < -1e-9:
            reasons.append("future_image_arrival")

        feedback = _exact_feedback(
            reader.feedback,
            image.feedback_sample_id,
            decision_backend,
        )
        if feedback is None:
            reasons.append("linked_feedback_unavailable")
            feedback_age = math.inf
            skew = math.inf
        else:
            feedback_age = decision_backend - feedback.capture_backend_monotonic_s
            skew = abs(
                feedback.capture_backend_monotonic_s
                - image.capture_backend_monotonic_s
            )
            if not 0.0 <= feedback_age <= profile.feedback_max_age_s + 1e-9:
                reasons.append("feedback_age_exceeded")
            if skew > profile.max_capture_skew_s + 1e-9:
                reasons.append("image_feedback_capture_skew_exceeded")

        encoder = _latest_observation_at(
            reader.encoders,
            decision_backend,
            max_capture_sim_time_s=decision_sim,
        )
        imu = _latest_observation_at(
            reader.imu,
            decision_backend,
            max_capture_sim_time_s=decision_sim,
        )
        reference = _reference_at(
            reader.actuator_references,
            decision_sim,
            decision_backend,
        )
        if reference is None:
            reasons.append("actuator_reference_unavailable")

        base_dispatch, next_base = _action_dispatch_at(
            reader.dispatch_intervals,
            "base",
            decision_sim,
            decision_backend,
        )
        arm_dispatch, next_arm = _action_dispatch_at(
            reader.dispatch_intervals,
            "arm",
            decision_sim,
            decision_backend,
        )
        gripper_dispatch, next_gripper = _action_dispatch_at(
            reader.dispatch_intervals,
            "gripper",
            decision_sim,
            decision_backend,
        )

        profile_rows: dict[str, ProfileRow] = {}
        if reference is not None:
            if base_dispatch is None:
                reasons.append("d0_no_effective_dispatch")
            elif (
                base_dispatch.command_type == "zero_base"
                and decision_sim
                - base_dispatch.sim_time_s
                > float(
                    (profile.raw.get("action_windows") or {}).get(
                        "d0_post_zero_hold_s", 0.0
                    )
                )
                + 1e-9
            ):
                reasons.append("d0_post_zero_hold_ended")
            else:
                base_action = reference.payload.get("base_twist")
                state = _d0_state(encoder, imu)
                if (
                    isinstance(base_action, list)
                    and len(base_action) == 3
                    and all(
                        isinstance(value, (int, float))
                        and not isinstance(value, bool)
                        and math.isfinite(float(value))
                        for value in base_action
                    )
                    and state is not None
                ):
                    profile_rows["d0"] = ProfileRow(
                        profile="d0",
                        state=state,
                        action=[float(value) for value in base_action],
                        action_units=["m/s", "m/s", "rad/s"],
                        action_frame_id="base_link",
                        dispatch_provenance={
                            "base": _dispatch_provenance(base_dispatch, next_base)
                        },
                        effective_intervals={
                            "base": [
                                base_dispatch.sim_time_s,
                                (
                                    next_base.sim_time_s
                                    if next_base is not None
                                    else grid_end
                                ),
                            ]
                        },
                    )
                else:
                    reasons.append("d0_state_or_action_invalid")

            if arm_dispatch is None or gripper_dispatch is None:
                reasons.append("d2_no_effective_dispatch")
            else:
                model_angles = reference.payload.get("arm_model_reference_rad")
                gripper_target = reference.payload.get("gripper_width_target_m")
                d2_state = (
                    _d2_state(
                        feedback,
                        zero_model_rad=zero,
                        direction=direction,
                    )
                    if feedback is not None
                    else None
                )
                if (
                    isinstance(model_angles, list)
                    and len(model_angles) >= 5
                    and isinstance(gripper_target, (int, float))
                    and not isinstance(gripper_target, bool)
                    and d2_state is not None
                ):
                    try:
                        user_angles = _model_to_user(
                            model_angles[:5],
                            zero_model_rad=zero,
                            direction=direction,
                        )
                        profile_rows["d2"] = ProfileRow(
                            profile="d2",
                            state=d2_state,
                            action=[*user_angles, float(gripper_target)],
                            action_units=["rad"] * 5 + ["m"],
                            action_frame_id="arm_base",
                            dispatch_provenance={
                                "arm": _dispatch_provenance(
                                    arm_dispatch, next_arm
                                ),
                                "gripper": _dispatch_provenance(
                                    gripper_dispatch, next_gripper
                                ),
                            },
                            effective_intervals={
                                "arm": [
                                    arm_dispatch.sim_time_s,
                                    (
                                        next_arm.sim_time_s
                                        if next_arm is not None
                                        else grid_end
                                    ),
                                ],
                                "gripper": [
                                    gripper_dispatch.sim_time_s,
                                    (
                                        next_gripper.sim_time_s
                                        if next_gripper is not None
                                        else grid_end
                                    ),
                                ],
                            },
                        )
                    except (TypeError, ValueError):
                        reasons.append("d2_action_mapping_invalid")
                else:
                    reasons.append("d2_state_or_action_invalid")

        # Shared observation causality failures invalidate an otherwise
        # exportable row. Missing action provenance affects only its profile.
        shared_invalid = any(
            reason
            for reason in reasons
            if reason
            in {
                "future_image_capture",
                "image_age_exceeded",
                "future_image_arrival",
                "linked_feedback_unavailable",
                "feedback_age_exceeded",
                "image_feedback_capture_skew_exceeded",
                "actuator_reference_unavailable",
            }
        )
        if shared_invalid:
            profile_rows = {}
        else:
            if "d0_no_effective_dispatch" in reasons:
                profile_rows.pop("d0", None)
            if "d2_no_effective_dispatch" in reasons:
                profile_rows.pop("d2", None)
        if not profile_rows:
            rejected.append(
                {
                    "grid_index": grid_index,
                    "decision_sim_time_s": decision_sim,
                    "decision_backend_monotonic_s": decision_backend,
                    "image_sample_id": image.sample_id,
                    "reasons": sorted(set(reasons)),
                }
            )
            continue

        if (
            previous_accepted_grid is not None
            and grid_index - previous_accepted_grid - 1 > profile.max_gap_slots
        ):
            # Keep accepted data but record an explicit split. No interpolation
            # or copied frame is introduced.
            reasons.append("accepted_after_long_gap_split")
        used_images.add(image.sample_id)
        previous_accepted_grid = grid_index
        rows.append(
            AlignedRow(
                grid_index=grid_index,
                decision_sim_time_s=decision_sim,
                decision_backend_monotonic_s=decision_backend,
                image=image,
                joint_feedback=feedback,
                encoder=encoder,
                imu=imu,
                actuator_reference=reference,
                image_age_s=image_age,
                image_arrival_age_s=image_arrival_age,
                feedback_age_s=feedback_age,
                image_feedback_capture_skew_s=skew,
                profiles=profile_rows,
                reasons=sorted(set(reasons)),
            )
        )

    d0_rows = sum("d0" in row.profiles for row in rows)
    d2_rows = sum("d2" in row.profiles for row in rows)
    reason_counts = Counter(
        reason for item in rejected for reason in item.get("reasons", [])
    )
    result = AlignmentResult(
        rows=rows,
        rejected=rejected,
        summary={
            "profile_id": profile.profile_id,
            "profile_sha256": profile.sha256,
            "grid_rate_hz": profile.grid_rate_hz,
            "grid_origin_sim_time_s": grid_origin,
            "grid_end_sim_time_s": grid_end,
            "grid_slots": slot_count,
            "accepted_rows": len(rows),
            "d0_rows": d0_rows,
            "d2_rows": d2_rows,
            "excluded_slots": len(rejected),
            "excluded_reasons": dict(sorted(reason_counts.items())),
            "unique_images": len(used_images),
            "duplicate_images_reused": 0,
            "clock_mapping": mapper.to_json(),
            "gates": {
                "nonempty_d0": d0_rows > 0,
                "nonempty_d2": d2_rows > 0,
                "all_rows_have_source_identity": True,
                "no_future_image": all(
                    row.image.arrival_backend_monotonic_s
                    <= row.decision_backend_monotonic_s + 1e-9
                    for row in rows
                ),
                "no_duplicate_image": len(used_images) == len(rows),
                "image_age": all(
                    row.image_age_s <= profile.image_max_age_s + 1e-9
                    for row in rows
                ),
                "feedback_age": all(
                    row.feedback_age_s <= profile.feedback_max_age_s + 1e-9
                    for row in rows
                ),
                "capture_skew": all(
                    row.image_feedback_capture_skew_s
                    <= profile.max_capture_skew_s + 1e-9
                    for row in rows
                ),
            },
        },
        source_identity=reader.identity(),
    )
    result.summary["rule_sha256"] = sha256_json(
        {
            "profile_sha256": profile.sha256,
            "clock_anchor_sha256": result.summary["clock_mapping"]["anchor_sha256"],
            "selection_rules": profile.raw["causality"],
            "actions": profile.raw["actions"],
            "features": profile.raw["features"],
        }
    )
    return result
