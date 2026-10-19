"""Independent format, export and training qualification checks for G2B."""

from __future__ import annotations

import math
from typing import Any, Iterable, Mapping

from .models import AlignmentResult, AlignedRow, ExportProfile


def _finite(values: Iterable[float]) -> bool:
    return all(
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
        for value in values
    )


def _row_failures(
    profile_name: str,
    profile: ExportProfile,
    row: AlignedRow,
) -> list[str]:
    key = profile_name.lower()
    failures: list[str] = []
    issued = row.profiles.get(key)
    if issued is None:
        return ["profile_row_missing"]
    feature = profile.feature_spec(key)
    action_spec = profile.action_spec(key)
    if len(issued.state) != len(feature["state_names"]):
        failures.append("state_shape_mismatch")
    if len(issued.action) != len(feature["action_names"]):
        failures.append("action_shape_mismatch")
    if not _finite(issued.state):
        failures.append("state_not_finite")
    if not _finite(issued.action):
        failures.append("action_not_finite")
    if list(issued.action_units) != list(action_spec["units"]):
        failures.append("action_units_mismatch")
    if issued.action_frame_id != action_spec["frame_id"]:
        failures.append("action_frame_mismatch")

    image = row.image
    if image.arrival_backend_monotonic_s > row.decision_backend_monotonic_s + 1e-9:
        failures.append("future_image_arrival")
    if image.capture_backend_monotonic_s > row.decision_backend_monotonic_s + 1e-9:
        failures.append("future_image_capture")
    if row.image_age_s < -1e-9 or row.image_age_s > profile.image_max_age_s + 1e-9:
        failures.append("image_age")
    if not image.sample_id or not image.control_tick_id or not image.feedback_sample_id:
        failures.append("image_source_identity")

    reference = row.actuator_reference
    if reference.backend_monotonic_s > row.decision_backend_monotonic_s + 1e-9:
        failures.append("future_action_reference")
    if reference.sim_time_s > row.decision_sim_time_s + 1e-9:
        failures.append("future_action_sim_time")
    for source_name, source in issued.dispatch_provenance.items():
        dispatch = (source or {}).get("dispatch") or {}
        dispatch_sim = dispatch.get("sim_time_s")
        if (
            isinstance(dispatch_sim, (int, float))
            and reference.sim_time_s + 1e-9 < float(dispatch_sim)
        ):
            failures.append(f"{source_name}_reference_before_dispatch")

    if key == "d0":
        for label, sample in (("encoder", row.encoder), ("imu", row.imu)):
            if sample is None:
                failures.append(f"{label}_missing")
                continue
            age = row.decision_backend_monotonic_s - sample.capture_backend_monotonic_s
            if sample.arrival_backend_monotonic_s > row.decision_backend_monotonic_s + 1e-9:
                failures.append(f"future_{label}_arrival")
            if sample.capture_sim_time_s > row.decision_sim_time_s + 1e-9:
                failures.append(f"future_{label}_capture")
            if age < -1e-9 or age > profile.feedback_max_age_s + 1e-9:
                failures.append(f"{label}_age")
            if (
                abs(
                    sample.capture_sim_time_s
                    - row.image.capture_sim_time_s
                )
                > profile.max_capture_skew_s + 1e-9
            ):
                failures.append(f"d0_image_{label}_skew")
        if "base" not in issued.dispatch_provenance:
            failures.append("d0_dispatch_provenance_missing")
        if "base" not in issued.effective_intervals:
            failures.append("d0_effective_interval_missing")
    elif key == "d2":
        feedback = row.joint_feedback
        if feedback is None:
            failures.append("feedback_missing")
        else:
            if feedback.control_tick_id != row.image.control_tick_id:
                failures.append("feedback_tick_mismatch")
            if feedback.arrival_backend_monotonic_s > row.decision_backend_monotonic_s + 1e-9:
                failures.append("future_feedback_arrival")
            if row.feedback_age_s < -1e-9 or row.feedback_age_s > profile.feedback_max_age_s + 1e-9:
                failures.append("feedback_age")
            if (
                row.image_feedback_capture_skew_s < -1e-9
                or row.image_feedback_capture_skew_s
                > profile.max_capture_skew_s + 1e-9
            ):
                failures.append("image_feedback_capture_skew")
        if set(("arm", "gripper")) - set(issued.dispatch_provenance):
            failures.append("d2_dispatch_provenance_missing")
        if set(("arm", "gripper")) - set(issued.effective_intervals):
            failures.append("d2_effective_interval_missing")
    return sorted(set(failures))


def assess_export_valid(
    *,
    profile_name: str,
    profile: ExportProfile,
    alignment: AlignmentResult,
) -> dict[str, Any]:
    key = profile_name.lower()
    rows = alignment.rows_for(key)
    failures: dict[str, int] = {}
    bad_rows: list[dict[str, Any]] = []
    for row in rows:
        row_failures = _row_failures(key, profile, row)
        if not row_failures:
            continue
        for reason in row_failures:
            failures[reason] = failures.get(reason, 0) + 1
        if len(bad_rows) < 20:
            bad_rows.append(
                {
                    "grid_index": row.grid_index,
                    "decision_sim_time_s": row.decision_sim_time_s,
                    "reasons": row_failures,
                }
            )
    sample_ids = [row.image.sample_id for row in rows]
    if len(sample_ids) != len(set(sample_ids)):
        failures["duplicate_image_sample"] = len(sample_ids) - len(set(sample_ids))
    gates = {
        "nonempty": bool(rows),
        "all_rows_valid": not failures,
        "all_rows_have_source_identity": all(
            row.image.sample_id
            and row.image.recording_segment_id
            and row.actuator_reference.control_tick_id
            for row in rows
        ),
        "no_future_observations": not (
            {"future_image_arrival", "future_image_capture"} & set(failures)
        ),
        "no_duplicate_image": "duplicate_image_sample" not in failures,
        "declared_profile": alignment.summary.get("profile_sha256") == profile.sha256,
    }
    return {
        "schema_version": 1,
        "profile": key.upper(),
        "profile_id": profile.profile_id,
        "profile_sha256": profile.sha256,
        "passed": all(gates.values()),
        "rows": len(rows),
        "segments": len(alignment.contiguous_segments(key)),
        "rejected_slots": len(alignment.rejected),
        "gates": gates,
        "row_failure_counts": dict(sorted(failures.items())),
        "bad_rows": bad_rows,
    }


def assess_train_qualified(
    *,
    profile_name: str,
    profile: ExportProfile,
    operator_kind: str | None,
    task_result: str | None,
    view_source: str | None,
    parent_episode_count: int,
    export_result: Mapping[str, Any],
) -> dict[str, Any]:
    rules = profile.raw["train_qualification"]
    reasons: list[str] = []
    if operator_kind != rules["required_operator_kind"]:
        reasons.append(
            f"operator_kind={operator_kind!r} requires {rules['required_operator_kind']!r}"
        )
    if task_result != rules["required_task_result"]:
        reasons.append(
            f"task_result={task_result!r} requires {rules['required_task_result']!r}"
        )
    if parent_episode_count < int(rules["minimum_parent_episodes"]):
        reasons.append(
            f"parent_episode_count={parent_episode_count} requires "
            f">={rules['minimum_parent_episodes']}"
        )
    if not export_result.get("passed"):
        reasons.append("export_valid_false")
    if view_source is None:
        reasons.append("view_source_missing")
    elif view_source not in rules.get("required_view_sources", [view_source]):
        reasons.append(f"view_source={view_source!r} is not in the training profile")
    return {
        "schema_version": 1,
        "training_profile_id": rules["training_profile_id"],
        "export_profile": profile_name.upper(),
        "qualified": not reasons,
        "reasons": reasons,
        "source_operator_kind": operator_kind,
        "source_task_result": task_result,
        "source_view_source": view_source,
        "parent_episode_count": parent_episode_count,
        "minimum_parent_episodes": int(rules["minimum_parent_episodes"]),
    }
