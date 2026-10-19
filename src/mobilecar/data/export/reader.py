"""Strict read-only access to existing G2A Episodes."""

from __future__ import annotations

import json
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Iterable, Mapping

from .models import (
    ActionDispatch,
    ActuatorReference,
    ImageSample,
    ObservationSample,
    sha256_file,
)

ACTION_GROUPS = {
    "set_base_twist": "base",
    "zero_base": "base",
    "set_arm_joint_goal": "arm",
    "set_gripper_width_goal": "gripper",
}


class SourceIntegrityError(RuntimeError):
    """Raised when a source identity or required immutable record is invalid."""


def read_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    value = json.loads(path.read_text(encoding="utf-8"))
    return value if isinstance(value, dict) else {}


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path.is_file():
        return rows
    with path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, 1):
            text = line.strip()
            if not text:
                continue
            value = json.loads(text)
            if not isinstance(value, dict):
                raise SourceIntegrityError(
                    f"{path}:{line_number} is not a JSON object"
                )
            rows.append(value)
    return rows


def _finite(value: Any) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("not numeric")
    result = float(value)
    if not result == result or result in (float("inf"), float("-inf")):
        raise ValueError("not finite")
    return result


def _field_float(row: Mapping[str, Any], key: str) -> float:
    try:
        return _finite(row.get(key))
    except (TypeError, ValueError) as exc:
        raise SourceIntegrityError(f"missing finite {key}") from exc


def _field_text(row: Mapping[str, Any], key: str) -> str:
    value = row.get(key)
    if not isinstance(value, str) or not value:
        raise SourceIntegrityError(f"missing text field {key}")
    return value


def _tick_number(value: Any) -> int | None:
    if not isinstance(value, str):
        return None
    try:
        return int(value.rsplit("_", 1)[-1])
    except (TypeError, ValueError):
        return None


def dispatch_group(command_type: str) -> str | None:
    """Return the effective-reference channel or None for non-actions."""

    return ACTION_GROUPS.get(str(command_type))


def resolve_episode_media(episode_dir: str | Path, relative_uri: str) -> Path:
    """Resolve a non-empty source asset without permitting path escape."""

    root = Path(episode_dir).resolve()
    path = (root / relative_uri).resolve()
    if not path.is_relative_to(root):
        raise SourceIntegrityError("source asset escapes Episode directory")
    if not path.is_file() or path.stat().st_size <= 0:
        raise SourceIntegrityError(f"source asset missing: {relative_uri}")
    return path


class EpisodeReader:
    """Immutable Episode view that never opens a backend or writes source files."""

    def __init__(
        self,
        run_root: str | Path,
        run_id: str,
        episode_id: str,
    ) -> None:
        self.run_root = Path(run_root).resolve()
        self.run_id = run_id
        self.episode_id = episode_id
        self.run_dir = self.run_root / run_id
        self.episode_dir = self.run_dir / "episodes" / episode_id
        if not self.episode_dir.is_dir():
            raise SourceIntegrityError(f"Episode directory missing: {self.episode_dir}")

        self.manifest = read_json(self.run_dir / "manifest.json")
        self.episode = read_json(self.episode_dir / "episode.json")
        self.episode_complete = read_json(self.episode_dir / "episode_complete.json")
        self.assessment = read_json(
            self.episode_dir / "assessments/task_assessment.json"
        )
        self.events = read_jsonl(self.episode_dir / "events.jsonl")
        self.commands = read_jsonl(self.episode_dir / "commands.jsonl")
        self.control = read_jsonl(self.episode_dir / "control.jsonl")
        self.observation_index = read_jsonl(
            self.episode_dir / "observations/index.jsonl"
        )
        self.persisted_images = read_jsonl(
            self.episode_dir / "observations/persist.jsonl"
        )
        self._validate_identity()

        self.recording_start = self._single_event("recording_start")
        self.recording_stop = self._last_event("recording_stop")
        self.images = self._build_images()
        self.feedback_published = self._build_observations("joint_feedback")
        self.feedback = self._build_event_feedback()
        self.encoders = self._build_observations("encoders")
        self.imu = self._build_observations("imu")
        self.dispatch_intervals = self._build_dispatches()
        self.actuator_references = self._build_actuator_references()
        self.segment_ranges = self._segment_ranges()

    def _validate_identity(self) -> None:
        if not self.episode:
            raise SourceIntegrityError("episode.json is missing")
        if self.episode.get("run_id") != self.run_id:
            raise SourceIntegrityError("episode run_id mismatch")
        if self.episode.get("episode_id") != self.episode_id:
            raise SourceIntegrityError("episode_id mismatch")
        if self.episode.get("contract_version") != "mobilecar-1.0":
            raise SourceIntegrityError("unsupported source contract_version")
        if self.manifest and self.manifest.get("run_id") != self.run_id:
            raise SourceIntegrityError("manifest run_id mismatch")

    def _single_event(self, event_type: str) -> dict[str, Any]:
        rows = [row for row in self.events if row.get("event_type") == event_type]
        if len(rows) != 1:
            raise SourceIntegrityError(
                f"expected one {event_type} event, found {len(rows)}"
            )
        return rows[0]

    def _last_event(self, event_type: str) -> dict[str, Any]:
        rows = [row for row in self.events if row.get("event_type") == event_type]
        if not rows:
            raise SourceIntegrityError(f"missing terminal {event_type} event")
        return rows[-1]

    def _build_images(self) -> list[ImageSample]:
        rgb_by_key = {
            (str(row.get("stream_id")), str(row.get("sample_id"))): row
            for row in self.observation_index
            if row.get("sensor_kind") == "rgb"
        }
        depth_by_key = {
            (str(row.get("stream_id")), str(row.get("sample_id"))): row
            for row in self.observation_index
            if row.get("sensor_kind") == "depth"
        }
        segment = self._active_segment_id()
        images: list[ImageSample] = []
        for persist in self.persisted_images:
            self._validate_row_identity(persist)
            key = (str(persist.get("stream_id")), str(persist.get("sample_id")))
            rgb = rgb_by_key.get(key)
            depth = depth_by_key.get(key)
            if rgb is None or depth is None:
                continue
            try:
                image = ImageSample(
                    sample_id=key[1],
                    stream_id=key[0],
                    recording_segment_id=segment,
                    capture_sim_time_s=_field_float(
                        persist, "capture_sim_time_s"
                    ),
                    capture_backend_monotonic_s=_field_float(
                        persist, "capture_backend_monotonic_s"
                    ),
                    arrival_backend_monotonic_s=_field_float(
                        persist, "arrival_backend_monotonic_s"
                    ),
                    persist_backend_monotonic_s=_field_float(
                        persist, "persist_backend_monotonic_s"
                    ),
                    publish_backend_monotonic_s=_field_float(
                        (rgb.get("metadata") or {}).get("timing") or {},
                        "publish_backend_monotonic_s",
                    ),
                    control_tick_id=_field_text(
                        persist, "capture_control_tick_id"
                    ),
                    feedback_sample_id=_field_text(
                        persist, "feedback_sample_id"
                    ),
                    rgb_uri=_field_text(persist, "rgb_uri"),
                    depth_uri=_field_text(persist, "depth_uri"),
                    valid_mask_uri=_field_text(persist, "valid_mask_uri"),
                    width=int((rgb.get("value") or {}).get("width", 0)),
                    height=int((rgb.get("value") or {}).get("height", 0)),
                    run_id=self.run_id,
                    episode_id=self.episode_id,
                    rgb=rgb,
                    depth=depth,
                    persist=persist,
                )
            except (TypeError, ValueError) as exc:
                raise SourceIntegrityError(
                    f"invalid persisted image {key[0]}:{key[1]}: {exc}"
                ) from exc
            images.append(image)
        images.sort(
            key=lambda item: (
                item.capture_backend_monotonic_s,
                item.sample_id,
            )
        )
        return images

    def _build_observations(self, sensor_kind: str) -> list[ObservationSample]:
        segment = self._active_segment_id()
        result: list[ObservationSample] = []
        for row in self.observation_index:
            if row.get("sensor_kind") != sensor_kind:
                continue
            self._validate_row_identity(row)
            try:
                result.append(
                    ObservationSample(
                        sample_id=_field_text(row, "sample_id"),
                        sensor_kind=sensor_kind,
                        stream_id=_field_text(row, "stream_id"),
                        recording_segment_id=segment,
                        control_tick_id=str(
                            row.get("capture_control_tick_id")
                            or (row.get("metadata") or {}).get("control_tick_id")
                            or ""
                        ),
                        capture_sim_time_s=_field_float(
                            row, "capture_sim_time_s"
                        ),
                        capture_backend_monotonic_s=_field_float(
                            row, "capture_backend_monotonic_s"
                        ),
                        arrival_backend_monotonic_s=_field_float(
                            row, "arrival_time_s"
                        ),
                        value=dict(row.get("value") or {}),
                        raw=row,
                    )
                )
            except (TypeError, ValueError) as exc:
                raise SourceIntegrityError(
                    f"invalid {sensor_kind} observation: {exc}"
                ) from exc
        result.sort(
            key=lambda item: (
                item.arrival_backend_monotonic_s,
                item.sample_id,
            )
        )
        return result

    def _build_event_feedback(self) -> list[ObservationSample]:
        """Build image-time feedback from the 100 Hz tick-bound feedback events.

        G2A publishes the joint_feedback SensorObservation at 10 Hz, while an
        RGB capture may reference any 100 Hz control tick.  The feedback event
        carries that exact tick and snapshot, so it is the authoritative source
        for D2 image-time proprioception.
        """

        segment = self._active_segment_id()
        result: list[ObservationSample] = []
        for event in self.events:
            if event.get("event_type") != "feedback":
                continue
            self._validate_row_identity(event)
            payload = event.get("payload") or {}
            tick = event.get("control_tick_id")
            if not isinstance(tick, str) or not tick:
                continue
            tick_number = _tick_number(tick)
            if tick_number is None:
                continue
            capture = payload.get("feedback_time_monotonic")
            if not isinstance(capture, (int, float)):
                capture = event.get("backend_monotonic_s")
            arrival = event.get("backend_monotonic_s")
            try:
                result.append(
                    ObservationSample(
                        sample_id=f"joint_feedback_{tick_number:09d}",
                        sensor_kind="joint_feedback",
                        stream_id="joint_feedback",
                        recording_segment_id=segment,
                        control_tick_id=tick,
                        capture_sim_time_s=_finite(payload.get("sim_time_s")),
                        capture_backend_monotonic_s=_finite(capture),
                        arrival_backend_monotonic_s=_finite(arrival),
                        value={
                            "model_joint_positions_rad": list(
                                payload.get("arm_model_qpos_rad") or []
                            ),
                            "gripper_width_m": payload.get("gripper_width_m"),
                        },
                        raw=event,
                        run_id=self.run_id,
                        episode_id=self.episode_id,
                    )
                )
            except (TypeError, ValueError) as exc:
                raise SourceIntegrityError(
                    f"invalid event feedback at {tick}: {exc}"
                ) from exc
        result.sort(
            key=lambda item: (
                item.arrival_backend_monotonic_s,
                item.sample_id,
            )
        )
        return result

    def _build_dispatches(self) -> list[ActionDispatch]:
        requested: dict[str, dict[str, Any]] = {}
        approved: dict[str, dict[str, Any]] = {}
        for event in self.events:
            command_id = event.get("command_id")
            if not isinstance(command_id, str):
                continue
            if event.get("event_type") == "requested":
                self._validate_row_identity(event)
                requested[command_id] = event
            elif event.get("event_type") == "approved":
                self._validate_row_identity(event)
                decision = (event.get("payload") or {}).get("decision") or {}
                if event.get("event_type") == "approved" and decision.get(
                    "approved", True
                ):
                    approved[command_id] = event

        dispatches: list[ActionDispatch] = []
        for event in self.events:
            if event.get("event_type") != "dispatch":
                continue
            self._validate_row_identity(event)
            command_id = event.get("command_id")
            if not isinstance(command_id, str):
                continue
            request_event = requested.get(command_id)
            approved_event = approved.get(command_id)
            if request_event is None or approved_event is None:
                continue
            request = (request_event.get("payload") or {}).get("request") or {}
            command_type = str(request.get("command_type", ""))
            group = dispatch_group(command_type)
            if group is None:
                continue
            try:
                dispatches.append(
                    ActionDispatch(
                        command_id=command_id,
                        command_type=command_type,
                        group=group,
                        backend_monotonic_s=_field_float(
                            event, "backend_monotonic_s"
                        ),
                        sim_time_s=_field_float(event, "sim_time_s"),
                        control_tick_id=event.get("control_tick_id"),
                        requested_event=request_event,
                        approved_event=approved_event,
                        dispatch_event=event,
                    )
                )
            except (TypeError, ValueError) as exc:
                raise SourceIntegrityError(
                    f"invalid dispatch {command_id}: {exc}"
                ) from exc
        dispatches.sort(
            key=lambda item: (
                item.backend_monotonic_s,
                item.command_id,
            )
        )
        return dispatches

    def _build_actuator_references(self) -> list[ActuatorReference]:
        result: list[ActuatorReference] = []
        for sequence, event in enumerate(self.events):
            if event.get("event_type") != "actuator_reference":
                continue
            self._validate_row_identity(event)
            try:
                result.append(
                    ActuatorReference(
                        sequence=sequence,
                        backend_monotonic_s=_field_float(
                            event, "backend_monotonic_s"
                        ),
                        sim_time_s=_field_float(event, "sim_time_s"),
                        control_tick_id=event.get("control_tick_id"),
                        payload=dict(event.get("payload") or {}),
                        raw=event,
                    )
                )
            except (TypeError, ValueError) as exc:
                raise SourceIntegrityError(
                    f"invalid actuator_reference event: {exc}"
                ) from exc
        result.sort(
            key=lambda item: (
                item.backend_monotonic_s,
                _tick_number(item.control_tick_id) or -1,
                item.sequence,
            )
        )
        return result

    def _validate_row_identity(self, row: Mapping[str, Any]) -> None:
        if row.get("run_id") != self.run_id:
            raise SourceIntegrityError("source row run_id mismatch")
        if row.get("episode_id") != self.episode_id:
            raise SourceIntegrityError("source row episode_id mismatch")

    def _active_segment_id(self) -> str:
        segments = self.episode.get("segments") or []
        if len(segments) != 1:
            raise SourceIntegrityError(
                "G2B v1 requires one immutable recording segment per source Episode"
            )
        segment_id = segments[0].get("recording_segment_id")
        if not isinstance(segment_id, str) or not segment_id:
            raise SourceIntegrityError("recording_segment_id missing")
        return segment_id

    def _segment_ranges(self) -> dict[str, dict[str, float]]:
        segments = self.episode.get("segments") or []
        if len(segments) != 1:
            raise SourceIntegrityError("unexpected source segment count")
        segment = segments[0]
        return {
            self._active_segment_id(): {
                "start_backend_monotonic_s": _field_float(
                    segment, "started_backend_monotonic_s"
                ),
                "start_sim_time_s": _finite(
                    self.recording_start.get("sim_time_s")
                ),
                "end_sim_time_s": _finite(self.recording_stop.get("sim_time_s")),
            }
        }

    @property
    def audit_status(self) -> str:
        return str(self.episode_complete.get("audit_status", "UNKNOWN"))

    @property
    def task_result(self) -> str:
        return str(
            self.assessment.get(
                "task_result",
                self.episode.get("result", "unknown"),
            )
        )

    @property
    def task_language(self) -> str:
        return str(self.episode.get("task_language", ""))

    @property
    def source(self) -> str:
        sources = {
            str(row.get("source"))
            for row in self.observation_index
            if row.get("sensor_kind") in {"rgb", "depth"}
        }
        return ",".join(sorted(sources))

    def identity(self) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "run_id": self.run_id,
            "episode_id": self.episode_id,
            "recording_segment_id": self._active_segment_id(),
            "clock_id": "backend_monotonic",
            "contract_version": self.episode.get("contract_version"),
            "operator_kind": self.episode.get("operator_kind"),
            "view_source": self.episode.get("view_source"),
            "source": self.source,
            "task_language": self.task_language,
            "task_result": self.task_result,
            "audit_status": self.audit_status,
            "recording_start": {
                "sim_time_s": self.recording_start.get("sim_time_s"),
                "backend_monotonic_s": self.recording_start.get(
                    "backend_monotonic_s"
                ),
                "control_tick_id": self.recording_start.get("control_tick_id"),
            },
            "recording_stop": {
                "sim_time_s": self.recording_stop.get("sim_time_s"),
                "backend_monotonic_s": self.recording_stop.get(
                    "backend_monotonic_s"
                ),
                "control_tick_id": self.recording_stop.get("control_tick_id"),
            },
            "counts": {
                "events": len(self.events),
                "commands": len(self.commands),
                "control": len(self.control),
                "observation_rows": len(self.observation_index),
                "persisted_rgbd": len(self.images),
                "joint_feedback": len(self.feedback),
                "joint_feedback_published": len(self.feedback_published),
                "encoders": len(self.encoders),
                "imu": len(self.imu),
                "dispatches": len(self.dispatch_intervals),
                "actuator_references": len(self.actuator_references),
            },
        }

    def source_file_hashes(self) -> dict[str, str]:
        result: dict[str, str] = {}
        for path in sorted(self.episode_dir.rglob("*")):
            if path.is_file() and not path.is_symlink():
                result[path.relative_to(self.episode_dir).as_posix()] = sha256_file(
                    path
                )
        return result

    def asset_hash(self, relative_uri: str) -> str:
        path = resolve_episode_media(self.episode_dir, relative_uri)
        return sha256_file(path)

    def check_required_media(self, images: Iterable[ImageSample]) -> list[str]:
        missing: list[str] = []
        for image in images:
            for uri in (image.rgb_uri, image.depth_uri, image.valid_mask_uri):
                try:
                    resolve_episode_media(self.episode_dir, uri)
                except SourceIntegrityError:
                    missing.append(f"{image.sample_id}:{uri}")
        return missing

    @property
    def event_counts(self) -> dict[str, int]:
        return dict(Counter(str(row.get("event_type")) for row in self.events))

    @property
    def observation_counts(self) -> dict[str, int]:
        return dict(
            Counter(str(row.get("sensor_kind")) for row in self.observation_index)
        )

    def references_by_backend(self) -> dict[str, list[ActuatorReference]]:
        by_segment: dict[str, list[ActuatorReference]] = defaultdict(list)
        for reference in self.actuator_references:
            by_segment[self._active_segment_id()].append(reference)
        return dict(by_segment)
