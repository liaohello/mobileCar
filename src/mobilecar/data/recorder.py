"""Bounded, durable native Episode recorder.

The recorder is the only owner of Episode bytes.  It accepts immutable public
events/observations, commits raw assets before publishing their references, and
never imports a simulator.
"""

from __future__ import annotations

import base64
import hashlib
import json
import os
import queue
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Iterable, Mapping
from uuid import uuid4

from ..contracts import (
    AuditStatus,
    EpisodeControlResponse,
    EpisodeStartRequest,
    EventRecord,
    RecordingState,
    RecordingStatus,
    RobotSnapshot,
    SensorKind,
    SensorObservation,
)

CONTRACT_VERSION = "mobilecar-1.0"
TERMINAL_EVENT = "episode_terminal"
COMPLETE_FILE = "episode_complete.json"
COMMAND_PREFIXES = (
    "requested",
    "approved",
    "rejected",
    "dispatch",
    "dispatched",
    "expired",
    "command_response",
)
CONTROL_TYPES = {
    "approved",
    "rejected",
    "dispatch",
    "dispatched",
    "expired",
    "actuator_reference",
    "feedback",
}
CRITICAL_TYPES = {
    "requested",
    "approved",
    "rejected",
    "dispatch",
    "dispatched",
    "expired",
    "command_response",
    "actuator_reference",
}


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _jsonable(value: Any) -> Any:
    """Convert public values to JSON without silently inventing measurements."""

    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, Mapping):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if hasattr(value, "tolist"):
        return _jsonable(value.tolist())
    if hasattr(value, "model_dump"):
        # Python mode preserves NumPy arrays until this recursive normalizer
        # can turn them into lists; JSON mode raises first.
        return _jsonable(value.model_dump(mode="python"))
    return repr(value)


def _snapshot_without_truth(snapshot: RobotSnapshot) -> dict[str, Any]:
    """Strip explicit world truth before a snapshot enters default Episode streams."""

    payload = _jsonable(snapshot.model_dump(mode="python"))
    diagnostics = payload.get("diagnostics")
    if isinstance(diagnostics, dict):
        for key in list(diagnostics):
            if "truth" in key.lower() or key in {"object_pose_world_sim", "cube_pose_world_sim"}:
                diagnostics.pop(key, None)
    return payload


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _fsync_directory(path: Path) -> None:
    try:
        descriptor = os.open(path, os.O_RDONLY)
    except OSError:
        return
    try:
        os.fsync(descriptor)
    except OSError:
        pass
    finally:
        os.close(descriptor)


def _atomic_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{uuid4().hex}.tmp")
    with temporary.open("w", encoding="utf-8") as handle:
        json.dump(_jsonable(payload), handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write("\n")
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)
    _fsync_directory(path.parent)


def _safe_relative(path: str) -> Path:
    value = Path(path)
    if value.is_absolute() or any(part == ".." for part in value.parts):
        raise ValueError("asset path must be a relative path without '..'")
    return value


def store_asset(episode_dir: str | Path, relative_path: str, data: bytes) -> dict[str, Any]:
    """Commit an asset with fsync before returning its durable reference."""

    root = Path(episode_dir).resolve()
    relative = _safe_relative(relative_path)
    target = (root / relative).resolve()
    if not target.is_relative_to(root):
        raise ValueError("asset escapes Episode directory")
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(f".{target.name}.{uuid4().hex}.tmp")
    with temporary.open("wb") as handle:
        handle.write(data)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, target)
    _fsync_directory(target.parent)
    return {
        "uri": relative.as_posix(),
        "sha256": hashlib.sha256(data).hexdigest(),
        "bytes": len(data),
    }


class _Writer:
    def __init__(self, run_dir: Path, episode_dir: Path, episode_id: str) -> None:
        self.run_dir = run_dir
        self.episode_dir = episode_dir
        self.episode_id = episode_id
        self.sequence = 0
        self.transform_samples: set[tuple[str, str]] = set()
        self._handles: dict[str, Any] = {}
        self.closed = False

    def next_sequence(self) -> int:
        self.sequence += 1
        return self.sequence

    def append(self, relative: str, record: Mapping[str, Any]) -> None:
        if self.closed:
            raise RuntimeError("Episode writer is closed")
        path = self.episode_dir / relative
        handle = self._handles.get(relative)
        if handle is None:
            path.parent.mkdir(parents=True, exist_ok=True)
            handle = path.open("a", encoding="utf-8", buffering=64 * 1024)
            self._handles[relative] = handle
        handle.write(json.dumps(_jsonable(record), ensure_ascii=False, separators=(",", ":")))
        handle.write("\n")

    def flush(self) -> None:
        for handle in self._handles.values():
            handle.flush()

    def sync(self) -> None:
        self.flush()
        for handle in self._handles.values():
            os.fsync(handle.fileno())

    def close(self) -> None:
        if self.closed:
            return
        self.flush()
        for handle in self._handles.values():
            try:
                os.fsync(handle.fileno())
            finally:
                handle.close()
        self._handles.clear()
        self.closed = True

    def write_event(self, event: EventRecord | Mapping[str, Any]) -> EventRecord:
        record = event if isinstance(event, EventRecord) else EventRecord.model_validate(event)
        if record.run_id != self.run_dir.name:
            raise ValueError("event run_id does not match active run")
        if record.episode_id != self.episode_id:
            raise ValueError("event episode_id does not match active Episode")
        if record.event_type in {"oracle_sample", "world_state_sample", "grasp_oracle_sample"}:
            # Truth-bearing evaluation data has a separate path from default
            # observations and does not consume the event append sequence.
            self.append(
                "assessments/oracle_samples.jsonl",
                {
                    **(record.payload or {}),
                    "run_id": record.run_id,
                    "episode_id": record.episode_id,
                    "backend_monotonic_s": record.backend_monotonic_s,
                    "sim_time_s": record.sim_time_s,
                    "source": "GT_DEBUG_ISOLATED",
                },
            )
            return record
        if record.event_seq is None:
            record = record.model_copy(update={"event_seq": self.next_sequence()})
        else:
            self.sequence = max(self.sequence, record.event_seq)
        payload = _jsonable(record.model_dump(mode="python"))
        self.append("events.jsonl", payload)
        event_type = str(record.event_type)
        if record.command_id or event_type in COMMAND_PREFIXES:
            self.append("commands.jsonl", payload)
        if event_type in CONTROL_TYPES:
            self.append("control.jsonl", payload)
        if event_type in {"transform", "transforms", "image_transforms"}:
            self.append("transforms.jsonl", payload)
        return record

    def write_observation(self, observation: SensorObservation | Mapping[str, Any]) -> SensorObservation:
        record = (
            observation
            if isinstance(observation, SensorObservation)
            else SensorObservation.model_validate(observation)
        )
        if record.run_id != self.run_dir.name or record.episode_id != self.episode_id:
            raise ValueError("observation identity does not match active Episode")
        kind = str(record.sensor_kind)
        payload = _jsonable(record.model_dump(mode="python"))
        self.append(f"observations/{kind}.jsonl", payload)
        self.append("observations/index.jsonl", payload)
        metadata = record.metadata or {}
        has_transforms = any(
            key.startswith("T_") or key in {"calibration_id", "rgbd_optical_convention", "intrinsics"}
            for key in metadata
        )
        key = (record.stream_id, record.sample_id)
        if has_transforms and key not in self.transform_samples:
            self.transform_samples.add(key)
            self.append(
                "transforms.jsonl",
                {
                    "contract_version": record.contract_version,
                    "run_id": record.run_id,
                    "episode_id": record.episode_id,
                    "stream_id": record.stream_id,
                    "sample_id": record.sample_id,
                    "measurement_time_s": record.measurement_time_s,
                    "arrival_time_s": record.arrival_time_s,
                    "clock_id": record.clock_id,
                    "frame_id": record.frame_id,
                    "source": record.source,
                    "metadata": metadata,
                },
            )
        return record


class Recorder:
    """Thread-safe EventSink with bounded queues and a durable Episode tail."""

    def __init__(
        self,
        root: str | Path | None = None,
        run_root: str | Path | None = None,
        *,
        queue_capacity: int = 4096,
        flush_interval_s: float = 0.05,
        close_drain_timeout_s: float = 10.0,
        fault_after_events: int | None = None,
        **_unused: Any,
    ) -> None:
        if queue_capacity < 16:
            raise ValueError("queue_capacity must be at least 16")
        if flush_interval_s <= 0 or close_drain_timeout_s <= 0:
            raise ValueError("flush and drain timeouts must be positive")
        self.root = Path(root or run_root or "runs").resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.queue_capacity = int(queue_capacity)
        self.flush_interval_s = float(flush_interval_s)
        self.close_drain_timeout_s = float(close_drain_timeout_s)
        self.fault_after_events = fault_after_events
        self._queue: queue.Queue[tuple[str, Any] | None] = queue.Queue(maxsize=queue_capacity)
        self._lock = threading.RLock()
        self._drain_lock = threading.Lock()
        self._failure_callbacks: list[Callable[[str, str], None]] = []
        self._run_dirs: dict[str, Path] = {}
        self._writers: dict[str, _Writer] = {}
        self._persist_pending: dict[tuple[str, str, str], dict[str, Any]] = {}
        self._persist_sequence: dict[str, int] = {}
        self._episode_dirs: dict[str, Path] = {}
        self._run_episodes: dict[str, str] = {}
        self._request: EpisodeStartRequest | None = None
        self._snapshot: RobotSnapshot | None = None
        self._episode_id: str | None = None
        self._segment_id: str | None = None
        self._status = RecordingStatus.IDLE
        self._capture_ready = False
        self._error: str | None = None
        self._dropped = 0
        self._processed = 0
        self._last_flush = time.monotonic()
        self._stopping = False
        self._closed = False
        self._worker = threading.Thread(target=self._drain, name="mobilecar-recorder", daemon=True)
        self._worker.start()

    # ------------------------------------------------------------------ sink

    def set_failure_callback(self, callback: Callable[[str, str], None]) -> None:
        with self._lock:
            self._failure_callbacks.append(callback)

    def publish(self, event: EventRecord | Mapping[str, Any]) -> None:
        record = event if isinstance(event, EventRecord) else EventRecord.model_validate(event)
        if not self._has_active_episode(record.episode_id):
            return
        self._enqueue("event", record, critical=str(record.event_type) in CRITICAL_TYPES)

    def publish_observation(
        self,
        observation: SensorObservation | Mapping[str, Any],
        *,
        asset_data: bytes | None = None,
    ) -> None:
        record = (
            observation
            if isinstance(observation, SensorObservation)
            else SensorObservation.model_validate(observation)
        )
        if not self._has_active_episode(record.episode_id):
            # Debug teleoperation may run without recording. There is no capture
            # stream to lose; once STARTING/RECORDING the same path stays strict.
            return
        record = self._commit_observation_assets(record, asset_data=asset_data)
        self._enqueue("observation", record, critical=False)

    def record_observation(self, observation: SensorObservation | Mapping[str, Any], **kwargs: Any) -> None:
        self.publish_observation(observation, **kwargs)

    def recording_gate(self, episode_id: str) -> bool:
        with self._lock:
            return bool(
                self._status == RecordingStatus.RECORDING
                and self._capture_ready
                and self._episode_id == episode_id
                and self._error is None
            )

    def status(self) -> RecordingState:
        with self._lock:
            return RecordingState(
                status=self._status,
                episode_id=self._episode_id,
                recording_segment_id=self._segment_id,
                command_capture_ready=self._capture_ready,
                queued_events=self._queue.qsize(),
                dropped_events=self._dropped,
                error=self._error,
            )

    # -------------------------------------------------------------- lifecycle

    def start(
        self,
        request: EpisodeStartRequest | Mapping[str, Any],
        snapshot: RobotSnapshot | Mapping[str, Any],
    ) -> EpisodeControlResponse:
        request_model = (
            request if isinstance(request, EpisodeStartRequest) else EpisodeStartRequest.model_validate(request)
        )
        snapshot_model = snapshot if isinstance(snapshot, RobotSnapshot) else RobotSnapshot.model_validate(snapshot)
        with self._lock:
            if self._closed:
                return EpisodeControlResponse(accepted=False, reason="recorder is closed")
            if self._status in {RecordingStatus.STARTING, RecordingStatus.RECORDING, RecordingStatus.DRAINING}:
                return EpisodeControlResponse(
                    accepted=False,
                    reason=f"recording is already {self._status}",
                    episode_id=self._episode_id,
                    recording_segment_id=self._segment_id,
                    recording_status=self._status,
                )
            self._status = RecordingStatus.STARTING
            self._error = None
            self._capture_ready = False
            self._dropped = 0
            try:
                writer, episode_dir = self._begin_episode(request_model, snapshot_model)
                self._request = request_model
                self._snapshot = snapshot_model
                self._episode_id = snapshot_model.episode_id
                started = _utc_now()
                start_event = EventRecord(
                    run_id=snapshot_model.run_id,
                    episode_id=snapshot_model.episode_id,
                    event_type="recording_start",
                    wall_utc=started,
                    backend_monotonic_s=snapshot_model.backend_monotonic_s,
                    sim_time_s=snapshot_model.sim_time_s,
                    control_tick_id=snapshot_model.control_tick_id,
                    payload={
                        "recording_segment_id": self._segment_id,
                        "operator_id": request_model.operator_id,
                        "operator_kind": str(request_model.operator_kind),
                        "view_source": request_model.view_source,
                    },
                )
                writer.write_event(start_event)
                initial_event = EventRecord(
                    run_id=snapshot_model.run_id,
                    episode_id=snapshot_model.episode_id,
                    event_type="feedback_initial",
                    wall_utc=started,
                    backend_monotonic_s=snapshot_model.backend_monotonic_s,
                    sim_time_s=snapshot_model.sim_time_s,
                    control_tick_id=snapshot_model.control_tick_id,
                    payload={
                        "snapshot": self._snapshot_default_payload(
                            snapshot_model, episode_dir
                        )
                    },
                )
                writer.write_event(initial_event)
                self._write_snapshot_oracle_sample(writer, snapshot_model)
                _atomic_json(
                    episode_dir / "initial_snapshot.json",
                    self._snapshot_default_payload(snapshot_model, episode_dir),
                )
                # STARTING -> RECORDING only after every start record is durable.
                writer.sync()
                self._capture_ready = True
                self._status = RecordingStatus.RECORDING
            except Exception as exc:
                self._status = RecordingStatus.FAILED
                self._error = f"episode start failed: {exc}"
                return EpisodeControlResponse(accepted=False, reason=self._error)
            return EpisodeControlResponse(
                accepted=True,
                episode_id=self._episode_id,
                recording_segment_id=self._segment_id,
                recording_status=self._status,
            )

    def stop(
        self,
        *,
        reason: str = "OPERATOR_FINISHED",
        final_snapshot: RobotSnapshot | Mapping[str, Any] | None = None,
    ) -> EpisodeControlResponse:
        with self._lock:
            if self._episode_id is None:
                return EpisodeControlResponse(accepted=False, reason="no active Episode")
            if self._status not in {RecordingStatus.RECORDING, RecordingStatus.FAILED}:
                return EpisodeControlResponse(
                    accepted=self._status == RecordingStatus.COMPLETE,
                    reason=f"recording is {self._status}",
                    episode_id=self._episode_id,
                    recording_segment_id=self._segment_id,
                    recording_status=self._status,
                )
            episode_id = self._episode_id
            failing = self._status == RecordingStatus.FAILED
            self._status = RecordingStatus.DRAINING
            self._capture_ready = False
            self._stopping = True
        try:
            snapshot = (
                final_snapshot
                if isinstance(final_snapshot, RobotSnapshot)
                else RobotSnapshot.model_validate(final_snapshot)
                if final_snapshot is not None
                else self._snapshot
            )
            if snapshot is not None:
                self._write_snapshot_oracle_sample(self._writers[episode_id], snapshot)
                event = EventRecord(
                    run_id=snapshot.run_id,
                    episode_id=episode_id,
                    event_type="recording_stop",
                    wall_utc=_utc_now(),
                    backend_monotonic_s=snapshot.backend_monotonic_s,
                    sim_time_s=snapshot.sim_time_s,
                    control_tick_id=snapshot.control_tick_id,
                    payload={
                        "reason": reason,
                        "snapshot": self._snapshot_default_payload(
                            snapshot, self._episode_dirs[episode_id]
                        ),
                    },
                )
                self._enqueue("event", event, critical=True, allow_stopping=True)
            if not self._wait_drained(self.close_drain_timeout_s):
                raise TimeoutError("recorder queue did not drain before timeout")
            self._finalize_episode(reason, failing=failing or self._error is not None)
        except Exception as exc:
            with self._lock:
                self._status = RecordingStatus.FAILED
                self._error = f"episode stop/drain failed: {exc}"
                self._capture_ready = False
            return EpisodeControlResponse(
                accepted=False,
                reason=self._error,
                episode_id=episode_id,
                recording_segment_id=self._segment_id,
                recording_status=self._status,
            )
        with self._lock:
            self._stopping = False
            self._status = RecordingStatus.FAILED if self._error else RecordingStatus.COMPLETE
            return EpisodeControlResponse(
                accepted=self._status == RecordingStatus.COMPLETE,
                reason=self._error or "",
                episode_id=episode_id,
                recording_segment_id=self._segment_id,
                recording_status=self._status,
            )

    def validate(self, episode_id: str):
        from .validator import validate_episode

        run_id = self._run_episodes.get(episode_id)
        return validate_episode(self.root, episode_id=episode_id, run_id=run_id, write_report=True)

    def close(self, timeout_s: float | None = None) -> None:
        with self._lock:
            if self._closed:
                return
            active = self._status in {
                RecordingStatus.STARTING,
                RecordingStatus.RECORDING,
                RecordingStatus.DRAINING,
            }
        if active:
            self.stop(
                reason="BACKEND_CLOSED",
                final_snapshot=self.snapshot(),
            )
        self._wait_drained(timeout_s or self.close_drain_timeout_s)
        with self._lock:
            self._closed = True
        self._queue.put(None)
        self._worker.join(timeout=timeout_s or self.close_drain_timeout_s)

    def snapshot(self) -> RobotSnapshot | None:
        with self._lock:
            return self._snapshot

    def _has_active_episode(self, episode_id: str) -> bool:
        with self._lock:
            return bool(
                self._episode_id == episode_id
                and self._status
                in {
                    RecordingStatus.STARTING,
                    RecordingStatus.RECORDING,
                    RecordingStatus.DRAINING,
                }
            )

    @staticmethod
    def _write_snapshot_oracle_sample(writer: _Writer, snapshot: RobotSnapshot) -> None:
        truth = snapshot.diagnostics.get("object_pose_world_sim")
        if not isinstance(truth, Mapping):
            return
        position = truth.get("position_m")
        if not isinstance(position, list) or len(position) != 3:
            return
        writer.append(
            "assessments/oracle_samples.jsonl",
            {
                "run_id": snapshot.run_id,
                "episode_id": snapshot.episode_id,
                "backend_monotonic_s": snapshot.backend_monotonic_s,
                "sim_time_s": snapshot.sim_time_s,
                "cube_position_world_sim_m": position,
                "left_contact_force_n": snapshot.gripper.left_contact_force_n,
                "right_contact_force_n": snapshot.gripper.right_contact_force_n,
                "bilateral_contact": snapshot.gripper.bilateral_contact,
                "source": "GT_DEBUG_SNAPSHOT_BOUNDARY",
            },
        )

    def _snapshot_default_payload(
        self,
        snapshot: RobotSnapshot,
        episode_dir: Path,
    ) -> dict[str, Any]:
        """Materialize snapshot assets and remove truth from a default stream."""

        payload = _snapshot_without_truth(snapshot)
        sensors = payload.get("sensors")
        if not isinstance(sensors, list):
            return payload
        for sensor in sensors:
            if not isinstance(sensor, dict):
                continue
            uri = sensor.get("asset_uri")
            if not isinstance(uri, str) or not uri:
                continue
            committed = self._copy_incoming_asset(
                episode_dir,
                snapshot.run_id,
                uri,
                str(sensor.get("kind", "unknown")),
            )
            if committed is not None:
                sensor["asset_uri"] = committed["uri"]
        return payload

    # ---------------------------------------------------------------- helpers

    def _begin_episode(
        self,
        request: EpisodeStartRequest,
        snapshot: RobotSnapshot,
    ) -> tuple[_Writer, Path]:
        run_dir = self.root / snapshot.run_id
        run_dir.mkdir(parents=True, exist_ok=True)
        manifest_path = run_dir / "manifest.json"
        manifest = {
            "contract_version": CONTRACT_VERSION,
            "run_id": snapshot.run_id,
            "created_wall_utc": _utc_now(),
            "software": {"name": "mobilecar", "version": "0.1.0"},
            "clocks": ["sim", "backend_monotonic", "device", "wall_utc"],
            "model_id": snapshot.diagnostics.get("model_id"),
            "model_sha256": snapshot.diagnostics.get("model_sha256"),
            "scene_id": snapshot.diagnostics.get("scene_id"),
            "map_mode": request.map_mode,
            "truth_access": "GT_DEBUG_SEPARATE_FROM_DEFAULT_OBSERVATION",
        }
        if manifest_path.exists():
            existing = json.loads(manifest_path.read_text(encoding="utf-8"))
            if existing.get("run_id") != snapshot.run_id:
                raise ValueError("manifest run_id mismatch")
        else:
            _atomic_json(manifest_path, manifest)

        episode_dir = run_dir / "episodes" / snapshot.episode_id
        for child in ("assets/rgb", "assets/depth", "assets/masks", "assessments", "observations"):
            (episode_dir / child).mkdir(parents=True, exist_ok=True)
        if request.map_mode != "unavailable":
            (episode_dir / "maps").mkdir(parents=True, exist_ok=True)
        episode_path = episode_dir / "episode.json"
        if episode_path.exists():
            metadata = json.loads(episode_path.read_text(encoding="utf-8"))
            if metadata.get("contract_version") != CONTRACT_VERSION:
                raise ValueError("existing Episode contract version mismatch")
        else:
            metadata = {
                "contract_version": CONTRACT_VERSION,
                "run_id": snapshot.run_id,
                "episode_id": snapshot.episode_id,
                "task_language": request.task_language,
                "operator_id": request.operator_id,
                "operator_kind": str(request.operator_kind),
                "view_source": request.view_source,
                "map_mode": request.map_mode,
                "require_recording": request.require_recording,
                "created_wall_utc": _utc_now(),
                "segments": [],
                "result": "unknown",
            }
        segment_number = len(metadata.get("segments", [])) + 1
        segment_id = f"seg_{segment_number:03d}_{uuid4().hex[:10]}"
        previous_segment = metadata.get("segments", [])[-1]["recording_segment_id"] if metadata.get("segments") else None
        metadata.setdefault("segments", []).append(
            {
                "recording_segment_id": segment_id,
                "previous_recording_segment_id": previous_segment,
                "started_wall_utc": _utc_now(),
                "started_backend_monotonic_s": snapshot.backend_monotonic_s,
                "status": "RECORDING",
                "reason": None,
            }
        )
        _atomic_json(episode_path, metadata)
        writer = _Writer(run_dir, episode_dir, snapshot.episode_id)
        self._run_dirs[snapshot.run_id] = run_dir
        self._writers[snapshot.episode_id] = writer
        self._episode_dirs[snapshot.episode_id] = episode_dir
        self._run_episodes[snapshot.episode_id] = snapshot.run_id
        self._segment_id = segment_id
        return writer, episode_dir

    def _commit_observation_assets(
        self,
        observation: SensorObservation,
        *,
        asset_data: bytes | None,
    ) -> SensorObservation:
        episode_id = observation.episode_id
        # If the observation arrives during STARTING, it has no reliable episode
        # writer yet. Reject rather than lose a referenced frame silently.
        with self._lock:
            episode_dir = self._episode_dirs.get(episode_id)
        if episode_dir is None:
            episode_dir = self.root / observation.run_id / "episodes" / episode_id
            if not episode_dir.is_dir():
                raise RuntimeError("observation arrived before durable Episode start")
        update: dict[str, Any] = {}
        metadata = dict(observation.metadata)
        # The backend may hand off an immutable file in its own run-owned
        # incoming directory. Copying it into the Episode is part of the
        # recorder's ownership boundary, not an optional export step.
        if asset_data is None and observation.asset_uri:
            committed = self._copy_incoming_asset(
                episode_dir,
                observation.run_id,
                observation.asset_uri,
                str(observation.sensor_kind),
            )
            if committed is not None:
                update["asset_uri"] = committed["uri"]
                metadata["asset_sha256"] = committed["sha256"]
                metadata["asset_bytes"] = committed["bytes"]
                for key in ("rgb_path", "depth_path", "path"):
                    if key in metadata and metadata[key] == observation.asset_uri:
                        metadata[key] = committed["uri"]
        if asset_data is None and metadata.get("asset_data_b64"):
            asset_data = base64.b64decode(metadata.pop("asset_data_b64"), validate=True)
        if asset_data is not None:
            suffix = {
                SensorKind.RGB: ".png",
                SensorKind.DEPTH: ".npy",
                SensorKind.LIDAR: ".npy",
                SensorKind.IMU: ".json",
                SensorKind.ENCODERS: ".json",
                SensorKind.JOINT_FEEDBACK: ".json",
                SensorKind.IMAGE_TRANSFORMS: ".json",
            }.get(SensorKind(observation.sensor_kind), ".bin")
            relative = observation.asset_uri or f"assets/{observation.sensor_kind}/{observation.sample_id}{suffix}"
            committed = store_asset(episode_dir, relative, asset_data)
            update["asset_uri"] = committed["uri"]
            metadata["asset_sha256"] = committed["sha256"]
            metadata["asset_bytes"] = committed["bytes"]
        mask_data = metadata.pop("valid_mask_data_b64", None)
        if mask_data:
            relative = observation.valid_mask_uri or f"assets/masks/{observation.sample_id}.png"
            committed = store_asset(episode_dir, relative, base64.b64decode(mask_data, validate=True))
            update["valid_mask_uri"] = committed["uri"]
            metadata["valid_mask_sha256"] = committed["sha256"]
        elif observation.valid_mask_uri:
            committed = self._copy_incoming_asset(
                episode_dir,
                observation.run_id,
                observation.valid_mask_uri,
                "masks",
            )
            if committed is not None:
                update["valid_mask_uri"] = committed["uri"]
                metadata["valid_mask_sha256"] = committed["sha256"]
                if metadata.get("valid_mask_path") == observation.valid_mask_uri:
                    metadata["valid_mask_path"] = committed["uri"]
        if metadata != observation.metadata:
            update["metadata"] = metadata
        return observation.model_copy(update=update) if update else observation

    def _copy_incoming_asset(
        self,
        episode_dir: Path,
        run_id: str,
        uri: str,
        category: str,
    ) -> dict[str, Any] | None:
        """Copy a backend-owned run file into the Episode asset tree."""

        candidate = Path(uri)
        if candidate.is_absolute():
            return None
        run_dir = (self.root / run_id).resolve()
        options = ((self.root / candidate).resolve(), (run_dir / candidate).resolve())
        path = next(
            (option for option in options if option.is_relative_to(run_dir) and option.is_file()),
            None,
        )
        if path is None:
            return None
        episode_root = episode_dir.resolve()
        if path.is_relative_to(episode_root):
            return None
        target_uri = f"assets/{category}/{path.name}"
        return store_asset(episode_dir, target_uri, path.read_bytes())

    def _enqueue(
        self,
        kind: str,
        item: Any,
        *,
        critical: bool,
        allow_stopping: bool = False,
    ) -> None:
        with self._lock:
            if self._closed or (self._stopping and not allow_stopping):
                return
            if self._status not in {RecordingStatus.RECORDING, RecordingStatus.DRAINING, RecordingStatus.FAILED}:
                return
        try:
            self._queue.put_nowait((kind, item))
        except queue.Full:
            with self._lock:
                self._dropped += 1
            if critical:
                self._fail("recording queue overflow on control event", "QUEUE_OVERFLOW")
            return

    def _drain(self) -> None:
        while True:
            item = self._queue.get()
            try:
                if item is None:
                    return
                kind, record = item
                with self._lock:
                    writer = self._writers.get(record.episode_id)
                    failing = self._status == RecordingStatus.FAILED and self._error is not None
                if writer is not None and not failing:
                    if kind == "event":
                        writer.write_event(record)
                    elif kind == "observation":
                        written = writer.write_observation(record)
                        self._record_persist(writer, written)
                    self._processed += 1
                    if time.monotonic() - self._last_flush >= self.flush_interval_s:
                        writer.flush()
                        self._last_flush = time.monotonic()
                    if self.fault_after_events is not None and self._processed >= self.fault_after_events:
                        self._fail("injected recorder write failure", "INJECTED_WRITE_FAULT")
            except Exception as exc:
                self._fail(f"recorder write failed: {exc}", "WRITE_FAILED")
            finally:
                self._queue.task_done()
                time.sleep(0)

    def _record_persist(self, writer: _Writer, observation: SensorObservation) -> None:
        """Record durable RGB/Depth pair completion after fsync."""

        kind = str(observation.sensor_kind)
        if kind not in {"rgb", "depth"}:
            return
        key = (observation.episode_id, observation.stream_id, observation.sample_id)
        with self._lock:
            pending = self._persist_pending.setdefault(key, {})
            pending[kind] = observation.model_dump(mode="python")
        if set(pending) != {"rgb", "depth"}:
            return
        # The observation JSONL and committed assets must be durable before a
        # persist timestamp may be published.
        writer.sync()
        persist_time = time.monotonic()
        rgb = pending["rgb"]
        depth = pending["depth"]
        arrivals = [
            float(item["arrival_time_s"])
            for item in (rgb, depth)
            if isinstance(item.get("arrival_time_s"), (int, float))
        ]
        captures = [
            float(item["capture_backend_monotonic_s"])
            for item in (rgb, depth)
            if isinstance(item.get("capture_backend_monotonic_s"), (int, float))
        ]
        with self._lock:
            sequence = self._persist_sequence.get(observation.episode_id, 0) + 1
            self._persist_sequence[observation.episode_id] = sequence
        record = {
            "interface_profile": "g2a-1.0",
            "run_id": observation.run_id,
            "episode_id": observation.episode_id,
            "stream_id": observation.stream_id,
            "sample_id": observation.sample_id,
            "kinds": ["rgb", "depth"],
            "persist_sequence": sequence,
            "persist_backend_monotonic_s": persist_time,
            "arrival_backend_monotonic_s": max(arrivals) if arrivals else None,
            "capture_backend_monotonic_s": min(captures) if captures else None,
            "capture_sim_time_s": rgb.get("capture_sim_time_s"),
            "capture_control_tick_id": rgb.get("capture_control_tick_id"),
            "feedback_sample_id": rgb.get("feedback_sample_id"),
            "arrival_to_persist_ms": (
                (persist_time - max(arrivals)) * 1000.0 if arrivals else None
            ),
            "capture_to_persist_ms": (
                (persist_time - min(captures)) * 1000.0 if captures else None
            ),
            "rgb_uri": rgb.get("asset_uri"),
            "depth_uri": depth.get("asset_uri"),
            "valid_mask_uri": depth.get("valid_mask_uri"),
            "profile_id": (rgb.get("metadata") or {}).get("profile_id"),
            "profile_version": (rgb.get("metadata") or {}).get("profile_version"),
            "profile_sha256": (rgb.get("metadata") or {}).get("profile_sha256"),
        }
        writer.append("observations/persist.jsonl", record)
        writer.sync()
        with self._lock:
            self._persist_pending.pop(key, None)

    def _wait_drained(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while self._queue.unfinished_tasks and time.monotonic() < deadline:
            time.sleep(0.005)
        return self._queue.unfinished_tasks == 0

    def _finalize_episode(self, reason: str, *, failing: bool) -> None:
        with self._lock:
            if self._episode_id is None or self._segment_id is None:
                raise ValueError("no Episode to finalize")
            episode_id = self._episode_id
            segment_id = self._segment_id
            episode_dir = self._episode_dirs[episode_id]
            writer = self._writers[episode_id]
            run_dir = self._run_dirs[writer.run_dir.name]
        terminal = EventRecord(
            run_id=run_dir.name,
            episode_id=episode_id,
            event_type=TERMINAL_EVENT,
            wall_utc=_utc_now(),
            payload={
                "recording_segment_id": segment_id,
                "termination_reason": reason,
                "recording_status": "FAILED" if failing else "COMPLETE",
                "audit_status": "INCOMPLETE" if failing else "COMPLETE",
            },
        )
        writer.write_event(terminal)
        writer.close()
        checksums: dict[str, str] = {}
        for path in sorted(episode_dir.rglob("*")):
            if path.is_file() and path.name != COMPLETE_FILE and not path.name.startswith("."):
                checksums[path.relative_to(episode_dir).as_posix()] = _sha256_file(path)
        complete = {
            "contract_version": CONTRACT_VERSION,
            "run_id": writer.run_dir.name,
            "episode_id": episode_id,
            "recording_segment_id": segment_id,
            "termination_reason": reason,
            "audit_status": "INCOMPLETE" if failing else "COMPLETE",
            "closed_wall_utc": _utc_now(),
            "checksums": checksums,
            "event_count": writer.sequence,
        }
        _atomic_json(episode_dir / COMPLETE_FILE, complete)
        episode_path = episode_dir / "episode.json"
        metadata = json.loads(episode_path.read_text(encoding="utf-8"))
        for segment in metadata.get("segments", []):
            if segment.get("recording_segment_id") == segment_id:
                segment.update(
                    {
                        "ended_wall_utc": _utc_now(),
                        "status": "FAILED" if failing else "COMPLETE",
                        "reason": reason,
                        "termination_reason": reason,
                    }
                )
        metadata["ended_wall_utc"] = _utc_now()
        metadata["termination_reason"] = reason
        metadata["recording_status"] = "FAILED" if failing else "COMPLETE"
        metadata["audit_status"] = "INCOMPLETE" if failing else "COMPLETE"
        _atomic_json(episode_path, metadata)
        quality = {
            "contract_version": CONTRACT_VERSION,
            "episode_id": episode_id,
            "audit_status": AuditStatus.INCOMPLETE.value if failing else AuditStatus.PENDING.value,
            "recording_status": "FAILED" if failing else "COMPLETE",
            "termination_reason": reason,
            "event_count": writer.sequence,
            "queued_events": self._queue.qsize(),
            "dropped_events": self._dropped,
            "generated_wall_utc": _utc_now(),
        }
        _atomic_json(episode_dir / "quality_report.json", quality)

    def _fail(self, message: str, code: str) -> None:
        callbacks: list[Callable[[str, str], None]]
        with self._lock:
            if self._error is None:
                self._error = message
                self._status = RecordingStatus.FAILED
                self._capture_ready = False
                callbacks = list(self._failure_callbacks)
            else:
                callbacks = []
        for callback in callbacks:
            try:
                callback(code, message)
            except Exception:
                pass


EpisodeRecorder = Recorder
