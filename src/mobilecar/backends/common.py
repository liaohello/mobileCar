"""Backend identity, event and snapshot helpers."""

from __future__ import annotations

import hashlib
import math
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from uuid import uuid4

from ..contracts.models import (
    CommandRequest,
    EventRecord,
    OperatorKind,
    RecordingState,
    RobotSnapshot,
)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def new_id(prefix: str) -> str:
    return f"{prefix}_{uuid4().hex}"


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def quat_wxyz_to_xyzw(values) -> list[float]:
    w, x, y, z = (float(value) for value in values)
    return [x, y, z, w]


def quat_xyzw_to_wxyz(values) -> list[float]:
    x, y, z, w = (float(value) for value in values)
    return [w, x, y, z]


def quat_angle(a, b) -> float:
    first = quat_wxyz_to_xyzw(a)
    second = quat_wxyz_to_xyzw(b)
    dot = abs(sum(x * y for x, y in zip(first, second)))
    return float(2.0 * math.acos(min(1.0, max(-1.0, dot))))


class EventPublisher:
    def __init__(self, run_id: str, episode_getter) -> None:
        self.run_id = run_id
        self._episode_getter = episode_getter
        self._sink = None
        self._lock = threading.Lock()

    def set_sink(self, sink) -> None:
        with self._lock:
            self._sink = sink

    @property
    def sink(self):
        return self._sink

    def emit(
        self,
        event_type: str,
        payload: dict[str, Any] | None = None,
        *,
        command: CommandRequest | None = None,
        backend_monotonic_s: float | None = None,
        sim_time_s: float | None = None,
        control_tick_id: str | None = None,
        plan_id: str | None = None,
        task_phase: str | None = None,
    ) -> EventRecord:
        event_payload = dict(payload or {})
        if command is not None:
            command_envelope = command.model_dump(mode="json")
            if event_type == "requested":
                event_payload["request"] = command_envelope
            else:
                event_payload.setdefault(
                    "command",
                    {
                        "command_id": command.command_id,
                        "command_seq": command.command_seq,
                        "session_id": command.session_id,
                        "operator_id": command.operator_id,
                        "operator_kind": command.operator_kind,
                        "episode_id": command.episode_id,
                        "backend_generation": command.backend_generation,
                        "requested_ttl_s": command.requested_ttl_s,
                    },
                )
        event = EventRecord(
            run_id=self.run_id,
            episode_id=self._episode_getter(),
            event_type=event_type,
            wall_utc=utc_now(),
            backend_monotonic_s=time.monotonic() if backend_monotonic_s is None else backend_monotonic_s,
            sim_time_s=sim_time_s,
            command_id=None if command is None else command.command_id,
            control_tick_id=control_tick_id,
            plan_id=plan_id,
            task_phase=task_phase,
            payload=event_payload,
        )
        sink = self._sink
        if sink is not None:
            try:
                sink.publish(event)
            except Exception:
                # The application observes and escalates recorder failures through
                # its sink. A backend event must never deadlock the physics thread.
                pass
        return event

    def publish_navigation(self, observation) -> None:
        """Forward a versioned navigation capture without blocking physics."""

        sink = self._sink
        method = getattr(sink, "publish_navigation", None)
        if method is not None:
            try:
                method(observation)
            except Exception:
                # Navigation health owns and reports consumer failures. The
                # physics thread must retain its single non-blocking boundary.
                pass

    def recording_gate(self, episode_id: str) -> bool:
        sink = self._sink
        return bool(sink is not None and sink.recording_gate(episode_id))


def minimal_snapshot(
    *,
    run_id: str,
    generation: int,
    episode_id: str,
    started: float,
    sim_time: float,
    base_position,
    base_quaternion_xyzw,
    tcp_position,
) -> RobotSnapshot:
    now = time.monotonic()
    return RobotSnapshot(
        run_id=run_id,
        backend_generation=generation,
        episode_id=episode_id,
        episode_started_backend_monotonic_s=started,
        backend_monotonic_s=now,
        sim_time_s=sim_time,
        feedback_time_s=now,
        wall_utc=utc_now(),
        control_tick_id="mock_0",
        base={
            "position_world_sim_m": [float(v) for v in base_position],
            "orientation_world_sim_xyzw": [float(v) for v in base_quaternion_xyzw],
            "vx_m_s": 0.0,
            "vy_m_s": 0.0,
            "wz_rad_s": 0.0,
            "stopped": True,
        },
        arm={
            "user_joint_positions_rad": [0.0] * 5,
            "user_joint_targets_rad": [0.0] * 5,
            "model_joint_positions_rad": [0.0, 0.0, 0.0, -math.pi / 2, 0.0],
            "model_joint_targets_rad": [0.0, 0.0, 0.0, -math.pi / 2, 0.0],
            "joint_velocities_rad_s": [0.0] * 5,
            "joint_torques_nm": [0.0] * 5,
            "joint_margin_rad": [2.09] * 5,
            "model_limits_rad": [[-2.09, 2.09]] * 5,
            "coordinates_status": "mock_fixture",
        },
        gripper={
            "width_m": 0.025,
            "target_width_m": 0.025,
            "model_drive_position_rad": -1.0776326566,
            "model_drive_target_rad": -1.0776326566,
        },
        tcp={
            "position_m": [float(v) for v in tcp_position],
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        },
        sensors=[],
        recording=RecordingState(),
    )
