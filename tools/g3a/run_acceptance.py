#!/usr/bin/env python3
"""Server-only G3A single-instance physical acceptance."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import signal
import socket
import subprocess
import sys
import threading
import time
import traceback
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from uuid import uuid4
from xml.sax.saxutils import quoteattr

import httpx
import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from mobilecar.contracts.navigation import NavigationControlMessage  # noqa: E402


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def atomic_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{uuid4().hex}.tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def free_port() -> int:
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def quaternion_matrix_xyzw(values: list[float]) -> np.ndarray:
    x, y, z, w = (float(value) for value in values)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("invalid quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def yaw_from_quaternion(values: list[float]) -> float:
    rotation = quaternion_matrix_xyzw(values)
    return float(math.atan2(rotation[1, 0], rotation[0, 0]))


def angle_delta(first: float, second: float) -> float:
    return float(math.atan2(math.sin(second - first), math.cos(second - first)))


@dataclass
class CaseResult:
    name: str
    passed: bool
    duration_s: float
    started_wall_utc: str
    details: dict[str, Any]
    error: str | None = None

    def as_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "passed": self.passed,
            "duration_s": self.duration_s,
            "started_wall_utc": self.started_wall_utc,
            "details": self.details,
            "error": self.error,
        }


class AppProcess:
    def __init__(self, config: Path, report_root: Path) -> None:
        self.port = free_port()
        self.log_path = report_root / "mobilecar_server.log"
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log_handle = self.log_path.open("w", encoding="utf-8")
        environment = os.environ.copy()
        environment.update(
            {
                "MUJOCO_GL": "egl",
                "PYOPENGL_PLATFORM": "egl",
                "PYTHONPATH": str(SRC),
                "ROS_DOMAIN_ID": "73",
                "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
            }
        )
        self.process = subprocess.Popen(
            [
                sys.executable,
                "-m",
                "mobilecar",
                "--config",
                str(config),
                "--host",
                "127.0.0.1",
                "--port",
                str(self.port),
            ],
            cwd=ROOT,
            env=environment,
            stdin=subprocess.DEVNULL,
            stdout=self.log_handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            text=True,
        )

    @property
    def url(self) -> str:
        return f"http://127.0.0.1:{self.port}"

    def wait_ready(self, timeout_s: float = 30.0) -> dict[str, Any]:
        deadline = time.monotonic() + timeout_s
        last_error: Exception | None = None
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                break
            try:
                response = httpx.get(f"{self.url}/api/v1/health", timeout=1.0)
                if response.status_code == 200:
                    return response.json()
            except Exception as exc:
                last_error = exc
            time.sleep(0.1)
        tail = self.log_path.read_text(encoding="utf-8", errors="replace")[-8000:]
        raise RuntimeError(
            f"application did not become ready: exit={self.process.poll()} "
            f"error={last_error}\n{tail}"
        )

    def close(self) -> None:
        if self.process.poll() is None:
            try:
                os.killpg(self.process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                self.process.wait(timeout=20.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(self.process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    self.process.wait(timeout=10.0)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(self.process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    self.process.wait(timeout=5.0)
        self.log_handle.close()


class HttpCommands:
    def __init__(
        self,
        client: httpx.Client,
        *,
        operator_id: str,
        operator_kind: str = "agent_ui",
        view_source: str = "g3a_server_acceptance",
    ) -> None:
        self.client = client
        self.operator_id = operator_id
        self.operator_kind = operator_kind
        self.view_source = view_source
        self.session_id = f"session_{uuid4().hex}"
        self.sequence = 0

    def submit(
        self,
        command_type: str,
        payload: dict[str, Any],
        *,
        ttl_s: float = 2.0,
    ) -> dict[str, Any]:
        snapshot = get_json(self.client, "/api/v1/snapshot")
        self.sequence += 1
        request = {
            "contract_version": "mobilecar-1.0",
            "run_id": snapshot["run_id"],
            "backend_generation": snapshot["backend_generation"],
            "episode_id": snapshot["episode_id"],
            "session_id": self.session_id,
            "operator_id": self.operator_id,
            "operator_kind": self.operator_kind,
            "view_source": self.view_source,
            "command_id": f"g3a_{uuid4().hex}",
            "command_seq": self.sequence,
            "command_type": command_type,
            "requested_ttl_s": ttl_s,
            "payload": payload,
        }
        response = self.client.post("/api/v1/commands", json=request)
        response.raise_for_status()
        return response.json()


class NavSocketClient:
    def __init__(self, path: Path, report_root: Path) -> None:
        self.path = path
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.socket.settimeout(1.0)
        self.socket.connect(str(path))
        self.socket.settimeout(None)
        self._buffer = bytearray()
        self._closed = threading.Event()
        self._condition = threading.Condition()
        self.envelopes: deque[dict[str, Any]] = deque(maxlen=10000)
        self.trace_path = report_root / "ipc_stream.ndjson"
        self.trace_path.parent.mkdir(parents=True, exist_ok=True)
        self.trace = self.trace_path.open("w", encoding="utf-8")
        self.reader = threading.Thread(
            target=self._read_loop,
            name="g3a-acceptance-ipc-reader",
            daemon=True,
        )
        self.reader.start()

    def _read_loop(self) -> None:
        while not self._closed.is_set():
            try:
                chunk = self.socket.recv(262144)
            except OSError:
                break
            if not chunk:
                break
            self._buffer.extend(chunk)
            while True:
                index = self._buffer.find(b"\n")
                if index < 0:
                    break
                line = bytes(self._buffer[: index + 1])
                del self._buffer[: index + 1]
                try:
                    envelope = json.loads(line)
                except json.JSONDecodeError:
                    continue
                self.trace.write(json.dumps(envelope, separators=(",", ":")) + "\n")
                with self._condition:
                    self.envelopes.append(envelope)
                    self._condition.notify_all()

    def latest(self, message_type: str | None = None) -> dict[str, Any] | None:
        with self._condition:
            for envelope in reversed(self.envelopes):
                if message_type is None or envelope.get("message_type") == message_type:
                    return envelope
        return None

    def wait_for(
        self,
        message_type: str,
        *,
        episode_id: str | None = None,
        timeout_s: float = 5.0,
        predicate=None,
    ) -> dict[str, Any]:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while time.monotonic() < deadline:
                for envelope in reversed(self.envelopes):
                    if envelope.get("message_type") != message_type:
                        continue
                    if episode_id is not None and envelope.get("episode_id") != episode_id:
                        continue
                    if predicate is not None and not predicate(envelope):
                        continue
                    return envelope
                self._condition.wait(timeout=0.05)
        raise TimeoutError(f"IPC message {message_type!r} did not arrive")

    def send(
        self,
        *,
        clock: dict[str, Any],
        message_type: str = "cmd_vel",
        vx_m_s: float = 0.0,
        vy_m_s: float = 0.0,
        wz_rad_s: float = 0.0,
        session_id: str = "g3a_acceptance_nav",
        lease_ttl_s: float = 0.25,
        reason: str = "",
    ) -> NavigationControlMessage:
        message = NavigationControlMessage(
            message_type=message_type,
            run_id=clock["run_id"],
            episode_id=clock["episode_id"],
            backend_generation=int(clock["backend_generation"]),
            navigation_epoch=int(clock["navigation_epoch"]),
            session_id=session_id,
            lease_ttl_s=lease_ttl_s,
            vx_m_s=vx_m_s,
            vy_m_s=vy_m_s,
            wz_rad_s=wz_rad_s,
            reason=reason,
        )
        self.socket.sendall(
            (
                json.dumps(message.model_dump(mode="json"), separators=(",", ":"))
                + "\n"
            ).encode("utf-8")
        )
        return message

    def close(self, clock: dict[str, Any] | None = None) -> None:
        if clock is not None and not self._closed.is_set():
            try:
                self.send(clock=clock, message_type="release", reason="acceptance_end")
                time.sleep(0.1)
            except Exception:
                pass
        self._closed.set()
        try:
            self.socket.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.socket.close()
        self.reader.join(timeout=2.0)
        self.trace.close()


def get_json(client: httpx.Client, path: str) -> dict[str, Any]:
    response = client.get(path, timeout=10.0)
    response.raise_for_status()
    return response.json()


def navigation(client: httpx.Client) -> dict[str, Any]:
    return get_json(client, "/api/v1/navigation")


def snapshot(client: httpx.Client) -> dict[str, Any]:
    return get_json(client, "/api/v1/snapshot")


def start_case(client: httpx.Client, name: str) -> dict[str, Any]:
    response = client.post(
        "/api/v1/episodes/start",
        json={
            "task_language": f"G3A finite navigation foundation case: {name}",
            "operator_id": "g3a_server_acceptance",
            "operator_kind": "scripted",
            "view_source": "g3a_server_acceptance",
            "map_mode": "unavailable",
            "require_recording": False,
        },
        timeout=30.0,
    )
    response.raise_for_status()
    result = response.json()
    if not result.get("accepted"):
        raise RuntimeError(f"Episode start rejected: {result}")
    return result


def stop_case(client: httpx.Client, reason: str) -> dict[str, Any]:
    response = client.post(
        "/api/v1/episodes/stop",
        json={"reason": reason},
        timeout=30.0,
    )
    response.raise_for_status()
    return response.json()


def wait_for_observation(
    client: httpx.Client,
    nav_ipc: NavSocketClient,
    *,
    episode_id: str,
    require_scan: bool = False,
    timeout_s: float = 8.0,
) -> tuple[dict[str, Any], dict[str, Any]]:
    predicate = None
    if require_scan:
        predicate = lambda item: item.get("payload", {}).get("scan") is not None
    envelope = nav_ipc.wait_for(
        "observation",
        episode_id=episode_id,
        timeout_s=timeout_s,
        predicate=predicate,
    )
    clock = envelope["payload"]["clock"]
    estimate_envelope = nav_ipc.wait_for(
        "estimate",
        episode_id=episode_id,
        timeout_s=timeout_s,
    )
    return clock, estimate_envelope["payload"]


def wait_latest_scan(client: httpx.Client, timeout_s: float = 5.0) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        response = client.get("/api/v1/navigation/scan", timeout=10.0)
        if response.status_code == 200:
            return response.json()
        time.sleep(0.05)
    raise TimeoutError("latest navigation scan did not reach telemetry")


def clock_from_evidence(evidence: dict[str, Any]) -> dict[str, Any]:
    value = evidence.get("status", {}).get("latest_clock")
    if not isinstance(value, dict):
        raise RuntimeError("navigation evidence has no capture clock")
    return value


def estimate_from_evidence(evidence: dict[str, Any]) -> dict[str, Any]:
    value = evidence.get("estimate")
    if not isinstance(value, dict):
        raise RuntimeError("navigation evidence has no estimate")
    return value


def wait_health(
    client: httpx.Client,
    episode_id: str,
    statuses: set[str],
    timeout_s: float = 5.0,
) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    latest: dict[str, Any] = {}
    while time.monotonic() < deadline:
        latest = navigation(client)
        health = latest.get("telemetry", {}).get("latest_health") or {}
        clock = latest.get("telemetry", {}).get("latest_clock") or {}
        if (
            health.get("status") in statuses
            and (not episode_id or clock.get("episode_id") == episode_id)
        ):
            return latest
        time.sleep(0.05)
    raise TimeoutError(
        f"health did not reach {sorted(statuses)}; latest={latest.get('telemetry')}"
    )


def wait_stopped(client: httpx.Client, timeout_s: float = 5.0) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    latest: dict[str, Any] = {}
    while time.monotonic() < deadline:
        latest = snapshot(client)
        if latest.get("base", {}).get("stopped"):
            return latest
        time.sleep(0.05)
    return latest


def _scene_world_boxes(scene: dict[str, Any]) -> list[tuple[str, np.ndarray, np.ndarray, np.ndarray]]:
    links = scene.get("world_links") or []
    by_name = {item["name"]: item for item in links}
    transforms: dict[str, tuple[np.ndarray, np.ndarray]] = {}

    def resolve(name: str, stack: tuple[str, ...] = ()) -> tuple[np.ndarray, np.ndarray]:
        if name in transforms:
            return transforms[name]
        if name in stack:
            raise ValueError(f"scene link cycle at {name}")
        link = by_name[name]
        local = link.get("local_transform") or {}
        position = np.asarray(local.get("position_m", [0, 0, 0]), dtype=float)
        rotation = quaternion_matrix_xyzw(
            local.get("orientation_xyzw", [0, 0, 0, 1])
        )
        parent = link.get("parent_name")
        if parent and parent in by_name:
            parent_position, parent_rotation = resolve(parent, (*stack, name))
            position = parent_position + parent_rotation @ position
            rotation = parent_rotation @ rotation
        transforms[name] = (position, rotation)
        return transforms[name]

    boxes = []
    for link in links:
        position, rotation = resolve(link["name"])
        for geom in link.get("geoms", []):
            # SceneManifest does not yet expose contype as a collision boolean.
            # world_links are already isolated from robot bodies.
            if geom.get("geom_type") != "box":
                continue
            local = geom.get("local_transform") or {}
            local_position = np.asarray(
                local.get("position_m", [0, 0, 0]), dtype=float
            )
            local_rotation = quaternion_matrix_xyzw(
                local.get("orientation_xyzw", [0, 0, 0, 1])
            )
            box_position = position + rotation @ local_position
            box_rotation = rotation @ local_rotation
            half_size = np.asarray(geom.get("size_m", []), dtype=float)
            if half_size.shape != (3,) or np.any(half_size <= 0):
                continue
            boxes.append(
                (
                    str(geom.get("name", "unknown")),
                    box_position,
                    box_rotation,
                    half_size,
                )
            )
    return boxes


def _ray_obb(
    origin: np.ndarray,
    direction: np.ndarray,
    box: tuple[str, np.ndarray, np.ndarray, np.ndarray],
) -> float | None:
    _, center, rotation, half = box
    local_origin = rotation.T @ (origin - center)
    local_direction = rotation.T @ direction
    near = -math.inf
    far = math.inf
    for axis in range(3):
        start = float(local_origin[axis])
        delta = float(local_direction[axis])
        lower = -float(half[axis])
        upper = float(half[axis])
        if abs(delta) < 1e-12:
            if start < lower or start > upper:
                return None
            continue
        first = (lower - start) / delta
        second = (upper - start) / delta
        if first > second:
            first, second = second, first
        near = max(near, first)
        far = min(far, second)
        if near > far:
            return None
    if far < 0:
        return None
    return near if near >= 0 else far


def independent_scan_geometry(
    scan: dict[str, Any],
    reference: dict[str, Any],
    scene: dict[str, Any],
    profile: dict[str, Any],
) -> dict[str, Any]:
    position = np.asarray(reference["base_position_world_sim_m"], dtype=float)
    rotation = quaternion_matrix_xyzw(reference["base_quaternion_world_xyzw"])
    mount = profile["frames"]["t_base_lidar"]
    origin = position + rotation @ np.asarray(mount["translation_m"], dtype=float)
    mount_rotation = quaternion_matrix_xyzw(mount["rotation_xyzw"])
    boxes = _scene_world_boxes(scene)
    matched = 0
    differences: list[float] = []
    false_hits = 0
    for index, status in enumerate(scan["statuses"]):
        if status != "HIT":
            continue
        angle = float(scan["angle_min_rad"]) + index * float(
            scan["angle_increment_rad"]
        )
        local_direction = np.array(
            [math.cos(angle), math.sin(angle), 0.0], dtype=float
        )
        direction = rotation @ mount_rotation @ local_direction
        distances = [
            value
            for value in (_ray_obb(origin, direction, box) for box in boxes)
            if value is not None
            and float(scan["range_min_m"]) <= value <= float(scan["range_max_m"])
        ]
        if not distances:
            continue
        expected = min(float(value) for value in distances)
        observed = scan["ranges_m"][index]
        if observed is None:
            continue
        difference = abs(float(observed) - expected)
        if difference <= 0.03:
            matched += 1
        else:
            false_hits += 1
        differences.append(difference)
    return {
        "world_box_count": len(boxes),
        "matched_hit_beams": matched,
        "mismatched_hit_beams": false_hits,
        "compared_hit_beams": len(differences),
        "max_abs_range_error_m": max(differences, default=None),
        "mean_abs_range_error_m": (
            float(np.mean(differences)) if differences else None
        ),
        "tolerance_m": 0.03,
    }


def run_stationary(client: httpx.Client, nav_ipc: NavSocketClient) -> dict[str, Any]:
    started = utc_now()
    started_monotonic = time.monotonic()
    result = start_case(client, "stationary_30s")
    episode_id = result["episode_id"]
    clock, initial_estimate = wait_for_observation(
        client, nav_ipc, episode_id=episode_id, require_scan=True
    )
    initial_snapshot = snapshot(client)
    initial_truth = {
        "position": initial_snapshot["base"]["position_world_sim_m"],
        "yaw": yaw_from_quaternion(
            initial_snapshot["base"]["orientation_world_sim_xyzw"]
        ),
    }
    start_telemetry = navigation(client)["telemetry"]
    samples: list[dict[str, Any]] = []
    deadline = time.monotonic() + 30.5
    while time.monotonic() < deadline:
        current = navigation(client)
        telemetry = current["telemetry"]
        estimate = telemetry.get("latest_estimate")
        if isinstance(estimate, dict):
            samples.append(
                {
                    "backend_monotonic_s": estimate["clock"][
                        "capture_backend_monotonic_s"
                    ],
                    "x_m": estimate["x_m"],
                    "y_m": estimate["y_m"],
                    "yaw_rad": estimate["yaw_rad"],
                }
            )
        time.sleep(0.15)
    final_snapshot = snapshot(client)
    end_telemetry = navigation(client)["telemetry"]
    final_estimate = end_telemetry.get("latest_estimate")
    if not isinstance(final_estimate, dict):
        raise RuntimeError("stationary case has no final estimate")
    elapsed = max(1e-9, time.monotonic() - started_monotonic)
    truth_delta = np.asarray(
        final_snapshot["base"]["position_world_sim_m"], dtype=float
    ) - np.asarray(initial_truth["position"], dtype=float)
    estimate_delta = np.array(
        [
            float(final_estimate["x_m"]) - float(initial_estimate["x_m"]),
            float(final_estimate["y_m"]) - float(initial_estimate["y_m"]),
        ]
    )
    drift = float(np.linalg.norm(estimate_delta - truth_delta[:2]))
    final_truth_yaw = yaw_from_quaternion(
        final_snapshot["base"]["orientation_world_sim_xyzw"]
    )
    yaw_error = abs(
        angle_delta(
            angle_delta(initial_estimate["yaw_rad"], float(final_estimate["yaw_rad"])),
            angle_delta(initial_truth["yaw"], final_truth_yaw),
        )
    )
    counts = {
        key: int(end_telemetry.get(key, 0)) - int(start_telemetry.get(key, 0))
        for key in ("observation_count", "scan_count", "estimate_count")
    }
    details = {
        "episode_id": episode_id,
        "run_id": final_snapshot["run_id"],
        "duration_s": elapsed,
        "navigation_epoch": clock["navigation_epoch"],
        "rates_hz": {
            "wheel_and_imu": counts["observation_count"] / elapsed,
            "odom": counts["estimate_count"] / elapsed,
            "scan": counts["scan_count"] / elapsed,
        },
        "counts": counts,
        "max_health_ages_s": {
            "wheel": (end_telemetry.get("latest_health") or {}).get("wheel_age_s"),
            "imu": (end_telemetry.get("latest_health") or {}).get("imu_age_s"),
            "scan": (end_telemetry.get("latest_health") or {}).get("scan_age_s"),
        },
        "final_health": end_telemetry.get("latest_health"),
        "truth_aligned_planar_drift_m": drift,
        "truth_aligned_yaw_error_deg": math.degrees(yaw_error),
        "samples_recorded": len(samples),
        "initial_estimate": initial_estimate,
        "final_estimate": final_estimate,
    }
    details["stop_response"] = stop_case(
        client, "G3A_STATIONARY_COMPLETE"
    )
    gates = {
        "wheel_hz": details["rates_hz"]["wheel_and_imu"] >= 45.0,
        "imu_hz": details["rates_hz"]["wheel_and_imu"] >= 45.0,
        "odom_hz": details["rates_hz"]["odom"] >= 45.0,
        "scan_hz": details["rates_hz"]["scan"] >= 9.0,
        "planar_drift": drift <= 0.01,
        "yaw_drift": math.degrees(yaw_error) <= 2.0,
        "health": (end_telemetry.get("latest_health") or {}).get("status")
        in {"HEALTHY", "DEGRADED"},
    }
    details["gates"] = gates
    _ = started
    return details


def _drive_nav(
    nav_ipc: NavSocketClient,
    client: httpx.Client,
    *,
    duration_s: float,
    vx_m_s: float,
    vy_m_s: float,
    wz_rad_s: float,
    session_id: str | None = None,
) -> str:
    session_id = session_id or f"g3a_nav_{uuid4().hex}"
    evidence = get_json(client, "/api/v1/navigation/evidence")
    clock = clock_from_evidence(evidence)
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        nav_ipc.send(
            clock=clock,
            vx_m_s=vx_m_s,
            vy_m_s=vy_m_s,
            wz_rad_s=wz_rad_s,
            session_id=session_id,
        )
        time.sleep(0.08)
    nav_ipc.send(
        clock=clock,
        message_type="release",
        session_id=session_id,
        reason="segment_complete",
    )
    return session_id


def run_motion_and_takeover(
    client: httpx.Client,
    nav_ipc: NavSocketClient,
) -> dict[str, Any]:
    started = utc_now()
    started_monotonic = time.monotonic()
    result = start_case(client, "motion_independent_arm_manual_takeover")
    episode_id = result["episode_id"]
    _, initial_estimate = wait_for_observation(
        client, nav_ipc, episode_id=episode_id, require_scan=True
    )
    initial = snapshot(client)
    human = HttpCommands(client, operator_id="g3a_manual_takeover")
    arm_initial = list(initial["arm"]["user_joint_positions_rad"])
    arm_target = list(arm_initial)
    arm_target[0] += 0.12
    arm_target[2] -= 0.08
    arm_response: dict[str, Any] = {}

    def command_arm() -> None:
        nonlocal arm_response
        time.sleep(0.45)
        arm_response = human.submit(
            "set_arm_joint_goal",
            {
                "joint_names": [f"joint{index}" for index in range(1, 6)],
                "joint_positions_rad": arm_target,
            },
        )

    arm_thread = threading.Thread(target=command_arm, daemon=True)
    arm_thread.start()
    active_nav_session = _drive_nav(
        nav_ipc,
        client,
        duration_s=0.9,
        vx_m_s=0.03,
        vy_m_s=0.0,
        wz_rad_s=0.0,
    )
    arm_thread.join(timeout=2.0)
    after_forward = snapshot(client)
    _drive_nav(
        nav_ipc,
        client,
        duration_s=0.9,
        vx_m_s=0.0,
        vy_m_s=-0.025,
        wz_rad_s=0.0,
    )
    _drive_nav(
        nav_ipc,
        client,
        duration_s=1.2,
        vx_m_s=0.018,
        vy_m_s=0.014,
        wz_rad_s=0.10,
    )
    before_takeover_evidence = get_json(client, "/api/v1/navigation/evidence")
    before_takeover_estimate = estimate_from_evidence(before_takeover_evidence)
    before_takeover = snapshot(client)
    takeover = human.submit(
        "set_base_twist",
        {
            "frame_id": "base_link",
            "vx_m_s": 0.0,
            "vy_m_s": 0.0,
            "wz_rad_s": 0.0,
        },
    )
    time.sleep(0.25)
    evidence = get_json(client, "/api/v1/navigation/evidence")
    clock = clock_from_evidence(evidence)
    for _ in range(15):
        nav_ipc.send(
            clock=clock,
            vx_m_s=0.03,
            vy_m_s=0.0,
            wz_rad_s=0.0,
            session_id=active_nav_session,
        )
        time.sleep(0.08)
    after_old_command = wait_stopped(client, timeout_s=2.0)
    release = human.submit(
        "zero_base",
        {"reason": "manual_takeover_acceptance_complete"},
    )
    final = snapshot(client)
    elapsed = time.monotonic() - started_monotonic
    displacement = np.asarray(final["base"]["position_world_sim_m"], dtype=float) - np.asarray(
        initial["base"]["position_world_sim_m"], dtype=float
    )
    truth_yaw_initial = yaw_from_quaternion(
        initial["base"]["orientation_world_sim_xyzw"]
    )
    truth_yaw_endpoint = yaw_from_quaternion(
        before_takeover["base"]["orientation_world_sim_xyzw"]
    )
    truth_yaw_delta = angle_delta(truth_yaw_initial, truth_yaw_endpoint)
    estimate_delta = np.array(
        [
            float(before_takeover_estimate["x_m"]) - float(initial_estimate["x_m"]),
            float(before_takeover_estimate["y_m"]) - float(initial_estimate["y_m"]),
        ]
    )
    endpoint_planar_error_m = float(
        np.linalg.norm(estimate_delta - displacement[:2])
    )
    estimate_yaw_delta = angle_delta(
        float(initial_estimate["yaw_rad"]),
        float(before_takeover_estimate["yaw_rad"]),
    )
    endpoint_yaw_error_deg = math.degrees(
        abs(angle_delta(estimate_yaw_delta, truth_yaw_delta))
    )
    arm_delta = np.asarray(
        final["arm"]["user_joint_positions_rad"], dtype=float
    ) - np.asarray(arm_initial, dtype=float)
    details = {
        "episode_id": episode_id,
        "run_id": final["run_id"],
        "duration_s": elapsed,
        "initial_world_position_m": initial["base"]["position_world_sim_m"],
        "before_takeover_world_position_m": before_takeover["base"][
            "position_world_sim_m"
        ],
        "final_world_position_m": final["base"]["position_world_sim_m"],
        "physical_displacement_world_sim_m": displacement.tolist(),
        "truth_planar_displacement_m": displacement[:2].tolist(),
        "estimate_planar_displacement_m": estimate_delta.tolist(),
        "endpoint_planar_error_m": endpoint_planar_error_m,
        "endpoint_yaw_error_deg": endpoint_yaw_error_deg,
        "initial_alignment": "once_before_segment",
        "arm_target_delta_rad": [
            float(target - start)
            for target, start in zip(arm_target, arm_initial)
        ],
        "arm_observed_delta_rad": arm_delta.tolist(),
        "arm_command_status": arm_response.get("status"),
        "manual_takeover_status": takeover.get("status"),
        "release_status": release.get("status"),
        "base_stopped_after_old_nav_cmd": bool(
            after_old_command.get("base", {}).get("stopped")
        ),
        "base_request_after_old_nav_cmd": {
            key: after_old_command.get("base", {}).get(key)
            for key in ("requested_vx_m_s", "requested_vy_m_s", "requested_wz_rad_s")
        },
        "final_health": navigation(client)["telemetry"].get("latest_health"),
    }
    details["stop_response"] = stop_case(client, "G3A_MOTION_COMPLETE")
    moved = float(np.linalg.norm(displacement[:2]))
    arm_moved = bool(np.max(np.abs(arm_delta)) >= 0.01)
    gates = {
        "base_moved": moved >= 0.01,
        "arm_moved_independently": arm_moved,
        "arm_dispatch": arm_response.get("status") in {"DISPATCHED", "APPLIED"},
        "manual_takeover": takeover.get("status") in {"DISPATCHED", "APPLIED"},
        "old_nav_not_restored": bool(
            after_old_command.get("base", {}).get("stopped")
        ),
        "endpoint_planar_within_0_05m": endpoint_planar_error_m <= 0.05,
        "endpoint_yaw_within_5deg": endpoint_yaw_error_deg <= 5.0,
    }
    details["gates"] = gates
    _ = started
    return details


def run_rotation_scan(
    client: httpx.Client,
    nav_ipc: NavSocketClient,
    profile: dict[str, Any],
) -> dict[str, Any]:
    started = utc_now()
    started_monotonic = time.monotonic()
    result = start_case(client, "rotation_scan_geometry")
    episode_id = result["episode_id"]
    wait_for_observation(client, nav_ipc, episode_id=episode_id, require_scan=True)
    _drive_nav(
        nav_ipc,
        client,
        duration_s=0.7,
        vx_m_s=0.025,
        vy_m_s=0.0,
        wz_rad_s=0.0,
    )
    _drive_nav(
        nav_ipc,
        client,
        duration_s=1.35,
        vx_m_s=0.0,
        vy_m_s=0.0,
        wz_rad_s=0.25,
    )
    _drive_nav(
        nav_ipc,
        client,
        duration_s=0.55,
        vx_m_s=0.0,
        vy_m_s=0.0,
        wz_rad_s=-0.12,
    )
    scan = get_json(client, "/api/v1/navigation/scan")
    tick = scan["clock"]["control_tick_id"]
    reference = get_json(
        client,
        f"/api/v1/navigation/reference/{tick}",
    )
    scene = get_json(client, "/api/v1/scene")
    geometry = independent_scan_geometry(scan, reference, scene, profile)
    final = snapshot(client)
    yaw = yaw_from_quaternion(final["base"]["orientation_world_sim_xyzw"])
    details = {
        "episode_id": episode_id,
        "run_id": final["run_id"],
        "duration_s": time.monotonic() - started_monotonic,
        "final_world_yaw_rad": yaw,
        "scan_clock": scan["clock"],
        "scan_status_counts": scan["status_counts"],
        "scan_valid_count": scan["valid_count"],
        "geometry": geometry,
        "reference_truth_label": reference.get("truth_label"),
    }
    gates = {
        "nonzero_or_changed_yaw": abs(yaw) > math.radians(4.0),
        "no_duplicate_endpoint": abs(
            (
                float(scan["angle_min_rad"])
                + (int(scan["beam_count"]) - 1) * float(scan["angle_increment_rad"])
            )
            - float(scan["angle_max_rad"])
        )
        <= 1e-9,
        "geometry_matches": geometry["world_box_count"] > 0
        and geometry["matched_hit_beams"] >= 8
        and geometry["mismatched_hit_beams"] == 0,
        "self_occlusion_explicit": int(scan["status_counts"].get("SELF_OCCLUDED", 0))
        > 0,
        "truth_isolated": reference.get("truth_label") == "GT_DEBUG_ONLY",
    }
    details["stop_response"] = stop_case(client, "G3A_ROTATION_SCAN_COMPLETE")
    details["gates"] = gates
    _ = started
    return details


def run_arm_occlusion(
    client: httpx.Client,
    nav_ipc: NavSocketClient,
) -> dict[str, Any]:
    started = utc_now()
    started_monotonic = time.monotonic()
    result = start_case(client, "finite_arm_activity_occlusion")
    episode_id = result["episode_id"]
    wait_for_observation(client, nav_ipc, episode_id=episode_id, require_scan=True)
    human = HttpCommands(client, operator_id="g3a_arm_activity")
    current = snapshot(client)
    joints = list(current["arm"]["user_joint_positions_rad"])
    target = list(joints)
    target[0] += 0.22
    target[1] += 0.12
    response = human.submit(
        "set_arm_joint_goal",
        {
            "joint_names": [f"joint{index}" for index in range(1, 6)],
            "joint_positions_rad": target,
        },
    )
    baseline_scan = wait_latest_scan(client)
    _drive_nav(
        nav_ipc,
        client,
        duration_s=2.0,
        vx_m_s=0.0,
        vy_m_s=0.0,
        wz_rad_s=0.16,
    )
    time.sleep(0.4)
    final = snapshot(client)
    scan = get_json(client, "/api/v1/navigation/scan")
    arm_delta = np.asarray(final["arm"]["user_joint_positions_rad"], dtype=float) - np.asarray(
        joints, dtype=float
    )
    details = {
        "episode_id": episode_id,
        "run_id": final["run_id"],
        "duration_s": time.monotonic() - started_monotonic,
        "arm_command_status": response.get("status"),
        "arm_observed_delta_rad": arm_delta.tolist(),
        "baseline_self_occluded": int(
            baseline_scan["status_counts"].get("SELF_OCCLUDED", 0)
        ),
        "active_self_occluded": int(scan["status_counts"].get("SELF_OCCLUDED", 0)),
        "active_nonhit_with_fabricated_range": sum(
            1
            for value, status in zip(scan["ranges_m"], scan["statuses"])
            if status != "HIT" and value is not None
        ),
        "final_health": navigation(client)["telemetry"].get("latest_health"),
    }
    details["stop_response"] = stop_case(client, "G3A_ARM_OCCLUSION_COMPLETE")
    gates = {
        "arm_dispatch": response.get("status") in {"DISPATCHED", "APPLIED"},
        "arm_motion": bool(np.max(np.abs(arm_delta)) >= 0.01),
        "self_occlusion_present": details["active_self_occluded"] > 0,
        "nonhit_not_clear_ray": details["active_nonhit_with_fabricated_range"] == 0,
        "scan_present_during_arm": scan["beam_count"] == 360,
    }
    details["gates"] = gates
    _ = started
    return details


def set_fault(client: httpx.Client, fault: str, active: bool) -> dict[str, Any]:
    response = client.post(
        "/api/v1/navigation/fault",
        json={"fault": fault, "active": active},
        timeout=10.0,
    )
    response.raise_for_status()
    return response.json()


def run_failures(
    client: httpx.Client,
    nav_ipc: NavSocketClient,
) -> dict[str, Any]:
    started = utc_now()
    started_monotonic = time.monotonic()
    result = start_case(client, "stale_invalid_manual_safety")
    episode_id = result["episode_id"]
    clock, _ = wait_for_observation(
        client, nav_ipc, episode_id=episode_id, require_scan=True
    )
    fault_records: list[dict[str, Any]] = []
    for fault in ("scan", "imu", "wheel"):
        for _ in range(4):
            nav_ipc.send(
                clock=clock,
                vx_m_s=0.03,
                vy_m_s=0.0,
                wz_rad_s=0.0,
                session_id=f"fault_{fault}",
            )
            time.sleep(0.08)
        active = set_fault(client, fault, True)
        active_status = wait_health(
            client,
            episode_id,
            {"STALE", "FAILED", "DEGRADED", "STOPPED"},
            timeout_s=2.0,
        )
        active_health = active_status["telemetry"].get("latest_health")
        active_snapshot = wait_stopped(client, timeout_s=2.0)
        set_fault(client, fault, False)
        time.sleep(0.3)
        recovered_health = navigation(client)["telemetry"].get("latest_health")
        recovered_snapshot = snapshot(client)
        fault_records.append(
            {
                "fault": fault,
                "activate": active,
                "active_health": active_health,
                "active_stopped": active_snapshot.get("base", {}).get("stopped"),
                "active_request": {
                    key: active_snapshot.get("base", {}).get(key)
                    for key in (
                        "requested_vx_m_s",
                        "requested_vy_m_s",
                        "requested_wz_rad_s",
                    )
                },
                "recovered_health": recovered_health,
                "recovered_stopped_without_new_command": recovered_snapshot.get(
                    "base", {}
                ).get("stopped"),
            }
        )
    human = HttpCommands(client, operator_id="g3a_failure_manual_takeover")
    human_takeover = human.submit(
        "set_base_twist",
        {
            "frame_id": "base_link",
            "vx_m_s": 0.0,
            "vy_m_s": 0.0,
            "wz_rad_s": 0.0,
        },
    )
    stale_clock = dict(clock)
    stale_clock["episode_id"] = "episode_stale_previous"
    nav_ipc.send(
        clock=stale_clock,
        vx_m_s=0.03,
        vy_m_s=0.0,
        wz_rad_s=0.0,
        session_id="stale_episode",
    )
    time.sleep(0.5)
    after_stale = wait_stopped(client, timeout_s=1.0)
    details = {
        "episode_id": episode_id,
        "duration_s": time.monotonic() - started_monotonic,
        "faults": fault_records,
        "manual_takeover_status": human_takeover.get("status"),
        "stale_episode_stopped": after_stale.get("base", {}).get("stopped"),
    }
    details["stop_response"] = stop_case(client, "G3A_FAILURE_CASE_COMPLETE")
    fault_gate = all(
        item["active_health"]
        and item["active_health"].get("status")
        in {"STALE", "FAILED", "DEGRADED", "STOPPED"}
        and item["active_health"].get("stop_required")
        for item in fault_records
    )
    recovery_gate = all(
        item["recovered_stopped_without_new_command"] for item in fault_records
    )
    gates = {
        "faults_invalidate_health_and_stop": fault_gate,
        "no_auto_resume_after_recovery": recovery_gate,
        "manual_takeover_success": human_takeover.get("status")
        in {"DISPATCHED", "APPLIED"},
        "stale_episode_rejected": bool(after_stale.get("base", {}).get("stopped")),
    }
    details["gates"] = gates
    _ = started
    return details


def run_reset(
    client: httpx.Client,
    nav_ipc: NavSocketClient,
) -> dict[str, Any]:
    started = utc_now()
    started_monotonic = time.monotonic()
    first = start_case(client, "reset_epoch_boundary")
    old_clock, old_estimate = wait_for_observation(
        client, nav_ipc, episode_id=first["episode_id"], require_scan=True
    )
    stopped = stop_case(client, "RESET_BOUNDARY_SETUP")
    if not stopped.get("accepted"):
        raise RuntimeError(f"could not stop setup Episode: {stopped}")
    second = start_case(client, "post_reset_new_epoch")
    new_clock, new_estimate = wait_for_observation(
        client, nav_ipc, episode_id=second["episode_id"], require_scan=True
    )
    stale_clock = dict(old_clock)
    nav_ipc.send(
        clock=stale_clock,
        vx_m_s=0.03,
        vy_m_s=0.0,
        wz_rad_s=0.0,
        session_id="pre_reset_navigation",
    )
    time.sleep(0.7)
    final = wait_stopped(client, timeout_s=2.0)
    final_evidence = get_json(client, "/api/v1/navigation/evidence")
    final_estimate = estimate_from_evidence(final_evidence)
    details = {
        "episode_id": second["episode_id"],
        "old_episode_id": first["episode_id"],
        "run_id": final["run_id"],
        "duration_s": time.monotonic() - started_monotonic,
        "old_navigation_epoch": old_clock["navigation_epoch"],
        "new_navigation_epoch": new_clock["navigation_epoch"],
        "old_estimate": old_estimate,
        "new_estimate": new_estimate,
        "final_estimate": final_estimate,
        "stale_old_command_stopped": final.get("base", {}).get("stopped"),
        "final_clock": final_evidence["status"]["latest_clock"],
    }
    details["stop_response"] = stop_case(client, "G3A_RESET_CASE_COMPLETE")
    gates = {
        "new_episode": first["episode_id"] != second["episode_id"],
        "epoch_incremented": int(new_clock["navigation_epoch"])
        > int(old_clock["navigation_epoch"]),
        "origin_reset": abs(float(new_estimate["x_m"])) <= 0.01
        and abs(float(new_estimate["y_m"])) <= 0.01,
        "old_command_rejected": bool(final.get("base", {}).get("stopped")),
        "clock_follows_new_episode": final_evidence["status"]["latest_clock"][
            "episode_id"
        ]
        == second["episode_id"],
    }
    details["gates"] = gates
    return details


def run_case(
    name: str,
    function,
    *arguments,
) -> CaseResult:
    started = utc_now()
    started_monotonic = time.monotonic()
    try:
        details = function(*arguments)
        gates = details.get("gates") or {}
        passed = bool(gates) and all(bool(value) for value in gates.values())
        return CaseResult(
            name=name,
            passed=passed,
            duration_s=time.monotonic() - started_monotonic,
            started_wall_utc=started,
            details=details,
        )
    except Exception:
        for argument in arguments:
            if isinstance(argument, httpx.Client):
                try:
                    stop_case(argument, f"G3A_{name.upper()}_FAILED")
                except Exception:
                    pass
                break
        return CaseResult(
            name=name,
            passed=False,
            duration_s=time.monotonic() - started_monotonic,
            started_wall_utc=started,
            details={},
            error=traceback.format_exc(),
        )


def write_junit(path: Path, results: list[CaseResult]) -> None:
    failures = len([item for item in results if not item.passed])
    rows = [
        '<?xml version="1.0" encoding="utf-8"?>',
        f'<testsuite name="g3a-server-acceptance" tests="{len(results)}" failures="{failures}">',
    ]
    for item in results:
        rows.append(
            f'  <testcase classname="g3a.server" name={quoteattr(item.name)} '
            f'time="{item.duration_s:.3f}">'
        )
        if not item.passed:
            message = (item.error or "one or more gates failed")[:1000]
            rows.append(f'    <failure message={quoteattr(message)}/>')
        rows.append("  </testcase>")
    rows.append("</testsuite>")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(rows) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("configs/g3a_server.yaml"))
    parser.add_argument("--profile", type=Path, default=Path("configs/profiles/g3a_navigation_v1.yaml"))
    parser.add_argument("--report-root", type=Path, required=True)
    parser.add_argument("--job-id", required=True)
    args = parser.parse_args()
    config = args.config.resolve()
    profile_path = args.profile.resolve()
    report_root = args.report_root.resolve()
    report_root.mkdir(parents=True, exist_ok=True)
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    server = AppProcess(config, report_root)
    nav_ipc: NavSocketClient | None = None
    results: list[CaseResult] = []
    startup_error: str | None = None
    health: dict[str, Any] = {}
    try:
        health = server.wait_ready()
        with httpx.Client(
            base_url=server.url,
            timeout=httpx.Timeout(60.0, connect=5.0),
        ) as client:
            socket_value = navigation(client).get("socket")
            if not socket_value:
                raise RuntimeError("navigation socket is not configured")
            nav_ipc = NavSocketClient(Path(socket_value), report_root)
            results.append(
                run_case("stationary_30s", run_stationary, client, nav_ipc)
            )
            results.append(
                run_case(
                    "motion_independent_arm_manual_takeover",
                    run_motion_and_takeover,
                    client,
                    nav_ipc,
                )
            )
            results.append(
                run_case(
                    "rotation_scan_geometry",
                    run_rotation_scan,
                    client,
                    nav_ipc,
                    profile,
                )
            )
            results.append(
                run_case("arm_activity_occlusion", run_arm_occlusion, client, nav_ipc)
            )
            results.append(
                run_case("stale_invalid_manual_safety", run_failures, client, nav_ipc)
            )
            results.append(run_case("reset_epoch", run_reset, client, nav_ipc))
            final_navigation = navigation(client)
    except Exception:
        startup_error = traceback.format_exc()
        final_navigation = {}
    finally:
        if nav_ipc is not None:
            try:
                nav_ipc.close()
            except Exception:
                pass
        server.close()

    passed = bool(results) and all(item.passed for item in results) and not startup_error
    report = {
        "report_version": "g3a-server-acceptance-1.0",
        "job_id": args.job_id,
        "generated_wall_utc": utc_now(),
        "profile_path": str(profile_path),
        "profile_sha256": sha256(profile_path),
        "config_path": str(config),
        "config_sha256": sha256(config),
        "server_environment": {
            "python": sys.version,
            "executable": sys.executable,
            "mujoco_gl": os.environ.get("MUJOCO_GL"),
            "ros_domain_id": 73,
        },
        "startup_health": health,
        "startup_error": startup_error,
        "navigation_final_status": final_navigation,
        "results": [item.as_dict() for item in results],
        "gates": {item.name: item.passed for item in results},
        "passed": passed,
        "execution_location": "server_only",
        "truth_policy": "GT_DEBUG_ONLY; initial trajectory alignment once for scoring only",
    }
    atomic_json(report_root / "g3a_acceptance_report.json", report)
    write_junit(report_root / "g3a_acceptance_junit.xml", results)
    print(json.dumps({"passed": passed, "report": str(report_root / "g3a_acceptance_report.json")}, indent=2))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
