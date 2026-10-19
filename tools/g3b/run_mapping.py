#!/usr/bin/env python3
"""Server-only G3B map generation from actual MuJoCo scan and G3A odometry."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import signal
import socket
import subprocess
import sys
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from uuid import uuid4

import httpx
import yaml

from mobilecar.config import load_config


ROOT = Path(__file__).resolve().parents[2]
PROJECT_ROOT = ROOT.parents[1] if (ROOT.parent.name == "releases") else ROOT
PYTHON = PROJECT_ROOT / "envs/g2a-py310-mujoco316/bin/python"


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


class Process:
    def __init__(self, command: list[str], log: Path, env: dict[str, str]) -> None:
        self.log_path = log
        self.handle = log.open("w", encoding="utf-8")
        self.process = subprocess.Popen(
            command,
            cwd=ROOT,
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=self.handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            text=True,
        )

    def alive(self) -> bool:
        return self.process.poll() is None

    def close(self, timeout_s: float = 15.0) -> int | None:
        if self.alive():
            try:
                os.killpg(self.process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                self.process.wait(timeout=timeout_s)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(self.process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    self.process.wait(timeout=8.0)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(self.process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    self.process.wait(timeout=5.0)
        self.handle.close()
        return self.process.returncode


def command_sequence(
    client: httpx.Client,
    *,
    session: str,
    sequence: int,
    vx: float,
    vy: float,
    wz: float,
) -> dict[str, Any]:
    snapshot = client.get("/api/v1/snapshot").json()
    request = {
        "contract_version": "mobilecar-1.0",
        "run_id": snapshot["run_id"],
        "backend_generation": snapshot["backend_generation"],
        "episode_id": snapshot["episode_id"],
        "session_id": session,
        "operator_id": "g3b_mapping_route",
        "operator_kind": "scripted",
        "view_source": "g3b_mapping_route",
        "command_id": f"map_velocity_{uuid4().hex}",
        "command_seq": sequence,
        "command_type": "set_base_twist",
        "requested_ttl_s": 3.0,
        "payload": {
            "frame_id": "base_link",
            "vx_m_s": vx,
            "vy_m_s": vy,
            "wz_rad_s": wz,
        },
    }
    response = client.post("/api/v1/commands", json=request)
    response.raise_for_status()
    return response.json()


def drive_route(client: httpx.Client) -> dict[str, Any]:
    session = f"g3b_mapping_{uuid4().hex}"
    sequence = 0
    started = time.monotonic()
    deadline = time.monotonic() + 24.0
    while time.monotonic() < deadline:
        sequence += 1
        response = command_sequence(
            client,
            session=session,
            sequence=sequence,
            vx=0.0,
            vy=0.0,
            wz=0.25,
        )
        if response.get("status") not in {"DISPATCHED", "APPLIED"}:
            raise RuntimeError(f"mapping rotation rejected: {response}")
        time.sleep(0.12)
    sequence += 1
    zero = command_sequence(
        client,
        session=session,
        sequence=sequence,
        vx=0.0,
        vy=0.0,
        wz=0.0,
    )
    if zero.get("status") not in {"DISPATCHED", "APPLIED"}:
        raise RuntimeError(f"mapping stop rejected: {zero}")
    time.sleep(1.0)
    return {
        "duration_s": time.monotonic() - started,
        "session_id": session,
        "command_count": sequence,
        "stop_response": zero,
    }


def topic_names(env: dict[str, str]) -> set[str]:
    result = subprocess.run(
        ["ros2", "topic", "list"],
        cwd=ROOT,
        env=env,
        text=True,
        capture_output=True,
        timeout=15.0,
    )
    return {line.strip() for line in result.stdout.splitlines() if line.strip()}


def wait_topics(env: dict[str, str], required: set[str], timeout_s: float) -> set[str]:
    deadline = time.monotonic() + timeout_s
    latest: set[str] = set()
    while time.monotonic() < deadline:
        latest = topic_names(env)
        if required.issubset(latest):
            return latest
        time.sleep(0.5)
    return latest


def scan_config_hash(profile: Path) -> str:
    payload = yaml.safe_load(profile.read_text(encoding="utf-8"))
    encoded = json.dumps(
        payload["sensors"]["scan"], sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--map-id", required=True)
    parser.add_argument("--scene-id", required=True)
    parser.add_argument("--report-root", type=Path, required=True)
    parser.add_argument("--map-root", type=Path, required=True)
    parser.add_argument("--domain-id", type=int, default=74)
    args = parser.parse_args()
    report_root = args.report_root.resolve()
    report_root.mkdir(parents=True, exist_ok=True)
    map_root = args.map_root.resolve()
    map_root.mkdir(parents=True, exist_ok=True)
    config = load_config(args.config.resolve()).data
    backend = config["backend"]
    measurement_profile = ROOT / backend["navigation_profile_path"]
    sensor_profile = ROOT / backend["observation_profile_path"]
    g3b_profile = ROOT / config["g3b"]["profile_path"]
    model = ROOT / backend["model_path"]

    env = os.environ.copy()
    env.update(
        {
            "MUJOCO_GL": "egl",
            "PYOPENGL_PLATFORM": "egl",
            "ROS_DOMAIN_ID": str(args.domain_id),
            "RMW_IMPLEMENTATION": env.get("G3A_ROS_RMW", "rmw_fastrtps_cpp"),
            "PYTHONPATH": str(ROOT / "src")
            + (":" + env["PYTHONPATH"] if env.get("PYTHONPATH") else ""),
        }
    )
    port = free_port()
    app = Process(
        [
            str(PYTHON),
            "-m",
            "mobilecar",
            "--config",
            str(args.config.resolve()),
            "--host",
            "127.0.0.1",
            "--port",
            str(port),
        ],
        report_root / "mobilecar_server.log",
        env,
    )
    supervisor = None
    slam = None
    report: dict[str, Any] = {
        "report_version": "g3b-mapping-1",
        "map_id": args.map_id,
        "scene_id": args.scene_id,
        "generated_wall_utc": utc_now(),
        "execution_location": "server_only",
        "success": False,
    }
    try:
        base_url = f"http://127.0.0.1:{port}"
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            if not app.alive():
                raise RuntimeError("MuJoCo application exited")
            try:
                if httpx.get(f"{base_url}/api/v1/health", timeout=1.0).status_code == 200:
                    break
            except Exception:
                pass
            time.sleep(0.1)
        else:
            raise RuntimeError("MuJoCo application startup timeout")
        with httpx.Client(base_url=base_url, timeout=30.0) as client:
            episode_response = client.post(
                "/api/v1/episodes/start",
                json={
                    "task_language": f"G3B map preparation {args.map_id}",
                    "operator_id": "g3b_mapping_operator",
                    "operator_kind": "scripted",
                    "view_source": "g3b_mapping_route",
                    "map_mode": "building",
                    "require_recording": True,
                },
            )
            episode_response.raise_for_status()
            episode = episode_response.json()
            if not episode.get("accepted"):
                raise RuntimeError(f"mapping Episode start rejected: {episode}")
            report["episode_id"] = episode["episode_id"]
            snapshot = client.get("/api/v1/snapshot").json()
            report["run_id"] = snapshot["run_id"]
            report["navigation_epoch"] = snapshot["diagnostics"]["navigation"][
                "navigation_epoch"
            ]
            supervisor = Process(
                [
                    "ros2",
                    "launch",
                    "mobilecar_goal_supervisor",
                    "g3b_supervisor.launch.py",
                    f"map_id:={args.map_id}",
                    f"map_manifest:={map_root / 'manifest.json'}",
                    f"runtime_config:={args.config.resolve()}",
                    "socket_path:=../../var/ipc/g3b_navigation.sock",
                ],
                report_root / "g3b_supervisor.log",
                env,
            )
            slam = Process(
                [
                    "ros2",
                    "launch",
                    "mobilecar_mapping_localization",
                    "mapping.launch.py",
                    "use_sim_time:=true",
                ],
                report_root / "slam_toolbox_mapping.log",
                env,
            )
            topics = wait_topics(
                env,
                {"/clock", "/scan", "/odom", "/map", "/pose"},
                timeout_s=45.0,
            )
            report["topics"] = sorted(topics)
            if not {"/clock", "/scan", "/odom", "/map", "/pose"}.issubset(topics):
                raise RuntimeError(f"mapping topics unavailable: {sorted(topics)}")
            report["route"] = drive_route(client)
            map_manager_report = report_root / "map_manager.json"
            manager = subprocess.run(
                [
                    str(PYTHON),
                    "-m",
                    "mobilecar_mapping_localization.mapping_manager",
                    "--map-root",
                    str(map_root),
                    "--map-id",
                    args.map_id,
                    "--scene-id",
                    args.scene_id,
                    "--model-sha256",
                    sha256(model),
                    "--profile-id",
                    str(yaml.safe_load(g3b_profile.read_text(encoding="utf-8"))["profile_id"]),
                    "--profile-version",
                    str(yaml.safe_load(g3b_profile.read_text(encoding="utf-8"))["profile_version"]),
                    "--profile-sha256",
                    sha256(g3b_profile),
                    "--navigation-profile-sha256",
                    sha256(measurement_profile),
                    "--sensor-profile-sha256",
                    sha256(sensor_profile),
                    "--scan-config-sha256",
                    scan_config_hash(measurement_profile),
                    "--run-id",
                    snapshot["run_id"],
                    "--episode-id",
                    episode["episode_id"],
                    "--navigation-epoch",
                    str(report["navigation_epoch"]),
                    "--output-manifest",
                    str(map_root / "manifest.json"),
                ],
                cwd=ROOT,
                env=env,
                text=True,
                capture_output=True,
                timeout=60.0,
            )
            map_manager_report.write_text(
                manager.stdout + ("\nSTDERR\n" + manager.stderr if manager.stderr else ""),
                encoding="utf-8",
            )
            if manager.returncode != 0:
                raise RuntimeError(f"map manager failed: {manager.stderr[-2000:]}")
            report["map_manager"] = json.loads(manager.stdout)
            stop_response = client.post(
                "/api/v1/episodes/stop",
                json={"reason": "G3B_MAPPING_COMPLETE"},
            )
            stop_response.raise_for_status()
            report["stop_response"] = stop_response.json()
        manifest_path = map_root / "manifest.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        report.update(
            {
                "manifest_path": str(manifest_path),
                "manifest_sha256": sha256(manifest_path),
                "map_assets": manifest["assets"],
                "success": True,
            }
        )
    except Exception:
        report["error"] = traceback.format_exc()
    finally:
        report["slam_exit_code"] = slam.close() if slam is not None else None
        report["supervisor_exit_code"] = (
            supervisor.close() if supervisor is not None else None
        )
        report["application_exit_code"] = app.close()
        atomic_json(report_root / "g3b_mapping_report.json", report)
    print(
        json.dumps(
            {
                "success": report["success"],
                "report": str(report_root / "g3b_mapping_report.json"),
                "map_id": args.map_id,
            },
            indent=2,
        )
    )
    return 0 if report["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
