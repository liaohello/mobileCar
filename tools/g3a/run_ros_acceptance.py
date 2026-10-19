#!/usr/bin/env python3
"""Server-only end-to-end MuJoCo IPC and ROS 2 Humble acceptance."""

from __future__ import annotations

import argparse
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


ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def atomic_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{uuid4().hex}.tmp")
    temporary.write_text(
        json.dumps(value, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def free_port() -> int:
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


class ManagedProcess:
    def __init__(
        self,
        command: list[str],
        *,
        log_path: Path,
        environment: dict[str, str],
    ) -> None:
        self.log_path = log_path
        log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log = log_path.open("w", encoding="utf-8")
        self.process = subprocess.Popen(
            command,
            cwd=ROOT,
            env=environment,
            stdin=subprocess.DEVNULL,
            stdout=self.log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            text=True,
        )

    def close(self, timeout_s: float = 15.0) -> int | None:
        if self.process.poll() is None:
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
        self.log.close()
        return self.process.returncode


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("configs/g3a_server.yaml"))
    parser.add_argument("--report-root", type=Path, required=True)
    parser.add_argument("--job-id", required=True)
    parser.add_argument("--domain-id", type=int, default=73)
    args = parser.parse_args()
    report_root = args.report_root.resolve()
    report_root.mkdir(parents=True, exist_ok=True)
    domain_id = int(args.domain_id)
    environment = os.environ.copy()
    environment.update(
        {
            "MUJOCO_GL": "egl",
            "PYOPENGL_PLATFORM": "egl",
            "ROS_DOMAIN_ID": str(domain_id),
            "RMW_IMPLEMENTATION": environment.get(
                "G3A_ROS_RMW", "rmw_fastrtps_cpp"
            ),
            "PYTHONPATH": str(SRC)
            + (":" + environment["PYTHONPATH"] if environment.get("PYTHONPATH") else ""),
        }
    )
    install_setup = ROOT / "ros2_ws/install/setup.bash"
    if not install_setup.is_file():
        raise RuntimeError("ROS workspace is not built in this release")

    port = free_port()
    app = ManagedProcess(
        [
            sys.executable,
            "-m",
            "mobilecar",
            "--config",
            str(args.config.resolve()),
            "--host",
            "127.0.0.1",
            "--port",
            str(port),
        ],
        log_path=report_root / "mobilecar_server.log",
        environment=environment,
    )
    bridge: ManagedProcess | None = None
    report: dict[str, Any] = {
        "report_version": "g3a-ros-acceptance-1.0",
        "job_id": args.job_id,
        "generated_wall_utc": utc_now(),
        "domain_id": domain_id,
        "passed": False,
        "execution_location": "server_only",
    }
    try:
        base_url = f"http://127.0.0.1:{port}"
        deadline = time.monotonic() + 30.0
        startup = None
        while time.monotonic() < deadline:
            if app.process.poll() is not None:
                break
            try:
                response = httpx.get(f"{base_url}/api/v1/health", timeout=1.0)
                if response.status_code == 200:
                    startup = response.json()
                    break
            except Exception:
                pass
            time.sleep(0.1)
        if startup is None:
            raise RuntimeError("MuJoCo application did not become ready")
        report["startup_health"] = startup

        with httpx.Client(base_url=base_url, timeout=30.0) as client:
            episode_response = client.post(
                "/api/v1/episodes/start",
                json={
                    "task_language": "G3A ROS bridge server acceptance",
                    "operator_id": "g3a_ros_acceptance",
                    "operator_kind": "scripted",
                    "view_source": "g3a_ros_acceptance",
                    "map_mode": "unavailable",
                    "require_recording": False,
                },
            )
            episode_response.raise_for_status()
            episode = episode_response.json()
            if not episode.get("accepted"):
                raise RuntimeError(f"Episode start rejected: {episode}")
            report["episode_id"] = episode["episode_id"]

            bridge = ManagedProcess(
                [
                    "ros2",
                    "launch",
                    "mobilecar_navigation_bridge",
                    "navigation_bridge.launch.py",
                ],
                log_path=report_root / "ros_bridge.log",
                environment=environment,
            )
            time.sleep(2.0)
            if bridge.process.poll() is not None:
                raise RuntimeError("ROS bridge exited during startup")
            initial = client.get("/api/v1/snapshot").json()
            probe_path = report_root / "ros_probe.json"
            probe = subprocess.run(
                [
                    sys.executable,
                    "scripts/g3a_ros_probe.py",
                    "--output",
                    str(probe_path),
                    "--duration",
                    "4.0",
                    "--drive",
                    "--vx",
                    "0.025",
                    "--vy",
                    "0.0",
                    "--wz",
                    "0.08",
                ],
                cwd=ROOT,
                env=environment,
                stdin=subprocess.DEVNULL,
                text=True,
                capture_output=True,
                timeout=40.0,
            )
            time.sleep(0.8)
            final = client.get("/api/v1/snapshot").json()
            navigation = client.get("/api/v1/navigation").json()
            stop_response = client.post(
                "/api/v1/episodes/stop",
                json={"reason": "G3A_ROS_ACCEPTANCE_COMPLETE"},
            )
            stop_response.raise_for_status()
            probe_report = (
                json.loads(probe_path.read_text(encoding="utf-8"))
                if probe_path.is_file()
                else None
            )
            displacement = [
                float(final["base"]["position_world_sim_m"][axis])
                - float(initial["base"]["position_world_sim_m"][axis])
                for axis in range(3)
            ]
            physical_motion = (
                displacement[0] ** 2 + displacement[1] ** 2
            ) ** 0.5
            gates = dict((probe_report or {}).get("gates", {}))
            gates["probe_exit_zero"] = probe.returncode == 0
            gates["probe_report_present"] = probe_report is not None
            gates["physical_motion_from_ros_cmd"] = physical_motion >= 0.005
            gates["bridge_alive"] = bridge.process.poll() is None
            report.update(
                {
                    "probe_returncode": probe.returncode,
                    "probe_stdout": probe.stdout,
                    "probe_stderr": probe.stderr,
                    "probe": probe_report,
                    "physical_displacement_world_sim_m": displacement,
                    "physical_planar_motion_m": physical_motion,
                    "navigation": navigation,
                    "stop_response": stop_response.json(),
                    "gates": gates,
                    "passed": bool(gates) and all(bool(value) for value in gates.values()),
                }
            )
    except Exception:
        report["error"] = traceback.format_exc()
    finally:
        if bridge is not None:
            report["bridge_exit_code"] = bridge.close()
        report["application_exit_code"] = app.close()
        atomic_json(report_root / "g3a_ros_acceptance_report.json", report)
    print(
        json.dumps(
            {
                "passed": report.get("passed", False),
                "report": str(report_root / "g3a_ros_acceptance_report.json"),
            },
            indent=2,
        )
    )
    return 0 if report.get("passed", False) else 1


if __name__ == "__main__":
    raise SystemExit(main())
