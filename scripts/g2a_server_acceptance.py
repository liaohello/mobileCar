#!/usr/bin/env python3
"""Server-only G2A profile, browser and failure-recovery acceptance."""

from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.metadata
import json
import os
import platform
import signal
import socket
import subprocess
import sys
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable
from uuid import uuid4

import httpx
import yaml
from playwright.sync_api import sync_playwright


ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = ROOT / "src"
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))
WEBGL_QUERY = """
() => {
  const canvas = document.createElement("canvas");
  const gl = canvas.getContext("webgl2") || canvas.getContext("webgl");
  if (!gl) return {available: false};
  const info = gl.getExtension("WEBGL_debug_renderer_info");
  return {
    available: true,
    vendor: info ? gl.getParameter(info.UNMASKED_VENDOR_WEBGL) : gl.getParameter(gl.VENDOR),
    renderer: info ? gl.getParameter(info.UNMASKED_RENDERER_WEBGL) : gl.getParameter(gl.RENDERER),
    version: gl.getParameter(gl.VERSION),
  };
}
"""


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def free_port() -> int:
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def atomic_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{uuid4().hex}.tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


class ServerProcess:
    def __init__(self, config: Path, log_path: Path) -> None:
        self.port = free_port()
        self.log_path = log_path
        self.log_handle = log_path.open("w", encoding="utf-8")
        environment = os.environ.copy()
        environment.update(
            {
                "MUJOCO_GL": "egl",
                "PYOPENGL_PLATFORM": "egl",
                "PYTHONPATH": str(ROOT / "src"),
            }
        )
        command = [
            sys.executable,
            "-m",
            "mobilecar",
            "--config",
            str(config),
            "--host",
            "127.0.0.1",
            "--port",
            str(self.port),
            "--require-recording",
        ]
        self.process = subprocess.Popen(
            command,
            cwd=ROOT,
            env=environment,
            stdout=self.log_handle,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            start_new_session=True,
            text=True,
        )

    @property
    def url(self) -> str:
        return f"http://127.0.0.1:{self.port}"

    def wait_ready(self, timeout_s: float = 30.0) -> None:
        deadline = time.monotonic() + timeout_s
        last_error: Exception | None = None
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                break
            try:
                response = httpx.get(f"{self.url}/api/v1/health", timeout=1.0)
                if response.status_code == 200:
                    return
            except Exception as exc:
                last_error = exc
            time.sleep(0.1)
        tail = self.log_path.read_text(encoding="utf-8", errors="replace")[-6000:]
        raise RuntimeError(
            f"server did not become ready: exit={self.process.poll()} error={last_error}\n{tail}"
        )

    def close(self) -> None:
        if self.process.poll() is None:
            try:
                os.killpg(self.process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                self.process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(self.process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    self.process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(self.process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    self.process.wait(timeout=5)
        self.log_handle.close()


class CommandSequence:
    def __init__(
        self,
        client: httpx.Client,
        *,
        operator_id: str,
        operator_kind: str = "agent_ui",
    ) -> None:
        self.client = client
        self.operator_id = operator_id
        self.operator_kind = operator_kind
        self.session_id = f"session_{uuid4().hex}"
        self.sequence = 0

    def submit(
        self,
        snapshot: dict[str, Any],
        command_type: str,
        payload: dict[str, Any],
        *,
        ttl_s: float = 0.5,
    ) -> dict[str, Any]:
        self.sequence += 1
        request = {
            "contract_version": "mobilecar-1.0",
            "run_id": snapshot["run_id"],
            "backend_generation": snapshot["backend_generation"],
            "episode_id": snapshot["episode_id"],
            "session_id": self.session_id,
            "operator_id": self.operator_id,
            "operator_kind": self.operator_kind,
            "view_source": "third_person_3d",
            "command_id": f"g2a_command_{uuid4().hex}",
            "command_seq": self.sequence,
            "command_type": command_type,
            "requested_ttl_s": ttl_s,
            "payload": payload,
        }
        response = self.client.post("/api/v1/commands", json=request)
        response.raise_for_status()
        return response.json()


def snapshot(client: httpx.Client) -> dict[str, Any]:
    response = client.get("/api/v1/snapshot")
    response.raise_for_status()
    return response.json()


def start_episode(client: httpx.Client, task: str) -> dict[str, Any]:
    response = client.post(
        "/api/v1/episodes/start",
        json={
            "task_language": task,
            "operator_id": "g2a_server_acceptance",
            "operator_kind": "agent_ui",
            "view_source": "third_person_3d",
            "map_mode": "unavailable",
            "require_recording": True,
        },
    )
    response.raise_for_status()
    result = response.json()
    if not result.get("accepted"):
        raise RuntimeError(f"Episode start rejected: {result}")
    return result


def stop_episode(client: httpx.Client, reason: str) -> dict[str, Any]:
    response = client.post("/api/v1/episodes/stop", json={"reason": reason})
    response.raise_for_status()
    return response.json()


def wait_recording(client: httpx.Client, status: str, timeout_s: float = 20.0) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    latest: dict[str, Any] = {}
    while time.monotonic() < deadline:
        latest = snapshot(client)
        current = (latest.get("recording") or {}).get("status")
        if current == status:
            return latest
        time.sleep(0.05)
    raise TimeoutError(f"recording did not reach {status}; latest={latest.get('recording')}")


def quality_report(client: httpx.Client, episode_id: str) -> dict[str, Any]:
    response = client.get(f"/api/v1/episodes/{episode_id}/g2a-quality")
    response.raise_for_status()
    return response.json()


def record_profile_phase(
    process: ServerProcess,
    *,
    phase_name: str,
    moving: bool,
) -> dict[str, Any]:
    with httpx.Client(base_url=process.url, timeout=httpx.Timeout(90.0, connect=5.0)) as client:
        started = start_episode(
            client,
            "G2A server RGB-D profile validation with declared 60 second Episode.",
        )
        episode_id = started["episode_id"]
        wait_recording(client, "RECORDING")
        initial = snapshot(client)
        commands = CommandSequence(client, operator_id=f"g2a_{phase_name}")
        phase_started = time.monotonic()
        action_records: list[dict[str, Any]] = []
        if moving:
            time.sleep(1.5)
            current = snapshot(client)
            base_command = commands.submit(
                current,
                "set_base_twist",
                {
                    "frame_id": "base_link",
                    "vx_m_s": 0.018,
                    "vy_m_s": 0.0,
                    "wz_rad_s": 0.04,
                },
            )
            action_records.append({"type": "set_base_twist", "response": base_command})
            motion_deadline = time.monotonic() + 3.0
            while time.monotonic() < motion_deadline:
                current = snapshot(client)
                commands.submit(
                    current,
                    "set_base_twist",
                    {
                        "frame_id": "base_link",
                        "vx_m_s": 0.018,
                        "vy_m_s": 0.0,
                        "wz_rad_s": 0.04,
                    },
                )
                client.post(
                    "/api/v1/lease/renew",
                    json={"session_id": commands.session_id},
                )
                time.sleep(0.15)
            current = snapshot(client)
            action_records.append(
                {
                    "type": "zero_base",
                    "response": commands.submit(
                        current,
                        "zero_base",
                        {"reason": "profile_phase_complete"},
                    ),
                }
            )
            time.sleep(0.5)
            current = snapshot(client)
            joints = list(current["arm"]["user_joint_positions_rad"])
            joints[0] = float(joints[0]) + 0.035
            joints[4] = float(joints[4]) + 0.035
            action_records.append(
                {
                    "type": "set_gripper_width_goal",
                    "response": commands.submit(
                        current,
                        "set_gripper_width_goal",
                        {"width_m": 0.033},
                    ),
                }
            )
            time.sleep(0.2)
            current = snapshot(client)
            action_records.append(
                {
                    "type": "set_arm_joint_goal",
                    "response": commands.submit(
                        current,
                        "set_arm_joint_goal",
                        {
                            "joint_names": [f"joint{index}" for index in range(1, 6)],
                            "joint_positions_rad": joints,
                        },
                    ),
                }
            )
            arm_deadline = time.monotonic() + 5.0
            while time.monotonic() < arm_deadline:
                client.post(
                    "/api/v1/lease/renew",
                    json={"session_id": commands.session_id},
                )
                time.sleep(0.15)
        while time.monotonic() - phase_started < 60.5:
            time.sleep(0.25)
        final = snapshot(client)
        stopped = stop_episode(client, f"G2A_{phase_name.upper()}_COMPLETE")
        report = quality_report(client, episode_id)
        return {
            "phase": phase_name,
            "episode_id": episode_id,
            "run_id": initial["run_id"],
            "started_wall_utc": utc_now(),
            "wall_duration_s": time.monotonic() - phase_started,
            "sim_duration_s": float(final["sim_time_s"]) - float(initial["sim_time_s"]),
            "initial_snapshot": {
                "backend_monotonic_s": initial["backend_monotonic_s"],
                "sim_time_s": initial["sim_time_s"],
                "control_tick_id": initial["control_tick_id"],
            },
            "final_snapshot": {
                "backend_monotonic_s": final["backend_monotonic_s"],
                "sim_time_s": final["sim_time_s"],
                "control_tick_id": final["control_tick_id"],
                "base_stopped": final["base"]["stopped"],
            },
            "commands": action_records,
            "recording_stop": stopped,
            "quality": report,
        }


def browser_phase(process: ServerProcess, report_root: Path) -> dict[str, Any]:
    screenshot = report_root / "g2a_browser_full_chromium.png"
    commands: list[dict[str, Any]] = []
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(
            channel="chromium",
            headless=True,
            args=[
                "--use-angle=vulkan",
                "--enable-features=Vulkan",
                "--disable-vulkan-surface",
                "--ignore-gpu-blocklist",
                "--enable-webgl",
            ],
        )
        context = browser.new_context(viewport={"width": 1440, "height": 900})

        def on_request(request) -> None:
            if request.method == "POST" and "/api/v1/commands" in request.url:
                try:
                    body = request.post_data_json
                except Exception:
                    body = None
                if isinstance(body, dict):
                    commands.append(
                        {
                            "command_id": body.get("command_id"),
                            "command_type": body.get("command_type"),
                            "payload": body.get("payload"),
                        }
                    )

        def on_response(response) -> None:
            if response.request.method != "POST" or "/api/v1/commands" not in response.url:
                return
            try:
                request_body = response.request.post_data_json
            except Exception:
                return
            command_id = request_body.get("command_id") if isinstance(request_body, dict) else None
            for item in reversed(commands):
                if item.get("command_id") == command_id:
                    item["http_status"] = response.status
                    break

        page = context.new_page()
        page.on("request", on_request)
        page.on("response", on_response)
        page.goto(process.url, wait_until="domcontentloaded")
        page.wait_for_function(
            "document.querySelector('#connection-text').textContent === '已连接'",
            timeout=15_000,
        )
        webgl = page.evaluate(WEBGL_QUERY)
        if not webgl.get("available") or "SwiftShader" in str(webgl.get("renderer")):
            raise AssertionError(f"full Chromium is not on the hardware WebGL renderer: {webgl}")
        page.check("#require-recording")
        page.select_option("#operator-kind", "agent_ui")
        page.fill("#operator-id", "g2a_browser_acceptance")
        page.click("[data-testid='start-recording']")
        page.wait_for_function(
            "document.querySelector('#recording-state').textContent === 'RECORDING'",
            timeout=15_000,
        )
        page.wait_for_function(
            """() => window.mobileCarApp.snapshot?.recording?.status === 'RECORDING'
              && window.mobileCarApp.episodeId === window.mobileCarApp.snapshot.episode_id""",
            timeout=15_000,
        )
        browser_episode_id = page.evaluate("window.mobileCarApp.episodeId")
        phase_started = time.monotonic()

        def command_count(command_type: str) -> int:
            return sum(item["command_type"] == command_type for item in commands)

        def wait_command(command_type: str, minimum: int, timeout_s: float = 15.0) -> None:
            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                if command_count(command_type) >= minimum:
                    return
                page.wait_for_timeout(50)
            raise TimeoutError(f"browser did not issue {command_type} #{minimum}")

        before = command_count("set_base_twist")
        page.keyboard.down("w")
        wait_command("set_base_twist", before + 1)
        page.keyboard.up("w")
        wait_command("zero_base", 1)

        before = command_count("set_gripper_width_goal")
        page.locator("[data-testid='gripper-slider']").evaluate(
            """slider => {
              const step = Number(slider.step);
              slider.value = String(Number(slider.min) + 3 * step);
              slider.dispatchEvent(new Event("input", {bubbles: true}));
            }"""
        )
        page.click("#execute-gripper")
        wait_command("set_gripper_width_goal", before + 1)

        current_tcp = page.evaluate(
            "window.mobileCarApp.snapshot.tcp.position_m.slice()"
        )
        for element_id, value in zip(
            ("#tcp-x", "#tcp-y", "#tcp-z"), current_tcp
        ):
            page.fill(element_id, f"{float(value):.6f}")
        page.select_option("#tcp-frame", "world_sim")
        before_preview = command_count("preview_tcp_target")
        preview_started = time.monotonic()
        page.click("[data-testid='preview-tcp']")
        page.keyboard.down("a")
        wait_command("set_base_twist", command_count("set_base_twist") + 1, 5.0)
        preview_input_latency_s = time.monotonic() - preview_started
        page.keyboard.up("a")
        wait_command("zero_base", 2)
        wait_command("preview_tcp_target", before_preview + 1, 20.0)

        page.wait_for_function(
            "window.mobileCarApp.snapshot?.base?.stopped === true",
            timeout=8_000,
        )
        current_tcp = page.evaluate(
            "window.mobileCarApp.snapshot.tcp.position_m.slice()"
        )
        for element_id, value in zip(
            ("#tcp-x", "#tcp-y", "#tcp-z"), current_tcp
        ):
            page.fill(element_id, f"{float(value):.6f}")
        page.click("[data-testid='preview-tcp']")
        page.wait_for_function(
            "!document.querySelector('#execute-tcp').disabled",
            timeout=20_000,
        )
        before_execute = command_count("execute_tcp_target")
        page.click("[data-testid='execute-tcp']")
        wait_command("execute_tcp_target", before_execute + 1, 10.0)

        page.keyboard.down("d")
        page.wait_for_timeout(300)
        blur_zero_before = command_count("zero_base")
        blur_started = time.monotonic()
        page.evaluate("window.dispatchEvent(new Event('blur'))")
        wait_command("zero_base", blur_zero_before + 1, 5.0)
        blur_zero_latency_s = time.monotonic() - blur_started
        page.keyboard.up("d")
        keys_after_blur = page.evaluate("window.mobileCarApp.keys.size")

        before_offline = command_count("set_base_twist")
        page.keyboard.down("w")
        wait_command("set_base_twist", before_offline + 1, 5.0)
        context.set_offline(True)
        page.wait_for_function(
            "document.querySelector('#connection-text').textContent === '已断开'",
            timeout=10_000,
        )
        keys_after_offline = page.evaluate("window.mobileCarApp.keys.size")
        context.set_offline(False)
        page.wait_for_function(
            "document.querySelector('#connection-text').textContent === '已连接'",
            timeout=15_000,
        )
        page.wait_for_timeout(500)
        reconnect_twist_count = command_count("set_base_twist")
        page.wait_for_timeout(1500)
        if command_count("set_base_twist") != reconnect_twist_count:
            raise AssertionError("reconnect restored an old drive input")
        page.keyboard.up("w")

        while time.monotonic() - phase_started < 30.5:
            page.keyboard.down("s")
            wait_command("set_base_twist", command_count("set_base_twist") + 1, 5.0)
            page.keyboard.up("s")
            page.wait_for_timeout(250)

        page.screenshot(path=str(screenshot), full_page=True)
        with page.expect_response(
            lambda response: response.request.method == "POST"
            and response.url.endswith("/api/v1/episodes/stop"),
            timeout=60_000,
        ) as stop_response_info:
            page.click("[data-testid='stop-recording']")
        if not stop_response_info.value.ok:
            raise AssertionError(
                f"Episode stop/finalization failed: HTTP {stop_response_info.value.status}"
            )
        page.wait_for_function(
            "document.querySelector('#recording-state').textContent === 'COMPLETE'",
            timeout=30_000,
        )
        browser_version = browser.version
        renderer = webgl
        context.close()
        browser.close()

    episode_id = browser_episode_id
    detail = {}
    g2a_quality = None
    if episode_id:
        detail_response = httpx.get(
            f"{process.url}/api/v1/episodes/{episode_id}", timeout=20.0
        )
        if detail_response.status_code == 200:
            detail = detail_response.json()
        quality_response = httpx.get(
            f"{process.url}/api/v1/episodes/{episode_id}/g2a-quality",
            timeout=20.0,
        )
        if quality_response.status_code == 200:
            g2a_quality = quality_response.json()
    return {
        "phase": "browser_30s",
        "duration_s": 30.5,
        "browser_version": browser_version,
        "channel": "chromium",
        "webgl": renderer,
        "episode_id": episode_id,
        "commands": commands,
        "command_count": len(commands),
        "keys_after_blur": keys_after_blur,
        "keys_after_offline": keys_after_offline,
        "preview_input_latency_s": preview_input_latency_s,
        "blur_zero_latency_s": blur_zero_latency_s,
        "reconnect_restored_input": False,
        "screenshot": str(screenshot),
        "screenshot_sha256": sha256(screenshot),
        "validation": detail.get("validation"),
        "g2a_quality": g2a_quality,
    }


def write_fault_config(
    path: Path,
    *,
    run_root: Path,
    quality_root: Path,
    fault_after_events: int,
) -> None:
    from mobilecar.config import load_config

    data = copy.deepcopy(load_config(ROOT / "configs/g2a_server.yaml").data)
    data.pop("extends", None)
    data["recording"]["root"] = str(run_root)
    data["recording"]["fault_after_events"] = fault_after_events
    data["quality"]["report_root"] = str(quality_root)
    path.write_text(
        yaml.safe_dump(data, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )


def fault_and_recovery_phase(
    report_root: Path,
    run_root: Path,
    quality_root: Path,
) -> dict[str, Any]:
    fault_config = report_root / "g2a_fault_config.yaml"
    write_fault_config(
        fault_config,
        run_root=run_root,
        quality_root=quality_root,
        fault_after_events=300,
    )
    fault_process = ServerProcess(fault_config, report_root / "fault_server.log")
    recovered_process = None
    result: dict[str, Any] = {"phase": "write_fault_and_recovery"}
    try:
        fault_process.wait_ready()
        with httpx.Client(
            base_url=fault_process.url,
            timeout=httpx.Timeout(30.0, connect=5.0),
        ) as client:
            started = start_episode(client, "G2A injected recorder failure.")
            fault_episode_id = started["episode_id"]
            wait_recording(client, "RECORDING")
            commands = CommandSequence(client, operator_id="g2a_fault_injection")
            response = commands.submit(
                snapshot(client),
                "set_base_twist",
                {
                    "frame_id": "base_link",
                    "vx_m_s": 0.01,
                    "vy_m_s": 0.0,
                    "wz_rad_s": 0.0,
                },
            )
            wait_recording(client, "FAILED", timeout_s=20.0)
            deadline = time.monotonic() + 10.0
            fault_snapshot = snapshot(client)
            while time.monotonic() < deadline:
                fault_snapshot = snapshot(client)
                if fault_snapshot.get("safety_latched") and fault_snapshot["base"]["stopped"]:
                    break
                time.sleep(0.05)
            stopped = stop_episode(client, "RECORDING_FAILED")
            detail_response = client.get(
                f"/api/v1/episodes/{fault_episode_id}"
            )
            detail = detail_response.json() if detail_response.status_code == 200 else {}
            result["fault"] = {
                "episode_id": fault_episode_id,
                "initial_command": response,
                "recording_stop": stopped,
                "safety_latched": fault_snapshot.get("safety_latched"),
                "base_stopped": fault_snapshot["base"]["stopped"],
                "audit_status": (detail.get("validation") or {}).get("audit_status"),
                "passed": bool(
                    fault_snapshot.get("safety_latched")
                    and fault_snapshot["base"]["stopped"]
                    and (detail.get("validation") or {}).get("audit_status")
                    in {"INCOMPLETE", "FAILED"}
                ),
            }
    finally:
        fault_process.close()

    recovered_process = ServerProcess(
        ROOT / "configs/g2a_server.yaml",
        report_root / "recovered_server.log",
    )
    try:
        recovered_process.wait_ready()
        with httpx.Client(
            base_url=recovered_process.url,
            timeout=httpx.Timeout(30.0, connect=5.0),
        ) as client:
            started = start_episode(client, "G2A post-fault recovery.")
            wait_recording(client, "RECORDING")
            commands = CommandSequence(client, operator_id="g2a_recovery")
            motion = commands.submit(
                snapshot(client),
                "set_base_twist",
                {
                    "frame_id": "base_link",
                    "vx_m_s": 0.01,
                    "vy_m_s": 0.0,
                    "wz_rad_s": 0.0,
                },
            )
            time.sleep(0.4)
            commands.submit(
                snapshot(client),
                "zero_base",
                {"reason": "recovery_complete"},
            )
            stopped = stop_episode(client, "G2A_RECOVERY_COMPLETE")
            result["recovery"] = {
                "episode_id": started["episode_id"],
                "motion_status": motion.get("status"),
                "recording_stop": stopped,
                "passed": bool(
                    motion.get("status") == "DISPATCHED"
                    and stopped.get("accepted")
                    and stopped.get("recording_status") == "COMPLETE"
                ),
            }
    finally:
        recovered_process.close()
    result["passed"] = bool(
        result.get("fault", {}).get("passed")
        and result.get("recovery", {}).get("passed")
    )
    return result


def source_identity() -> dict[str, Any]:
    manifest_path = ROOT / "SOURCE_MANIFEST.json"
    manifest = (
        json.loads(manifest_path.read_text(encoding="utf-8"))
        if manifest_path.is_file()
        else {}
    )
    return {
        "snapshot_id": manifest.get("snapshot_id"),
        "aggregate_sha256": manifest.get("aggregate_sha256"),
        "file_count": manifest.get("file_count"),
        "release_root": str(ROOT),
    }


def environment_identity() -> dict[str, Any]:
    packages = {}
    for name in (
        "mobilecar",
        "mujoco",
        "fastapi",
        "pydantic",
        "playwright",
        "numpy",
        "pillow",
    ):
        try:
            packages[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            packages[name] = None
    return {
        "python": sys.version,
        "executable": sys.executable,
        "platform": platform.platform(),
        "node": os.environ.get("G2A_NODE_VERSION"),
        "packages": packages,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--report-root", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/g2a_server.yaml")
    args = parser.parse_args()
    report_root = args.report_root.resolve()
    report_root.mkdir(parents=True, exist_ok=True)
    report_path = report_root / "g2a_server_acceptance.json"
    run_root = (ROOT / "../../var/runs").resolve()
    quality_root = (ROOT / "../../var/reports").resolve()
    report: dict[str, Any] = {
        "schema_version": 1,
        "report_version": "g2a-server-acceptance-1.0",
        "status": "RUNNING",
        "started_wall_utc": utc_now(),
        "job_id": os.environ.get("G2A_JOB_ID"),
        "source": source_identity(),
        "environment": environment_identity(),
        "profile_sha256": sha256(ROOT / "configs/profiles/g2a_rgbd_5hz.yaml"),
        "action_profile_sha256": sha256(
            ROOT / "configs/profiles/g2a_action_teleop_v1.yaml"
        ),
        "model_sha256": sha256(ROOT / "assets/landerpi/model/robot.xml"),
        "phases": {},
        "known_limitations": [
            "No navigation, policy training, export, or physical-robot claims are included.",
            "RGB-D profile success applies only to the declared 640x480, nominal 5 Hz, 60 s profile.",
            "The browser phase demonstrates input reliability, not long-duration profile throughput.",
        ],
    }
    process: ServerProcess | None = None
    try:
        process = ServerProcess(args.config, report_root / "profile_server.log")
        process.wait_ready()
        report["phases"]["stationary_60s"] = record_profile_phase(
            process,
            phase_name="stationary",
            moving=False,
        )
        report["phases"]["motion_arm_60s"] = record_profile_phase(
            process,
            phase_name="motion_arm",
            moving=True,
        )
        process.close()
        process = None

        browser_process = ServerProcess(
            args.config, report_root / "browser_server.log"
        )
        try:
            browser_process.wait_ready()
            report["phases"]["browser_30s"] = browser_phase(
                browser_process, report_root
            )
        finally:
            browser_process.close()

        report["phases"]["fault_recovery"] = fault_and_recovery_phase(
            report_root,
            run_root,
            quality_root,
        )
        profile_passed = all(
            bool(report["phases"][name]["quality"].get("passed"))
            for name in ("stationary_60s", "motion_arm_60s")
        )
        browser = report["phases"]["browser_30s"]
        browser_passed = bool(
            browser.get("webgl", {}).get("available")
            and "NVIDIA" in str(browser.get("webgl", {}).get("renderer"))
            and browser.get("keys_after_blur") == 0
            and browser.get("keys_after_offline") == 0
            and not browser.get("reconnect_restored_input")
            and browser.get("validation", {}).get("audit_status") == "COMPLETE"
            and browser.get("preview_input_latency_s", 99) < 0.75
            and browser.get("blur_zero_latency_s", 99) < 0.75
        )
        report["gates"] = {
            "stationary_profile": bool(
                report["phases"]["stationary_60s"]["quality"].get("passed")
            ),
            "motion_arm_profile": bool(
                report["phases"]["motion_arm_60s"]["quality"].get("passed")
            ),
            "browser_full_chromium_input": browser_passed,
            "write_fault_controlled_stop": bool(
                report["phases"]["fault_recovery"].get("fault", {}).get("passed")
            ),
            "post_fault_recovery": bool(
                report["phases"]["fault_recovery"].get("recovery", {}).get("passed")
            ),
        }
        report["passed"] = bool(profile_passed and browser_passed and report["phases"]["fault_recovery"].get("passed"))
        report["status"] = "PASS" if report["passed"] else "FAIL"
    except Exception as exc:
        report["status"] = "FAIL"
        report["passed"] = False
        report["error"] = f"{type(exc).__name__}: {exc}"
        report["traceback"] = traceback.format_exc()
    finally:
        if process is not None:
            process.close()
        report["finished_wall_utc"] = utc_now()
        atomic_json(report_path, report)
        print(json.dumps({"report": str(report_path), "status": report["status"], "passed": report.get("passed")}, indent=2))
    return 0 if report.get("passed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
