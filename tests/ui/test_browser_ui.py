from __future__ import annotations

import json
import os
import socket
import threading
import time
from pathlib import Path
from typing import Any

import pytest
import uvicorn
from fastapi import FastAPI, HTTPException
from playwright.sync_api import sync_playwright

from mobilecar.ui.routes import create_ui_router


ROOT = Path(__file__).resolve().parents[2]


def _load(name: str) -> dict[str, Any]:
    return json.loads((ROOT / "configs/examples" / name).read_text(encoding="utf-8"))


def _free_port() -> int:
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


class FixtureRuntime:
    def __init__(self) -> None:
        self.commands: list[dict[str, Any]] = []
        self.lock = threading.Lock()
        self.snapshot = _load("snapshot_fixture.json")
        self.capabilities = _load("capabilities_fixture.json")
        self.scene = _load("scene_fixture.json")
        self.recording_status = "IDLE"
        self.preview_delay_s = 0.0

    def submit(self, command: dict[str, Any]) -> dict[str, Any]:
        with self.lock:
            self.commands.append(command)
        if (
            command["command_type"] == "preview_tcp_target"
            and self.preview_delay_s > 0
        ):
            time.sleep(self.preview_delay_s)
        response: dict[str, Any] = {
            "contract_version": "mobilecar-1.0",
            "command_id": command["command_id"],
            "command_seq": command["command_seq"],
            "status": "DISPATCHED",
            "reason": "",
            "backend_monotonic_s": self.snapshot["backend_monotonic_s"] + 0.01,
            "sim_time_s": self.snapshot["sim_time_s"] + 0.01,
            "decision": {
                "approved": True,
                "safety_revision": "browser-fixture",
                "reason": "",
                "clipped": {},
                "effective_payload": command["payload"],
                "allowed_contacts": [],
            },
            "control_tick_id": self.snapshot["control_tick_id"],
        }
        if command["command_type"] == "preview_tcp_target":
            response["plan"] = {
                "plan_id": "fixture_plan_1",
                "command_id": command["command_id"],
                "target_revision": 1,
                "frame_id": command["payload"]["frame_id"],
                "position_m": command["payload"]["position_m"],
                "orientation_xyzw": None,
                "waypoints_model_rad": [[0.0] * 5, [0.05, -0.02, 0.03, -1.52, 0.02]],
                "times_s": [0.0, 0.4],
                "predicted_position_error_m": 0.001,
                "predicted_orientation_error_rad": 0.02,
                "joint_margin_rad": [1.9, 1.8, 1.7, 1.5, 1.4],
                "collision_free": True,
                "collision_details": [],
                "created_backend_monotonic_s": 100.2,
                "expires_backend_monotonic_s": 130.2,
                "status": "PREVIEWED",
            }
        return response


@pytest.fixture(scope="module")
def ui_server():
    runtime = FixtureRuntime()
    app = FastAPI()

    @app.get("/api/v1/health")
    def health():
        return {"status": "ok", "run_id": "fixture_run", "backend_generation": 1}

    @app.get("/api/v1/capabilities")
    def capabilities():
        return runtime.capabilities

    @app.get("/api/v1/scene")
    def scene():
        return runtime.scene

    @app.get("/api/v1/snapshot")
    def snapshot():
        body = dict(runtime.snapshot)
        body["recording"] = {
            **runtime.snapshot["recording"],
            "status": runtime.recording_status,
        }
        return body

    @app.post("/api/v1/commands")
    def commands(command: dict[str, Any]):
        return runtime.submit(command)

    @app.post("/api/v1/episodes/start")
    def start_episode(_: dict[str, Any]):
        runtime.recording_status = "RECORDING"
        return {
            "accepted": True,
            "reason": "",
            "episode_id": "fixture_episode",
            "recording_segment_id": "fixture_segment",
            "recording_status": "RECORDING",
        }

    @app.post("/api/v1/episodes/stop")
    def stop_episode(_: dict[str, Any]):
        runtime.recording_status = "COMPLETE"
        return {
            "accepted": True,
            "reason": "",
            "episode_id": "fixture_episode",
            "recording_segment_id": "fixture_segment",
            "recording_status": "COMPLETE",
        }

    @app.get("/api/v1/episodes")
    def episodes():
        return {
            "episodes": [
                {
                    "episode_id": "fixture_episode",
                    "audit_status": "COMPLETE",
                    "task_result": "unknown",
                    "operator_kind": "synthetic_fixture",
                }
            ]
        }

    @app.get("/api/v1/episodes/{episode_id}")
    def episode(episode_id: str):
        if episode_id != "fixture_episode":
            raise HTTPException(404)
        return {
            "episode": {
                "episode_id": episode_id,
                "contract_version": "mobilecar-1.0",
                "operator_kind": "synthetic_fixture",
            },
            "validation": {"audit_status": "COMPLETE", "warnings": [], "errors": []},
            "assessment": {"task_result": "unknown"},
            "events": [
                {
                    "event_type": "requested",
                    "backend_monotonic_s": 100.1,
                    "command_id": "fixture_command_1",
                    "task_phase": "APPROACH",
                }
            ],
        }

    @app.post("/api/v1/episodes/{episode_id}/validate")
    def validate(episode_id: str):
        return {
            "contract_version": "mobilecar-1.0",
            "episode_id": episode_id,
            "audit_status": "COMPLETE",
            "passed": True,
            "generated_wall_utc": "2026-10-17T12:00:00Z",
        }

    @app.get("/api/v1/episodes/{episode_id}/assessment")
    def assessment(episode_id: str):
        return {
            "contract_version": "mobilecar-1.0",
            "episode_id": episode_id,
            "task_result": "unknown",
            "oracle_source": "fixture",
        }

    @app.get("/api/v1/episodes/{episode_id}/eligibility")
    def eligibility(episode_id: str):
        return {
            "contract_version": "mobilecar-1.0",
            "episode_id": episode_id,
            "audit_status": "COMPLETE",
            "task_result": "unknown",
        }

    app.include_router(create_ui_router())
    port = _free_port()
    config = uvicorn.Config(app, host="127.0.0.1", port=port, log_level="warning")
    server = uvicorn.Server(config)
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()
    deadline = time.monotonic() + 10
    while not server.started and time.monotonic() < deadline:
        time.sleep(0.02)
    if not server.started:
        raise RuntimeError("fixture UI server did not start")
    yield f"http://127.0.0.1:{port}", runtime
    server.should_exit = True
    thread.join(timeout=5)


def _commands(runtime: FixtureRuntime, command_type: str) -> list[dict[str, Any]]:
    with runtime.lock:
        return [item for item in runtime.commands if item["command_type"] == command_type]


def _launch_hardware_chromium(playwright):
    return playwright.chromium.launch(
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


def test_real_browser_teleoperation_and_replay(ui_server):
    url, runtime = ui_server
    with sync_playwright() as playwright:
        browser = _launch_hardware_chromium(playwright)
        page = browser.new_page(viewport={"width": 1440, "height": 900})
        browser_messages: list[str] = []
        page.on("console", lambda message: browser_messages.append(f"console:{message.type}:{message.text}"))
        page.on("pageerror", lambda error: browser_messages.append(f"pageerror:{error}"))
        page.goto(url, wait_until="domcontentloaded")
        try:
            page.wait_for_function(
                "document.querySelector('#connection-text').textContent === '已连接'",
                timeout=5000,
            )
        except Exception as error:
            events = page.locator("#event-list").inner_text()
            raise AssertionError("\n".join(browser_messages + [f"events:\n{events}"])) from error
        assert page.locator("[data-testid='three-container'] canvas").count() == 1
        renderer = page.evaluate(
            """() => {
              const canvas = document.createElement("canvas");
              const gl = canvas.getContext("webgl2") || canvas.getContext("webgl");
              if (!gl) return null;
              const info = gl?.getExtension("WEBGL_debug_renderer_info");
              return info
                ? gl.getParameter(info.UNMASKED_RENDERER_WEBGL)
                : gl.getParameter(gl.RENDERER);
            }"""
        )
        assert renderer and "SwiftShader" not in renderer, renderer
        assert page.evaluate("window.mobileCarApp.sceneView.renderer.getPixelRatio()") <= 1.5
        render_before = page.evaluate("window.mobileCarApp.sceneView.renderCount")
        page.wait_for_timeout(500)
        render_after = page.evaluate("window.mobileCarApp.sceneView.renderCount")
        assert render_after - render_before <= 8
        canvas = page.locator("[data-testid='three-container'] canvas")
        box = canvas.bounding_box()
        assert box

        camera_before = page.evaluate(
            "window.mobileCarApp.sceneView.camera.position.toArray()"
        )
        hit_element = page.evaluate(
            """([x, y]) => {
              const node = document.elementFromPoint(x, y);
              return node ? `${node.tagName}.${node.className}` : "none";
            }""",
            [
                    box["x"] + box["width"] * 0.15,
                    box["y"] + box["height"] * 0.25,
            ],
        )
        assert hit_element.startswith("CANVAS"), hit_element
        page.mouse.move(box["x"] + box["width"] * 0.15, box["y"] + box["height"] * 0.25)
        page.mouse.down()
        page.mouse.move(box["x"] + box["width"] * 0.28, box["y"] + box["height"] * 0.15, steps=6)
        page.mouse.up()
        page.wait_for_timeout(120)
        camera_rotated = page.evaluate(
            "window.mobileCarApp.sceneView.camera.position.toArray()"
        )
        assert camera_rotated != camera_before
        distance_before = page.evaluate(
            "window.mobileCarApp.sceneView.camera.position.distanceTo(window.mobileCarApp.sceneView.controls.target)"
        )
        page.mouse.move(box["x"] + box["width"] * 0.65, box["y"] + box["height"] * 0.55)
        page.mouse.wheel(0, -480)
        page.wait_for_timeout(160)
        distance_after = page.evaluate(
            "window.mobileCarApp.sceneView.camera.position.distanceTo(window.mobileCarApp.sceneView.controls.target)"
        )
        assert distance_after < distance_before

        before = len(_commands(runtime, "set_base_twist"))
        page.keyboard.down("w")
        page.wait_for_function(
            f"window.mobileCarApp && {before} < window.mobileCarApp.commandSeq",
            timeout=4000,
        )
        assert any(
            item["payload"]["vx_m_s"] > 0
            for item in _commands(runtime, "set_base_twist")[before:]
        )
        page.keyboard.up("w")
        page.wait_for_timeout(250)
        assert _commands(runtime, "zero_base")

        zero_before_blur = len(_commands(runtime, "zero_base"))
        page.keyboard.down("a")
        page.wait_for_timeout(220)
        page.evaluate("window.dispatchEvent(new Event('blur'))")
        page.wait_for_timeout(220)
        assert len(_commands(runtime, "zero_base")) > zero_before_blur
        assert page.evaluate("window.mobileCarApp.keys.size") == 0

        twist_before_focus = len(_commands(runtime, "set_base_twist"))
        page.locator("#tcp-x").focus()
        page.keyboard.press("w")
        page.wait_for_timeout(250)
        assert len(_commands(runtime, "set_base_twist")) == twist_before_focus

        page.locator("#tcp-y").fill("0.01")
        page.locator("[data-testid='preview-tcp']").click()
        page.wait_for_function("!document.querySelector('#execute-tcp').disabled")
        assert _commands(runtime, "preview_tcp_target")
        page.locator("[data-testid='execute-tcp']").click()
        page.wait_for_timeout(150)
        assert _commands(runtime, "execute_tcp_target")

        joint_command_before = len(_commands(runtime, "set_arm_joint_goal"))
        page.locator("[data-joint='0']").fill("0.12")
        page.locator("[data-testid='execute-joints']").click()
        page.wait_for_timeout(150)
        joint_commands = _commands(runtime, "set_arm_joint_goal")
        assert len(joint_commands) > joint_command_before
        assert joint_commands[-1]["payload"]["joint_positions_rad"][0] == pytest.approx(0.12)

        gripper_command_before = len(_commands(runtime, "set_gripper_width_goal"))
        page.locator("[data-testid='gripper-slider']").fill("0.02")
        page.locator("#execute-gripper").click()
        page.wait_for_timeout(150)
        gripper_commands = _commands(runtime, "set_gripper_width_goal")
        assert len(gripper_commands) > gripper_command_before
        assert gripper_commands[-1]["payload"]["width_m"] == pytest.approx(0.02)

        target_before = page.evaluate("window.mobileCarApp.target.slice()")
        target_screen = page.evaluate(
            """() => {
              const app = window.mobileCarApp;
              const point = app.sceneView.camera.position.clone();
              app.sceneView.targetGroup.getWorldPosition(point);
              point.project(app.sceneView.camera);
              const rect = app.sceneView.renderer.domElement.getBoundingClientRect();
              return {
                x: rect.left + (point.x + 1) * 0.5 * rect.width,
                y: rect.top + (1 - point.y) * 0.5 * rect.height,
              };
            }"""
        )
        page.mouse.move(target_screen["x"], target_screen["y"])
        page.mouse.down()
        page.mouse.move(target_screen["x"] + 28, target_screen["y"] - 18, steps=6)
        page.mouse.up()
        page.wait_for_timeout(120)
        target_after = page.evaluate("window.mobileCarApp.target.slice()")
        assert target_after != target_before
        assert page.locator("[data-testid='execute-tcp']").is_disabled()
        screenshot = os.environ.get("MOBILECAR_UI_SCREENSHOT")
        if screenshot:
            page.screenshot(path=screenshot, full_page=True)

        page.locator("[data-testid='start-recording']").click()
        page.wait_for_function("document.querySelector('#recording-state').textContent === 'RECORDING'")
        page.locator("[data-testid='stop-recording']").click()
        page.wait_for_function("document.querySelector('#recording-state').textContent === 'COMPLETE'")

        page.locator("[data-testid='replay-open']").click()
        page.wait_for_selector(".episode-item")
        page.locator(".episode-item").first.click()
        page.wait_for_function(
            "document.querySelector('#replay-report').textContent.includes('COMPLETE')"
        )
        assert page.locator(".replay-event").count() >= 1

        page.locator("[data-testid='stop-button']").click()
        page.wait_for_timeout(150)
        assert _commands(runtime, "stop")
        browser.close()


def test_delayed_preview_does_not_block_safety_inputs(ui_server):
    url, runtime = ui_server
    with runtime.lock:
        runtime.preview_delay_s = 1.5
    try:
        with sync_playwright() as playwright:
            browser = _launch_hardware_chromium(playwright)
            page = browser.new_page(viewport={"width": 1280, "height": 800})
            page.goto(url, wait_until="domcontentloaded")
            page.wait_for_function(
                "document.querySelector('#connection-text').textContent === '已连接'",
                timeout=15_000,
            )
            preview_before = len(_commands(runtime, "preview_tcp_target"))
            page.click("[data-testid='preview-tcp']")
            deadline = time.monotonic() + 3
            while (
                len(_commands(runtime, "preview_tcp_target")) <= preview_before
                and time.monotonic() < deadline
            ):
                page.wait_for_timeout(20)
            assert len(_commands(runtime, "preview_tcp_target")) > preview_before

            twist_before = len(_commands(runtime, "set_base_twist"))
            started = time.monotonic()
            page.keyboard.down("w")
            deadline = started + 1.0
            while (
                len(_commands(runtime, "set_base_twist")) <= twist_before
                and time.monotonic() < deadline
            ):
                page.wait_for_timeout(10)
            assert len(_commands(runtime, "set_base_twist")) > twist_before
            assert time.monotonic() - started < 0.75

            zero_before = len(_commands(runtime, "zero_base"))
            started = time.monotonic()
            page.evaluate("window.dispatchEvent(new Event('blur'))")
            deadline = started + 1.0
            while (
                len(_commands(runtime, "zero_base")) <= zero_before
                and time.monotonic() < deadline
            ):
                page.wait_for_timeout(10)
            assert len(_commands(runtime, "zero_base")) > zero_before
            assert time.monotonic() - started < 0.75
            assert page.evaluate("window.mobileCarApp.keys.size") == 0
            browser.close()
    finally:
        with runtime.lock:
            runtime.preview_delay_s = 0.0
