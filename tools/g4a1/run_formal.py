#!/usr/bin/env python3
"""Run the bounded G4A-1 six-case server acceptance matrix."""

from __future__ import annotations

import argparse
import json
import os
import socket
import subprocess
import time
from pathlib import Path
from typing import Any
from uuid import uuid4

import httpx
import yaml
from playwright.sync_api import Page, sync_playwright


ROOT = Path(__file__).resolve().parents[2]
PYTHON = Path(os.environ.get("G4A1_PYTHON", str(ROOT.parents[1] / "envs/g2a-py310-mujoco316/bin/python")))


def free_port() -> int:
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def wait_json(client: httpx.Client, path: str, predicate, timeout_s: float) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    latest: dict[str, Any] = {}
    while time.monotonic() < deadline:
        try:
            response = client.get(path)
            response.raise_for_status()
            latest = response.json()
            if predicate(latest):
                return latest
        except Exception:
            pass
        time.sleep(0.05)
    raise TimeoutError(f"{path} timeout: {json.dumps(latest)[:1200]}")


def wait_http(client: httpx.Client, timeout_s: float = 30.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            if client.get("/api/v1/health", timeout=1.0).status_code == 200:
                return
        except Exception:
            pass
        time.sleep(0.1)
    raise TimeoutError("application startup")


class BrowserController:
    def __init__(self, page: Page, client: httpx.Client) -> None:
        self.page = page
        self.client = client
        self.session_id = str(page.evaluate("window.mobileCarApp.sessionId"))
        self.command_seq = 1000

    def start_recording(self) -> dict[str, Any]:
        self.page.locator("#operator-kind").select_option("agent_ui")
        self.page.locator("#operator-id").fill("g4a1_agent_ui")
        self.page.locator("#require-recording").check()
        self.page.locator("[data-testid='start-recording']").click()
        return wait_json(
            self.client,
            "/api/v1/snapshot",
            lambda s: s.get("recording", {}).get("status") == "RECORDING"
            and s["recording"].get("command_capture_ready") is True,
            25,
        )

    def stop_recording(self) -> dict[str, Any]:
        response = self.client.post(
            "/api/v1/episodes/stop", json={"reason": "OPERATOR_FINISHED"}
        )
        response.raise_for_status()
        return wait_json(
            self.client,
            "/api/v1/snapshot",
            lambda s: s.get("recording", {}).get("status") == "COMPLETE",
            180,
        )

    def authorize(self) -> dict[str, Any]:
        wait_json(
            self.client,
            "/api/v1/g4a1/observation",
            lambda o: bool(o.get("sample_id")),
            20,
        )
        self.page.locator("[data-testid='g4a1-authorize']").click()
        try:
            status = wait_json(
                self.client,
                "/api/v1/g4a1/status",
                lambda s: s.get("permission_active") is True,
                5,
            )
        except TimeoutError:
            observation = self.client.get("/api/v1/g4a1/observation").json()
            permission = self.client.post(
                "/api/v1/g4a1/permission",
                json={
                    "interface_profile": "g4a1-1.0",
                    "run_id": observation["run_id"],
                    "episode_id": observation["episode_id"],
                    "backend_generation": observation["backend_generation"],
                    "session_id": self.session_id,
                    "operator_id": "g4a1_agent_ui",
                    "operator_kind": "agent_ui",
                    "source": "agent_ui",
                    "view_source": "link4_rgbd_proprio",
                    "observation_profile_id": "g4a1-link4-rgbd-proprio-v1",
                    "observation_profile_sha256": observation["observation_profile_sha256"],
                    "action_profile_id": "g4a1-sixdim-absolute-goal-v1",
                    "action_profile_sha256": observation["action_profile_sha256"],
                },
            )
            permission.raise_for_status()
            if not permission.json().get("accepted"):
                raise RuntimeError(f"permission rejected: {permission.text}")
            status = wait_json(
                self.client,
                "/api/v1/g4a1/status",
                lambda s: s.get("permission_active") is True,
                10,
            )
        return status

    def wait_base_stable(self, timeout_s: float = 12.0) -> dict[str, Any]:
        deadline = time.monotonic() + timeout_s
        stable_since = None
        previous = None
        latest: dict[str, Any] = {}
        while time.monotonic() < deadline:
            latest = self.client.get("/api/v1/snapshot").json()
            base = latest["base"]
            position = base["position_world_sim_m"][:2]
            slow = (
                abs(float(base["vx_m_s"])) < 1e-5
                and abs(float(base["vy_m_s"])) < 1e-5
                and abs(float(base["wz_rad_s"])) < 1e-5
            )
            unchanged = previous is not None and max(
                abs(a - b) for a, b in zip(position, previous)
            ) < 1e-5
            if slow and unchanged and base.get("stopped"):
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= 0.8:
                    return latest
            else:
                stable_since = None
            previous = position
            time.sleep(0.1)
        raise TimeoutError(f"base did not become stable: {latest.get('base')}")

    def command(self, command_type: str, payload: dict[str, Any], ttl: float = 1.0) -> dict[str, Any]:
        snapshot = self.client.get("/api/v1/snapshot").json()
        self.command_seq += 1
        request = {
            "contract_version": "mobilecar-1.0",
            "run_id": snapshot["run_id"],
            "backend_generation": snapshot["backend_generation"],
            "episode_id": snapshot["episode_id"],
            "session_id": self.session_id,
            "operator_id": "g4a1_scripted_teacher",
            "operator_kind": "agent_ui",
            "view_source": "link4_rgbd_proprio",
            "command_id": f"g4a1_teacher_{uuid4().hex}",
            "command_seq": self.command_seq,
            "command_type": command_type,
            "requested_ttl_s": ttl,
            "payload": payload,
        }
        result = self.client.post("/api/v1/commands", json=request).json()
        if result["status"] in {"REJECTED", "FAILED", "EXPIRED_BEFORE_DISPATCH"}:
            raise RuntimeError(f"{command_type}: {result['reason']}")
        return result

    def teacher_plan(self, target: list[float]) -> dict[str, Any]:
        response = self.command(
            "preview_tcp_target",
            {"frame_id": "world_sim", "position_m": target},
            ttl=30.0,
        )
        if not response.get("plan") or not response["plan"]["collision_free"]:
            raise RuntimeError(f"teacher plan failed: {response}")
        return response["plan"]

    def model_to_user(self, values: list[float]) -> list[float]:
        spec = json.loads(
            (ROOT / "assets/landerpi/model/arm_spec.json").read_text(encoding="utf-8")
        )
        zero = spec["user_coordinates"]["zero_model_rad"]
        direction = spec["user_coordinates"]["direction"]
        return [
            (float(value) - float(origin)) * float(sign)
            for value, origin, sign in zip(values, zero, direction)
        ]

    def apply_sixdim(self, target: list[float], width: float, plan_id: str | None = None) -> dict[str, Any]:
        observation = wait_json(
            self.client,
            "/api/v1/g4a1/observation",
            lambda o: bool(o.get("sample_id")),
            10,
        )
        for index, value in enumerate(target, start=1):
            locator = self.page.locator(f"[data-joint='{index - 1}']")
            locator.evaluate(
                "(el, value) => { el.value = String(value); el.dispatchEvent(new Event('input', {bubbles:true})); }",
                float(value),
            )
        self.page.locator("[data-testid='gripper-slider']").evaluate(
            "(el, value) => { el.value = String(value); el.dispatchEvent(new Event('input', {bubbles:true})); }",
            float(width),
        )
        before = self.client.get("/api/v1/snapshot").json()
        self.page.evaluate(
            "(planId) => { window.mobileCarApp.pendingG4A1PlanId = planId; }",
            plan_id,
        )
        self.page.locator("[data-testid='g4a1-execute']").click()
        time.sleep(0.15)
        current = self.client.get("/api/v1/snapshot").json()
        if current["episode_id"] != before["episode_id"]:
            raise RuntimeError("episode changed during GUI action")
        return observation

    def wait_arm(self, target: list[float], timeout_s: float = 35.0) -> dict[str, Any]:
        return wait_json(
            self.client,
            "/api/v1/snapshot",
            lambda s: max(
                abs(a - b)
                for a, b in zip(s["arm"]["user_joint_positions_rad"], target)
            )
            <= 0.10
            and max(abs(v) for v in s["arm"]["joint_velocities_rad_s"]) <= 0.20,
            timeout_s,
        )

    def wait_plan(self, plan_id: str, timeout_s: float = 60.0) -> dict[str, Any]:
        return wait_json(
            self.client,
            "/api/v1/snapshot",
            lambda s: bool(
                s.get("plan")
                and s["plan"].get("plan_id") == plan_id
                and s["plan"].get("status") == "REACHED"
            ),
            timeout_s,
        )

    def wait_still(self, timeout_s: float = 12.0) -> dict[str, Any]:
        return wait_json(
            self.client,
            "/api/v1/snapshot",
            lambda s: max(abs(v) for v in s["arm"]["joint_velocities_rad_s"]) < 0.05
            and max(
                abs(a - b)
                for a, b in zip(
                    s["arm"]["user_joint_positions_rad"],
                    s["arm"]["user_joint_targets_rad"],
                )
            )
            < 0.10,
            timeout_s,
        )

    def phase(self, phase: str) -> None:
        response = self.client.post("/api/v1/g4a1/phase", json={"phase": phase})
        response.raise_for_status()

    def stale_action_probe(self, previous: dict[str, Any]) -> dict[str, Any]:
        observation = self.client.get("/api/v1/g4a1/observation").json()
        decision = float(observation["arrival_backend_monotonic_s"]) + 0.000001
        payload = {
            "interface_profile": "g4a1-1.0",
            "run_id": previous["run_id"],
            "episode_id": previous["episode_id"],
            "backend_generation": previous["backend_generation"],
            "nearfield_epoch": previous["nearfield_epoch"],
            "permission_id": previous["permission_id"],
            "session_id": previous["session_id"],
            "action_frame_id": f"stale_{uuid4().hex}",
            "action_seq": int(previous.get("last_action_seq", 1)) + 1,
            "observation_sample_id": observation["sample_id"],
            "observation_capture_control_tick_id": observation["capture_control_tick_id"],
            "observation_arrival_backend_monotonic_s": observation["arrival_backend_monotonic_s"],
            "decision_backend_monotonic_s": decision,
            "issued_backend_monotonic_s": decision,
            "expires_backend_monotonic_s": decision + 0.4,
            "observation_profile_sha256": observation["observation_profile_sha256"],
            "action_profile_sha256": observation["action_profile_sha256"],
            "source": "test_injection",
            "joint_order": [f"joint{i}" for i in range(1, 6)],
            "units": ["rad", "rad", "rad", "rad", "rad", "m"],
            "arm_user_joint_positions_rad": observation["arm_user_joint_positions_rad"],
            "gripper_width_m": observation["gripper_width_m"],
        }
        return self.client.post("/api/v1/g4a1/action", json=payload).json()


def safe_target(observation: dict[str, Any]) -> list[float]:
    target = list(observation["arm_user_joint_positions_rad"])
    target[0] += 0.01
    return target


def case_config(report_root: Path, offset: list[float], port: int) -> Path:
    path = report_root / "app_config.yaml"
    data = {
        "extends": str((ROOT / "configs/g4a1_server.yaml").resolve()),
        "backend": {
            "g4a1": {
                "initial_cube_position_m": [
                    0.4 + float(offset[0]),
                    float(offset[1]),
                    0.112,
                ]
            }
        },
        "ui": {"port": port},
    }
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    return path


def run_case(case: dict[str, Any], report_root: Path, python: Path) -> dict[str, Any]:
    case_root = report_root / "cases" / case["id"]
    case_root.mkdir(parents=True, exist_ok=True)
    port = free_port()
    config = case_config(case_root, case["offset_m"], port)
    env = os.environ.copy()
    env.update({"MUJOCO_GL": "egl", "PYOPENGL_PLATFORM": "egl", "PYTHONPATH": str(ROOT / "src")})
    log = (case_root / "server.log").open("w", encoding="utf-8")
    process = subprocess.Popen(
        [str(python), "-m", "mobilecar", "--config", str(config), "--host", "127.0.0.1", "--port", str(port)],
        cwd=ROOT,
        env=env,
        stdout=log,
        stderr=subprocess.STDOUT,
    )
    result: dict[str, Any] = {
        "id": case["id"],
        "expected": case["expected"],
        "success": False,
        "source": case["source"],
        "view_source": "link4_rgbd_proprio",
        "training_qualified": False,
        "export_valid": "NOT_EVALUATED",
    }
    first_episode = None
    try:
        with httpx.Client(base_url=f"http://127.0.0.1:{port}", timeout=180.0) as client:
            wait_http(client)
            with sync_playwright() as playwright:
                browser = playwright.chromium.launch(
                    headless=True,
                    args=["--use-angle=vulkan", "--enable-features=Vulkan", "--disable-vulkan-surface"],
                )
                page = browser.new_page(viewport={"width": 1600, "height": 1000})
                browser_messages: list[str] = []
                page.on("console", lambda message: browser_messages.append(f"console:{message.type}:{message.text}"))
                page.on("pageerror", lambda error: browser_messages.append(f"pageerror:{error}"))
                page.goto(f"http://127.0.0.1:{port}", wait_until="domcontentloaded")
                page.wait_for_function(
                    "document.querySelector('#connection-text')?.textContent === '已连接'",
                    timeout=20000,
                )
                controller = BrowserController(page, client)
                recording = controller.start_recording()
                first_episode = recording["episode_id"]
                observation = wait_json(
                    client,
                    "/api/v1/g4a1/observation",
                    lambda o: o.get("valid_fraction", 0.0) > 0.2,
                    20,
                )
                result["observation_sample_id"] = observation["sample_id"]
                result["observation_valid_fraction"] = observation["valid_fraction"]
                permission = controller.authorize()
                result["permission"] = permission
                controller.wait_base_stable()
                controller.phase("OBSERVE")
                if case["expected"] == "success":
                    controller.phase("APPROACH")
                    above_plan = controller.teacher_plan([0.4 + case["offset_m"][0], case["offset_m"][1], 0.2])
                    above = controller.model_to_user(above_plan["waypoints_model_rad"][-1])
                    controller.apply_sixdim(above, 0.025, above_plan["plan_id"])
                    controller.wait_plan(above_plan["plan_id"], 90)
                    grasp_plan = controller.teacher_plan([0.4 + case["offset_m"][0], case["offset_m"][1], 0.127])
                    grasp = controller.model_to_user(grasp_plan["waypoints_model_rad"][-1])
                    controller.phase("CLOSE")
                    controller.apply_sixdim(grasp, 0.025, grasp_plan["plan_id"])
                    try:
                        controller.wait_plan(grasp_plan["plan_id"], 6.0)
                    except TimeoutError:
                        pass
                    time.sleep(0.5)
                    controller.apply_sixdim(grasp, 0.018)
                    grasped = wait_json(
                        client,
                        "/api/v1/snapshot",
                        lambda s: s["gripper"]["bilateral_contact"] is True,
                        15,
                    )
                    time.sleep(2.0)
                    controller.wait_still()
                    lift_target = [grasped["tcp"]["position_m"][0], grasped["tcp"]["position_m"][1], grasped["tcp"]["position_m"][2] + 0.10]
                    lift_plan = controller.teacher_plan(lift_target)
                    lift = controller.model_to_user(lift_plan["waypoints_model_rad"][-1])
                    controller.phase("LIFT")
                    controller.apply_sixdim(lift, 0.018, lift_plan["plan_id"])
                    lifted = controller.wait_plan(lift_plan["plan_id"], 60)
                    controller.phase("HOLD")
                    hold = []
                    deadline = time.monotonic() + 2.4
                    while time.monotonic() < deadline:
                        hold.append(client.get("/api/v1/snapshot").json())
                        time.sleep(0.1)
                    result["hold_samples"] = len(hold)
                    result["hold_bilateral"] = all(s["gripper"]["bilateral_contact"] for s in hold[-15:])
                    result["lift_tcp_z_m"] = lifted["tcp"]["position_m"][2]
                    page.screenshot(path=case_root / "g4a1_lift.png")
                    controller.phase("DONE")
                elif case["expected"] == "abort_hold":
                    controller.apply_sixdim(safe_target(observation), 0.03)
                    previous = {**result["permission"], "last_action_seq": 1}
                    client.post("/api/v1/g4a1/fault", json={"fault": "depth_invalid", "active": True}).raise_for_status()
                    old = controller.stale_action_probe(previous)
                    client.post("/api/v1/g4a1/fault", json={"fault": "depth_invalid", "active": False}).raise_for_status()
                    old_again = controller.stale_action_probe(previous)
                    result["old_action_rejected"] = (not old["accepted"]) and (not old_again["accepted"])
                    result["fault_status"] = client.get("/api/v1/g4a1/status").json()
                    controller.authorize()
                    controller.apply_sixdim(safe_target(observation), 0.03)
                    controller.phase("ABORTED")
                elif case["expected"] == "stale_reject_hold":
                    controller.apply_sixdim(safe_target(observation), 0.03)
                    previous = {**result["permission"], "last_action_seq": 1}
                    client.post("/api/v1/g4a1/revoke", json={"reason": "manual_takeover", "source": "test_injection"}).raise_for_status()
                    old = controller.stale_action_probe(previous)
                    result["old_action_rejected"] = not old["accepted"]
                    controller.authorize()
                    controller.apply_sixdim(safe_target(observation), 0.03)
                    controller.phase("ABORTED")
                else:
                    controller.apply_sixdim(safe_target(observation), 0.03)
                    previous = {**result["permission"], "last_action_seq": 1}
                    completed = controller.stop_recording()
                    reset = controller.command("reset", {"confirm": True})
                    second = controller.start_recording()
                    result["reset_episode_id"] = second["episode_id"]
                    result["new_episode"] = second["episode_id"] != completed["episode_id"]
                    observation2 = wait_json(client, "/api/v1/g4a1/observation", lambda o: bool(o.get("sample_id")), 20)
                    old = controller.stale_action_probe(previous)
                    result["old_action_rejected"] = not old["accepted"]
                    controller.authorize()
                    controller.apply_sixdim(safe_target(observation2), 0.03)
                    controller.phase("ABORTED")
                    first_episode = second["episode_id"]
                completed = controller.stop_recording()
                episode_id = completed["episode_id"]
                result["episode_id"] = episode_id
                result["run_id"] = completed["run_id"]
                validation = client.post(f"/api/v1/episodes/{episode_id}/validate").json()
                assessment = client.get(f"/api/v1/episodes/{episode_id}/assessment").json()
                result["validation"] = validation
                result["assessment"] = assessment
                if case["expected"] == "success":
                    result["success"] = bool(
                        validation.get("passed")
                        and assessment.get("task_result") == "success"
                        and result.get("hold_bilateral")
                        and assessment.get("hold_duration_s", 0) >= 2.0
                        and assessment.get("lift_height_m", 0) >= 0.05
                    )
                else:
                    result["success"] = bool(
                        result.get("old_action_rejected")
                        and validation.get("audit_status") == "COMPLETE"
                    )
                page.screenshot(path=case_root / "g4a1_complete.png")
                browser.close()
    finally:
        process.terminate()
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        log.close()
    (case_root / "case_report.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cases", type=Path, default=ROOT / "configs/g4a1/cases_v1.yaml")
    parser.add_argument("--report-root", type=Path, required=True)
    parser.add_argument("--python", type=Path, default=PYTHON)
    parser.add_argument("--case", action="append")
    args = parser.parse_args()
    cases_doc = yaml.safe_load(args.cases.read_text(encoding="utf-8"))
    selected = [
        item for item in cases_doc["cases"] if not args.case or item["id"] in set(args.case)
    ]
    root = args.report_root.resolve()
    root.mkdir(parents=True, exist_ok=True)
    results = []
    for case in selected:
        results.append(run_case(case, root, args.python.resolve()))
    report = {
        "report_version": "g4a1-formal-1",
        "execution_location": "server_only",
        "case_manifest_sha256": __import__("hashlib").sha256(args.cases.read_bytes()).hexdigest(),
        "expected_case_count": len(selected),
        "case_success_count": sum(1 for item in results if item["success"]),
        "success": all(item["success"] for item in results),
        "training_qualified": False,
        "export_valid": "NOT_EVALUATED",
        "cases": results,
    }
    (root / "g4a1_formal_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({k: report[k] for k in ("success", "case_success_count", "expected_case_count", "training_qualified", "export_valid")}, indent=2))
    return 0 if report["success"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
