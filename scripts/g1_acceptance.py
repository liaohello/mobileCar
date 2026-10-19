#!/usr/bin/env python3
"""Deterministic G1 physical acceptance through the application API.

Chromium owns the visible same-origin scene, starts the formal recording and
performs read-only replay. Motion is issued by the acceptance agent through the
same versioned API with operator_kind=agent_ui; the independent UI tests cover
keyboard, slider and drag interaction.
"""

from __future__ import annotations

import argparse
import json
import threading
import time
from pathlib import Path
from typing import Any
from uuid import uuid4

import httpx
from playwright.sync_api import Page, sync_playwright


ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "docs/evidence"


def wait_snapshot(
    client: httpx.Client,
    predicate,
    timeout_s: float,
    label: str,
) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    latest = None
    while time.monotonic() < deadline:
        latest = client.get("/api/v1/snapshot").json()
        if predicate(latest):
            return latest
        time.sleep(0.05)
    raise TimeoutError(f"{label}: {json.dumps(latest, ensure_ascii=False)[:2000]}")


class Acceptance:
    def __init__(self, client: httpx.Client, session_id: str) -> None:
        self.client = client
        self.session_id = session_id
        self.sequence = 0

    def snapshot(self) -> dict[str, Any]:
        return self.client.get("/api/v1/snapshot").json()

    def command(
        self,
        command_type: str,
        payload: dict[str, Any],
        ttl: float = 0.5,
    ) -> dict[str, Any]:
        self.sequence += 1
        snapshot = self.snapshot()
        request = {
            "contract_version": "mobilecar-1.0",
            "run_id": snapshot["run_id"],
            "backend_generation": snapshot["backend_generation"],
            "episode_id": snapshot["episode_id"],
            "session_id": self.session_id,
            "operator_id": "agent_ui_acceptance",
            "operator_kind": "agent_ui",
            "view_source": "third_person_3d",
            "command_id": f"acceptance_{self.sequence}_{uuid4().hex}",
            "command_seq": self.sequence,
            "command_type": command_type,
            "requested_ttl_s": ttl,
            "payload": payload,
        }
        response = self.client.post("/api/v1/commands", json=request)
        response.raise_for_status()
        result = response.json()
        if result["status"] in {"REJECTED", "FAILED", "EXPIRED_BEFORE_DISPATCH"}:
            raise RuntimeError(f"{command_type} rejected: {result['reason']}")
        return result

    def move_tcp(self, position: list[float], timeout_s: float = 45.0) -> dict[str, Any]:
        preview = self.command(
            "preview_tcp_target",
            {"frame_id": "world_sim", "position_m": position},
            ttl=30.0,
        )
        plan = preview.get("plan")
        if not plan:
            raise RuntimeError(f"no plan for {position}: {preview['reason']}")
        if not plan["collision_free"]:
            raise RuntimeError(f"collision for {position}: {plan['collision_details']}")
        execute = self.command(
            "execute_tcp_target",
            {"frame_id": "world_sim", "position_m": position, "plan_id": plan["plan_id"]},
            ttl=1.0,
        )
        if execute["status"] != "DISPATCHED":
            raise RuntimeError(f"execute not dispatched: {execute['reason']}")
        result = wait_snapshot(
            self.client,
            lambda s: bool(
                s.get("plan")
                and s["plan"].get("plan_id") == plan["plan_id"]
                and s["plan"].get("status") == "REACHED"
            ),
            timeout_s,
            f"TCP target {position}",
        )
        if not result["recording"]["command_capture_ready"]:
            raise RuntimeError("recording gate lost during TCP motion")
        return result


def poll_page(page: Page) -> None:
    page.evaluate("window.mobileCarApp.pollSnapshot()")
    page.wait_for_timeout(400)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="http://127.0.0.1:8787")
    args = parser.parse_args()
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    heartbeat_stop = threading.Event()
    heartbeat_thread = None
    recording_started = False
    result: dict[str, Any] = {}

    with httpx.Client(base_url=args.url, timeout=180.0) as client:
        with sync_playwright() as playwright:
            browser = playwright.chromium.launch(
                headless=True,
                args=["--disable-background-timer-throttling", "--disable-renderer-backgrounding"],
            )
            page = browser.new_page(viewport={"width": 1600, "height": 1000})
            try:
                page.goto(args.url, wait_until="domcontentloaded")
                page.wait_for_function(
                    "document.querySelector('#connection-text')?.textContent === '已连接'",
                    timeout=15000,
                )
                page.locator("#operator-kind").select_option("agent_ui")
                page.locator("#operator-id").fill("agent_ui_acceptance")
                page.locator("#require-recording").check()
                page.locator("[data-testid='start-recording']").click()
                recording = wait_snapshot(
                    client,
                    lambda s: s.get("recording", {}).get("status") == "RECORDING"
                    and s["recording"].get("command_capture_ready") is True,
                    20,
                    "recording start",
                )
                recording_started = True
                episode_id = recording["episode_id"]
                run_id = recording["run_id"]
                session_id = page.evaluate("window.mobileCarApp.sessionId")
                page.screenshot(path=EVIDENCE / "g1_initial_recording.png")

                heartbeat_thread = threading.Thread(
                    target=lambda: heartbeat_loop(args.url, session_id, heartbeat_stop),
                    name="g1-acceptance-heartbeat",
                    daemon=True,
                )
                heartbeat_thread.start()
                controller = Acceptance(client, session_id)
                controller.command("set_task_phase", {"phase": "APPROACH"})

                deadline = time.monotonic() + 20
                driven = controller.snapshot()
                while time.monotonic() < deadline:
                    controller.command(
                        "set_base_twist",
                        {
                            "frame_id": "base_link",
                            "vx_m_s": 0.05,
                            "vy_m_s": 0.0,
                            "wz_rad_s": 0.0,
                        },
                    )
                    driven = controller.snapshot()
                    if driven["base"]["position_world_sim_m"][0] >= 0.138:
                        break
                    time.sleep(0.1)
                else:
                    raise TimeoutError("base approach timeout")
                controller.command("zero_base", {"reason": "approach_complete"})
                stopped = wait_snapshot(
                    client,
                    lambda s: s["base"].get("stopped") is True,
                    8,
                    "base stop",
                )
                if not 0.132 <= stopped["base"]["position_world_sim_m"][0] <= 0.155:
                    raise RuntimeError(f"approach position {stopped['base']['position_world_sim_m']}")

                above = controller.move_tcp([0.4, 0.0, 0.2])
                poll_page(page)
                page.screenshot(path=EVIDENCE / "g1_tcp_above_cube.png")
                at_cube = controller.move_tcp([0.4, 0.0, 0.127])
                controller.command("set_task_phase", {"phase": "MANIPULATE"})

                controller.command("set_gripper_width_goal", {"width_m": 0.018})
                grasped = wait_snapshot(
                    client,
                    lambda s: s["gripper"].get("bilateral_contact") is True,
                    12,
                    "bilateral grasp",
                )
                time.sleep(1.8)
                settled = client.get("/api/v1/snapshot").json()
                if not settled["gripper"]["bilateral_contact"]:
                    raise RuntimeError("bilateral contact lost while settling")

                lift = [
                    settled["tcp"]["position_m"][0],
                    settled["tcp"]["position_m"][1],
                    settled["tcp"]["position_m"][2] + 0.1,
                ]
                lifted = controller.move_tcp(lift)
                hold_samples = []
                hold_deadline = time.monotonic() + 3.2
                while time.monotonic() < hold_deadline:
                    current = client.get("/api/v1/snapshot").json()
                    hold_samples.append(
                        {
                            "sim_time_s": current["sim_time_s"],
                            "cube_z_m": current["diagnostics"]["object_pose_world_sim"][
                                "position_m"
                            ][2],
                            "bilateral_contact": current["gripper"]["bilateral_contact"],
                            "held": current["tcp"]["held"],
                        }
                    )
                    time.sleep(0.1)
                if not all(item["bilateral_contact"] and item["held"] for item in hold_samples[-20:]):
                    raise RuntimeError("lift hold was not stable for the final 2 seconds")
                controller.command("set_task_phase", {"phase": "VERIFY"})
                poll_page(page)
                page.screenshot(path=EVIDENCE / "g1_mujoco_lift_hold.png")

                stopped_recording = client.post(
                    "/api/v1/episodes/stop",
                    json={"reason": "OPERATOR_FINISHED"},
                )
                stopped_recording.raise_for_status()
                completed = wait_snapshot(
                    client,
                    lambda s: s.get("recording", {}).get("status") == "COMPLETE",
                    180,
                    "recording completion",
                )
                recording_started = False
                heartbeat_stop.set()
                if heartbeat_thread is not None:
                    heartbeat_thread.join(timeout=2)

                validation = client.post(
                    f"/api/v1/episodes/{episode_id}/validate"
                ).json()
                assessment = client.get(
                    f"/api/v1/episodes/{episode_id}/assessment"
                ).json()
                eligibility = client.get(
                    f"/api/v1/episodes/{episode_id}/eligibility"
                ).json()
                detail = client.get(f"/api/v1/episodes/{episode_id}").json()

                page.locator("[data-testid='replay-open']").click()
                page.wait_for_selector(f"[data-episode-id='{episode_id}']", timeout=15000)
                page.locator(f"[data-episode-id='{episode_id}']").click()
                page.wait_for_function(
                    "document.querySelector('#replay-report')?.textContent?.length > 10",
                    timeout=20000,
                )
                page.screenshot(path=EVIDENCE / "g1_replay_success.png")
                page.locator("#load-assessment").click()
                page.wait_for_function(
                    "document.querySelector('#replay-report')?.textContent?.includes('success')",
                    timeout=20000,
                )
                page.screenshot(path=EVIDENCE / "g1_replay_oracle_success.png")

                result = {
                    "run_id": run_id,
                    "episode_id": episode_id,
                    "operator_kind": "agent_ui",
                    "view_source": "third_person_3d",
                    "driven_base_x_m": driven["base"]["position_world_sim_m"][0],
                    "stopped_base_x_m": stopped["base"]["position_world_sim_m"][0],
                    "above": {
                        "tcp_m": above["tcp"]["position_m"],
                        "error_m": above["tcp"]["position_error_m"],
                    },
                    "grasp": {
                        "tcp_m": at_cube["tcp"]["position_m"],
                        "error_m": at_cube["tcp"]["position_error_m"],
                        "width_m": grasped["gripper"]["width_m"],
                        "left_force_n": settled["gripper"]["left_contact_force_n"],
                        "right_force_n": settled["gripper"]["right_contact_force_n"],
                    },
                    "lift": {
                        "target_m": lift,
                        "tcp_m": lifted["tcp"]["position_m"],
                        "error_m": lifted["tcp"]["position_error_m"],
                        "cube_z_m": hold_samples[-1]["cube_z_m"],
                        "height_m": hold_samples[-1]["cube_z_m"] - 0.10990787854182482,
                        "hold_samples": len(hold_samples),
                    },
                    "validation": validation,
                    "assessment": assessment,
                    "eligibility": eligibility,
                    "recording": completed["recording"],
                    "media": detail.get("media"),
                    "event_count": len(detail.get("events", [])),
                }
            finally:
                heartbeat_stop.set()
                if heartbeat_thread is not None:
                    heartbeat_thread.join(timeout=2)
                if recording_started:
                    try:
                        client.post(
                            "/api/v1/episodes/stop",
                            json={"reason": "ACCEPTANCE_INTERRUPTED"},
                            timeout=180,
                        )
                    except Exception:
                        pass
                browser.close()

    evidence_path = EVIDENCE / "g1_success_acceptance.json"
    evidence_path.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result.get("validation", {}).get("passed") and result.get(
        "assessment", {}
    ).get("task_result") == "success" else 2


def heartbeat_loop(url: str, session_id: str, stop: threading.Event) -> None:
    with httpx.Client(base_url=url, timeout=10.0) as client:
        while not stop.wait(0.1):
            try:
                client.post(
                    "/api/v1/lease/renew",
                    json={"session_id": session_id},
                ).raise_for_status()
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())

