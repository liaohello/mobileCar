#!/usr/bin/env python3
"""End-to-end validation of the browser base-drive chain against a live service.

The chain under test is the one reported as broken:

    key/pointer event -> target twist -> HTTP command -> backend accept/reject
    -> control tick -> wheel motion -> measured base displacement

Input is real Chromium keyboard and pointer input. The Episode is recorded with
``operator_kind=scripted`` and no human session is ever confirmed, so nothing in
this report is a human demonstration or training-qualified data.

The workspace has a 0.10 m platform whose front face sits at x = 0.30 m, so each
forward push is followed by a backward return; otherwise the base would stall
against the step and a near-zero velocity would make later checks vacuous.

Run on the server against one (and only one) MuJoCo instance::

    envs/g2a-py310-mujoco316/bin/python tools/drive/validate_drive_chain.py \
        --base http://127.0.0.1:8795 --out var/reports/drive_validation/report.json
"""

from __future__ import annotations

import argparse
import json
import math
import time
import urllib.error
import urllib.request
import uuid
from pathlib import Path
from typing import Any, Callable

from playwright.sync_api import Page, sync_playwright

WHEEL_JOINTS = ("front_left_joint", "rear_left_joint", "front_right_joint", "rear_right_joint")
TRANSLATION_LIMIT = 0.006
YAW_LIMIT = 0.02


class Api:
    def __init__(self, base: str) -> None:
        self.base = base.rstrip("/")
        self.session = "drive_validation_" + uuid.uuid4().hex
        self.seq = 0

    def call(self, method: str, path: str, body: Any = None) -> Any:
        data = None if body is None else json.dumps(body).encode()
        request = urllib.request.Request(
            self.base + path,
            data=data,
            headers={"Content-Type": "application/json"},
            method=method,
        )
        try:
            with urllib.request.urlopen(request, timeout=30) as response:
                return json.load(response)
        except urllib.error.HTTPError as exc:  # noqa: PERF203 - surfaced verbatim
            raise RuntimeError(f"{method} {path} -> {exc.code}: {exc.read().decode()}") from exc

    def snapshot(self) -> dict[str, Any]:
        return self.call("GET", "/api/v1/snapshot")

    def health(self) -> dict[str, Any]:
        return self.call("GET", "/api/v1/health")

    def stop_latch(self, latch: bool) -> dict[str, Any]:
        snapshot = self.snapshot()
        self.seq += 1
        return self.call(
            "POST",
            "/api/v1/commands",
            {
                "contract_version": "mobilecar-1.0",
                "run_id": snapshot["run_id"],
                "backend_generation": snapshot["backend_generation"],
                "episode_id": snapshot["episode_id"],
                "session_id": self.session,
                "operator_id": "drive_validation",
                "operator_kind": "scripted",
                "view_source": "drive_validation",
                "command_id": "cmd_" + uuid.uuid4().hex,
                "command_seq": self.seq,
                "command_type": "stop",
                "requested_ttl_s": 5.0,
                "payload": {"latch": latch, "reason": "drive_validation_precondition"},
            },
        )

    def start_episode(self) -> dict[str, Any]:
        return self.call(
            "POST",
            "/api/v1/episodes/start",
            {
                "task_language": "Drive validation: W/S/A/D/Q/E with stop, blur and disconnect checks.",
                "operator_id": "drive_validation",
                "operator_kind": "scripted",
                "view_source": "third_person_3d",
                "map_mode": "unavailable",
                "require_recording": True,
            },
        )

    def stop_episode(self) -> dict[str, Any]:
        return self.call(
            "POST", "/api/v1/episodes/stop", {"reason": "DRIVE_VALIDATION_FINISHED"}
        )


def wait_for(predicate: Callable[[], Any], timeout: float, interval: float = 0.05) -> Any:
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        last = predicate()
        if last:
            return last
        time.sleep(interval)
    raise TimeoutError(f"condition not met within {timeout}s (last={last!r})")


def yaw_of(quat_xyzw: list[float]) -> float:
    x, y, z, w = quat_xyzw
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def pose(snapshot: dict[str, Any]) -> dict[str, Any]:
    base = snapshot["base"]
    wheels = {
        name: float(snapshot.get("model_joint_positions_rad", {}).get(name, 0.0))
        for name in WHEEL_JOINTS
    }
    return {
        "position_m": list(base["position_world_sim_m"]),
        "yaw_rad": yaw_of(base["orientation_world_sim_xyzw"]),
        "velocity": [base["vx_m_s"], base["vy_m_s"], base["wz_rad_s"]],
        "requested": [
            base.get("requested_vx_m_s"),
            base.get("requested_vy_m_s"),
            base.get("requested_wz_rad_s"),
        ],
        "wheels_rad": wheels,
        "sim_time_s": snapshot["sim_time_s"],
        "control_tick_id": snapshot["control_tick_id"],
        "recording_status": snapshot["recording"]["status"],
    }


def speed(item: dict[str, Any]) -> float:
    return math.hypot(item["velocity"][0], item["velocity"][1])


def delta(before: dict[str, Any], after: dict[str, Any]) -> dict[str, Any]:
    return {
        "dx_m": after["position_m"][0] - before["position_m"][0],
        "dy_m": after["position_m"][1] - before["position_m"][1],
        "dyaw_rad": wrap(after["yaw_rad"] - before["yaw_rad"]),
        "wheel_delta_rad": {
            name: wrap(after["wheels_rad"][name] - before["wheels_rad"][name])
            for name in WHEEL_JOINTS
        },
        "sim_time_s": after["sim_time_s"] - before["sim_time_s"],
    }


def gate(page: Page) -> dict[str, str]:
    return {
        "state": page.locator("#drive-gate").get_attribute("data-state") or "",
        "code": page.locator("#drive-gate").get_attribute("data-code") or "",
        "title": page.locator("#drive-gate-title").inner_text(),
        "detail": page.locator("#drive-gate-detail").inner_text(),
    }


def readouts(page: Page) -> dict[str, dict[str, str]]:
    return {
        axis: {
            "target": page.locator(f"#drive-intent-{axis}").inner_text(),
            "accepted": page.locator(f"#drive-accepted-{axis}").inner_text(),
            "actual": page.locator(f"#base-{axis}").inner_text(),
        }
        for axis in ("vx", "vy", "wz")
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base", default="http://127.0.0.1:8795")
    parser.add_argument("--out", required=True)
    parser.add_argument("--screenshot", default=None)
    parser.add_argument("--hold-s", type=float, default=0.75)
    parser.add_argument("--settle-s", type=float, default=1.2)
    args = parser.parse_args()

    api = Api(args.base)
    report: dict[str, Any] = {
        "generated_wall_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "base_url": api.base,
        "source": "automation_scripted",
        "human_session_confirmed": False,
        "training_qualified": False,
        "cases": [],
        "checks": [],
    }

    def check(name: str, passed: bool, detail: Any = None) -> None:
        """Record an outcome; keep going so one failure still yields a receipt."""

        report["checks"].append({"name": name, "passed": bool(passed), "detail": detail})

    def require(name: str, passed: bool, detail: Any = None) -> None:
        check(name, passed, detail)
        if not passed:
            raise AssertionError(f"precondition failed: {name}: {detail!r}")

    health = api.health()
    report["health"] = health
    require(
        "service_declares_recording_policy",
        isinstance(health.get("recording_required_for_motion"), bool),
        health.get("recording_required_for_motion"),
    )

    initial = api.snapshot()
    report["initial_snapshot"] = pose(initial)
    report["initial_gate"] = initial["diagnostics"].get("base_drive_gate")
    require(
        "initial_gate_blocks_drive",
        report["initial_gate"]["blocked"]
        and report["initial_gate"]["code"]
        in {"RECORDING_REQUIRED", "SOFTWARE_STOP_LATCHED"},
        report["initial_gate"],
    )

    started = api.start_episode()
    report["episode_start"] = started
    if not started.get("accepted") and "already RECORDING" in str(started.get("reason", "")):
        # A previous aborted run may have left an active recording; reusing it is
        # safe because this script owns no human data.
        report["episode_start"] = {**started, "reused_active_recording": True}
        started = {"accepted": True, "reason": "reused_active_recording"}
    require("episode_started", bool(started.get("accepted")), report["episode_start"])
    wait_for(
        lambda: api.snapshot()["recording"]["status"] == "RECORDING"
        and api.snapshot()["recording"]["command_capture_ready"],
        timeout=20,
    )
    if api.snapshot()["safety_latched"]:
        report["precondition_unlock"] = api.stop_latch(False)
        wait_for(lambda: not api.snapshot()["safety_latched"], timeout=5)
    ready = api.snapshot()
    report["episode_id"] = ready["episode_id"]
    report["run_id"] = ready["run_id"]
    report["model_sha256"] = api.call("GET", "/api/v1/capabilities")["model_sha256"]
    report["gate_after_recording"] = ready["diagnostics"].get("base_drive_gate")
    require(
        "gate_ready_after_recording",
        ready["diagnostics"]["base_drive_gate"]["code"] == "READY",
        ready["diagnostics"]["base_drive_gate"],
    )

    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(
            headless=True,
            args=[
                "--use-angle=vulkan",
                "--enable-features=Vulkan",
                "--disable-vulkan-surface",
                "--ignore-gpu-blocklist",
            ],
        )
        page = browser.new_page(viewport={"width": 1440, "height": 900})
        console: list[str] = []
        page.on("pageerror", lambda error: console.append(f"pageerror:{error}"))
        page.goto(api.base, wait_until="domcontentloaded", timeout=30000)
        page.wait_for_function(
            "document.querySelector('#connection-text')?.textContent === '已连接'",
            timeout=20000,
        )
        require("page_connected", True)
        page.wait_for_function(
            "document.querySelector('#drive-gate')?.dataset.code === 'READY'", timeout=10000
        )
        report["gate_ready_visible"] = gate(page)
        require("drive_gate_visible_ready", report["gate_ready_visible"]["state"] == "ready")
        policy = page.locator("[data-testid='recording-policy']").inner_text()
        require("recording_policy_forced_in_ui", "服务端强制" in policy, policy)

        # Guarantee platform clearance even when this instance was left parked
        # close to the step by an earlier run.
        page.keyboard.down("s")
        time.sleep(0.9)
        page.keyboard.up("s")
        time.sleep(1.4)
        report["clearance_start"] = pose(api.snapshot())

        def hold(key: str, name: str, seconds: float | None = None) -> dict[str, Any]:
            hold_s = args.hold_s if seconds is None else seconds
            before = pose(api.snapshot())
            page.keyboard.down(key)
            time.sleep(hold_s)
            while_held = pose(api.snapshot())
            ui = readouts(page)
            page.keyboard.up(key)
            time.sleep(args.settle_s)
            after = pose(api.snapshot())
            return {
                "name": name,
                "key": key,
                "hold_s": hold_s,
                "before": before,
                "while_held": while_held,
                "after": after,
                "delta": delta(before, after),
                "ui_readout": ui,
                "speed_while_held_m_s": speed(while_held),
                "velocity_after_release": after["velocity"],
                "stopped_after_release": api.snapshot()["base"]["stopped"],
            }

        def return_to_start(tag: str) -> None:
            """Drive backward so the next forward push has platform clearance."""

            before = pose(api.snapshot())
            page.keyboard.down("s")
            time.sleep(0.75)
            page.keyboard.up("s")
            time.sleep(args.settle_s)
            after = pose(api.snapshot())
            report.setdefault("returns", []).append(
                {"tag": tag, "before_x": before["position_m"][0], "after_x": after["position_m"][0]}
            )

        for key, name, axis, minimum in (
            ("w", "W_forward", "dx", 0.02),
            ("s", "S_backward", "dx", -0.015),
            ("a", "A_strafe_left", "dy", 0.015),
            ("d", "D_strafe_right", "dy", -0.015),
        ):
            case = hold(key, name, 1.0 if key == "w" else None)
            report["cases"].append(case)
            value = case["delta"]["dx_m"] if axis == "dx" else case["delta"]["dy_m"]
            check(
                f"{name}_direction",
                (value >= minimum) if minimum > 0 else (value <= minimum),
                case["delta"],
            )
            check(f"{name}_moved_base", abs(value) > 0.012, {"value": value})
            check(
                f"{name}_was_actually_moving",
                case["speed_while_held_m_s"] > 0.02 or abs(case["while_held"]["velocity"][2]) > 0.05,
                {"speed_while_held_m_s": case["speed_while_held_m_s"], "velocity": case["while_held"]["velocity"]},
            )
            check(
                f"{name}_wheels_turned",
                max(abs(v) for v in case["delta"]["wheel_delta_rad"].values()) > 0.05,
                case["delta"]["wheel_delta_rad"],
            )
            check(
                f"{name}_released_to_zero",
                abs(case["velocity_after_release"][0]) < TRANSLATION_LIMIT
                and abs(case["velocity_after_release"][1]) < TRANSLATION_LIMIT
                and abs(case["velocity_after_release"][2]) < YAW_LIMIT,
                case["velocity_after_release"],
            )
            case["stopped_flag"] = case.pop("stopped_after_release")
            if key == "w":
                return_to_start("after_W")

        for key, name, sign in (("q", "Q_yaw_left", 1), ("e", "E_yaw_right", -1)):
            case = hold(key, name, 1.0 if key == "w" else None)
            report["cases"].append(case)
            check(
                f"{name}_direction",
                case["delta"]["dyaw_rad"] * sign >= 0.08,
                case["delta"],
            )
            check(
                f"{name}_was_actually_moving",
                abs(case["while_held"]["velocity"][2]) > 0.05,
                case["while_held"]["velocity"],
            )
            check(
                f"{name}_wheels_turned",
                max(abs(v) for v in case["delta"]["wheel_delta_rad"].values()) > 0.05,
                case["delta"]["wheel_delta_rad"],
            )

        # --- key release and blur must stop a moving base -----------------
        def stop_timing(action: Callable[[], None], label: str) -> dict[str, Any]:
            """Dispatch one stop trigger and time the zero request and standstill."""

            action()
            started = time.monotonic()
            zero_at = None
            stop_at = None
            deadline = started + 4.0
            while time.monotonic() < deadline:
                snapshot = api.snapshot()
                base = snapshot["base"]
                if zero_at is None and max(
                    abs(base.get("requested_vx_m_s") or 0.0),
                    abs(base.get("requested_vy_m_s") or 0.0),
                    abs(base.get("requested_wz_rad_s") or 0.0),
                ) <= 1e-9:
                    zero_at = time.monotonic() - started
                if (
                    abs(base["vx_m_s"]) < TRANSLATION_LIMIT
                    and abs(base["vy_m_s"]) < TRANSLATION_LIMIT
                    and abs(base["wz_rad_s"]) < YAW_LIMIT
                ):
                    stop_at = time.monotonic() - started
                    if zero_at is not None:
                        break
                time.sleep(0.02)
            final = pose(api.snapshot())
            return {
                "label": label,
                "zero_request_latency_s": zero_at,
                "standstill_latency_s": stop_at,
                "stopped_flag": api.snapshot()["base"]["stopped"],
                "final_velocity": final["velocity"],
                "final_position": final["position_m"],
            }

        page.keyboard.down("w")
        time.sleep(0.5)
        moving_release = pose(api.snapshot())
        release_timing = stop_timing(lambda: page.keyboard.up("w"), "key_release")
        released = pose(api.snapshot())
        report["key_release"] = {
            "moving": moving_release,
            "after": released,
            "timing": release_timing,
        }
        check("key_release_started_from_motion", speed(moving_release) > 0.02, moving_release["velocity"])
        check(
            "key_release_zero_request",
            release_timing["zero_request_latency_s"] is not None
            and release_timing["zero_request_latency_s"] < 1.0,
            release_timing,
        )
        check(
            "key_release_standstill",
            release_timing["standstill_latency_s"] is not None
            and release_timing["standstill_latency_s"] < 2.0,
            release_timing,
        )
        return_to_start("after_key_release")

        page.keyboard.down("w")
        time.sleep(0.5)
        moving_blur = pose(api.snapshot())
        blur_timing = stop_timing(
            lambda: page.evaluate("window.dispatchEvent(new Event('blur'))"), "blur"
        )
        page.keyboard.up("w")
        blurred = pose(api.snapshot())
        report["blur"] = {
            "moving": moving_blur,
            "after": blurred,
            "timing": blur_timing,
            "keys_after_blur": page.evaluate("window.mobileCarApp.keys.size"),
        }
        check("blur_started_from_motion", speed(moving_blur) > 0.02, moving_blur["velocity"])
        check("blur_releases_keys", report["blur"]["keys_after_blur"] == 0, report["blur"])
        check(
            "blur_zero_request",
            blur_timing["zero_request_latency_s"] is not None
            and blur_timing["zero_request_latency_s"] < 1.0,
            blur_timing,
        )
        check(
            "blur_standstill",
            blur_timing["standstill_latency_s"] is not None
            and blur_timing["standstill_latency_s"] < 2.0,
            blur_timing,
        )
        return_to_start("after_blur")

        # --- on-screen pointer drive --------------------------------------
        button = page.locator("[data-drive-key='w']")
        box = button.bounding_box()
        before = pose(api.snapshot())
        page.mouse.move(box["x"] + box["width"] / 2, box["y"] + box["height"] / 2)
        page.mouse.down()
        time.sleep(args.hold_s)
        held = pose(api.snapshot())
        pointer_ui = readouts(page)
        page.mouse.up()
        time.sleep(args.settle_s)
        after = pose(api.snapshot())
        report["onscreen_key"] = {
            "before": before,
            "while_held": held,
            "after": after,
            "delta": delta(before, after),
            "ui_readout": pointer_ui,
            "speed_while_held_m_s": speed(held),
        }
        check(
            "onscreen_W_was_actually_moving",
            speed(held) > 0.02,
            held["velocity"],
        )
        check(
            "onscreen_W_moves_forward",
            report["onscreen_key"]["delta"]["dx_m"] >= 0.015,
            report["onscreen_key"]["delta"],
        )
        report["onscreen_key"]["stopped"] = api.snapshot()["base"]["stopped"]
        check(
            "onscreen_W_release_stops",
            max(abs(v) for v in after["velocity"]) < TRANSLATION_LIMIT
            or report["onscreen_key"]["stopped"],
            after["velocity"],
        )
        return_to_start("after_onscreen")

        # --- software stop gate + backend rejection surfacing -------------
        page.locator("#stop-button").click()
        page.wait_for_function(
            "document.querySelector('#drive-gate')?.dataset.code === 'SOFTWARE_STOP_LATCHED'",
            timeout=5000,
        )
        report["latched_gate"] = gate(page)
        check("latched_gate_visible", report["latched_gate"]["state"] == "blocked", report["latched_gate"])
        check(
            "latched_gate_explains_unlock",
            "解除" in report["latched_gate"]["title"] + report["latched_gate"]["detail"],
            report["latched_gate"],
        )
        page.evaluate(
            """() => {
              window.__driveKeyLog = [];
              document.addEventListener("keydown", (event) => {
                window.__driveKeyLog.push({
                  key: event.key,
                  target: event.target.tagName,
                  editable: Boolean(event.target.closest(
                    "input, textarea, select, [contenteditable='true'], [role='textbox']")),
                });
              }, true);
            }"""
        )
        page.evaluate(
            """() => {
              const app = window.mobileCarApp;
              window.__calls = [];
              const original = app.pressDriveKey.bind(app);
              app.pressDriveKey = (key) => {
                window.__calls.push({
                  key,
                  latched: app.snapshot?.safety_latched,
                  resolved: app.resolveDriveGate(),
                  keys_before: app.keys.size,
                });
                return original(key);
              };
            }"""
        )
        seq_before = page.evaluate("window.mobileCarApp.commandSeq")
        page.keyboard.down("w")
        time.sleep(0.25)
        during = page.evaluate(
            """() => ({
              latched: window.mobileCarApp.snapshot?.safety_latched,
              gateField: window.mobileCarApp.snapshot?.diagnostics?.base_drive_gate,
              resolved: window.mobileCarApp.resolveDriveGate(),
              keys: window.mobileCarApp.keys.size,
              notice: window.mobileCarApp.driveGateNotice,
              detail: document.querySelector('#drive-gate-detail').textContent,
            })"""
        )
        time.sleep(0.15)
        page.keyboard.up("w")
        report["latched_key_interception"] = {
            "keys": page.evaluate("window.mobileCarApp.keys.size"),
            "command_seq_before": seq_before,
            "command_seq_after": page.evaluate("window.mobileCarApp.commandSeq"),
            "key_log": page.evaluate("window.__driveKeyLog"),
            "notice": page.evaluate("window.mobileCarApp.driveGateNotice"),
            "detail": page.locator("#drive-gate-detail").inner_text(),
            "during_hold": during,
            "press_calls": page.evaluate("window.__calls"),
            "aria_disabled": page.locator("[data-drive-key='w']").get_attribute("aria-disabled"),
        }
        intercepted = report["latched_key_interception"]
        check(
            "latched_key_is_blocked_before_http",
            intercepted["keys"] == 0
            and intercepted["command_seq_after"] == intercepted["command_seq_before"]
            and any(item["key"] == "w" and not item["editable"] for item in intercepted["key_log"]),
            intercepted,
        )
        check(
            "latched_key_shows_interception_notice",
            "已拦截" in intercepted["detail"] or bool(intercepted["notice"]),
            intercepted,
        )

        rejected = page.evaluate(
            """async () => {
              const app = window.mobileCarApp;
              return app.sendCommand(
                "set_base_twist",
                {frame_id: "base_link", vx_m_s: 0.0, vy_m_s: 0.0, wz_rad_s: 0.0},
                1.0,
                {quiet: true, priority: true},
              );
            }"""
        )
        page.wait_for_function("!document.querySelector('#drive-reject').hidden", timeout=4000)
        report["backend_rejection"] = {
            "response": rejected,
            "banner": page.locator("[data-testid='drive-reject']").inner_text(),
        }
        check(
            "backend_rejection_surfaced",
            rejected.get("status") == "REJECTED"
            and "software stop is latched" in report["backend_rejection"]["banner"],
            report["backend_rejection"],
        )

        page.locator("#unlock-stop").click()
        page.wait_for_function(
            "document.querySelector('#drive-gate')?.dataset.code === 'READY'", timeout=8000
        )
        require("unlock_restores_ready", True)

        # --- documented recovery path: stop -> unlock -> start recording -> drive
        ended = api.stop_episode()
        report["recovery_episode_stop"] = ended
        page.wait_for_function(
            "document.querySelector('#drive-gate')?.dataset.code === 'SOFTWARE_STOP_LATCHED'",
            timeout=15000,
        )
        page.locator("#drive-gate-action").click()
        page.wait_for_function(
            "document.querySelector('#drive-gate')?.dataset.code === 'RECORDING_REQUIRED'",
            timeout=15000,
        )
        report["recovery_gate_recording_required"] = gate(page)
        require(
            "gate_action_unlocks_stop",
            report["recovery_gate_recording_required"]["code"] == "RECORDING_REQUIRED",
            report["recovery_gate_recording_required"],
        )
        page.locator("#drive-gate-action").click()
        page.wait_for_function(
            "document.querySelector('#drive-gate')?.dataset.code === 'READY'", timeout=30000
        )
        page.wait_for_function(
            "document.querySelector('#recording-state')?.textContent === 'RECORDING'",
            timeout=30000,
        )
        report["recovery_gate_ready"] = gate(page)
        report["recovery_episode_id"] = api.snapshot()["episode_id"]
        before_recovery = pose(api.snapshot())
        page.keyboard.down("w")
        time.sleep(args.hold_s)
        while_recovery = pose(api.snapshot())
        page.keyboard.up("w")
        time.sleep(args.settle_s)
        after_recovery = pose(api.snapshot())
        report["recovery_drive"] = {
            "before": before_recovery,
            "while_held": while_recovery,
            "after": after_recovery,
            "delta": delta(before_recovery, after_recovery),
            "speed_while_held_m_s": speed(while_recovery),
        }
        require(
            "gate_action_starts_recording_and_drive_works",
            report["recovery_drive"]["delta"]["dx_m"] >= 0.015
            and report["recovery_drive"]["speed_while_held_m_s"] > 0.02,
            report["recovery_drive"],
        )
        return_to_start("after_recovery")

        if args.screenshot:
            page.screenshot(path=args.screenshot, full_page=True)
            report["screenshot"] = args.screenshot

        # --- disconnect while genuinely moving backward -------------------
        page.keyboard.down("s")
        time.sleep(0.6)
        moving = pose(api.snapshot())
        page.close()
        t_close = time.monotonic()
        time.sleep(1.2)
        after_disconnect = pose(api.snapshot())
        owner_after = api.snapshot()["base_owner_session_id"]
        time.sleep(0.6)
        final = pose(api.snapshot())
        report["disconnect"] = {
            "moving": moving,
            "after_disconnect": after_disconnect,
            "owner_after": owner_after,
            "final": final,
            "stable_after_stop_m": math.dist(
                after_disconnect["position_m"][:2], final["position_m"][:2]
            ),
            "seconds_after_close": time.monotonic() - t_close,
        }
        check("disconnect_started_from_motion", speed(moving) > 0.02, moving["velocity"])
        check(
            "disconnect_stops_base",
            max(abs(v) for v in after_disconnect["velocity"]) < 0.01
            and report["disconnect"]["stable_after_stop_m"] < 0.004,
            report["disconnect"],
        )
        check("disconnect_releases_lease", owner_after is None, owner_after)

        report["console"] = console
        check("no_page_errors", not console, console)
        browser.close()

    stopped = api.stop_episode()
    report["episode_stop"] = stopped
    check("episode_stopped", bool(stopped.get("accepted")), stopped)
    final_snapshot = api.snapshot()
    report["final_snapshot"] = pose(final_snapshot)
    report["final_gate"] = final_snapshot["diagnostics"].get("base_drive_gate")
    check(
        "final_gate_latched_after_stop",
        report["final_gate"]["code"] == "SOFTWARE_STOP_LATCHED",
        report["final_gate"],
    )

    report["passed"] = all(item["passed"] for item in report["checks"])
    report["summary"] = {
        "checks": len(report["checks"]),
        "failed": [item["name"] for item in report["checks"] if not item["passed"]],
        "cases": [item["name"] for item in report["cases"]],
    }
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(report["summary"], indent=2, ensure_ascii=False))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
