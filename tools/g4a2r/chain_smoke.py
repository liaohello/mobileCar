#!/usr/bin/env python3
"""G4A-2R Phase 3 server-only smoke for the six-dimensional action chain.

Fixtures only (synthetic API, real page HTML/JS from this release):

  1. display frame vs control observation: a new frame arriving between intent
     and action must NOT break the intent's display binding; the action's
     display_sample_id stays the intent frame while observation_sample_id is
     the latest sample;
  2. chain feedback: 发送 -> 已接收 -> 执行中 -> 已到达 (arm converges in the
     fixture observation stream);
  3. unchanged target: sending the same target after reaching shows 目标未改变
     and does not create another action;
  4. rejection: a rejected action shows 被拒绝 with the server reason;
  5. disabled reasons: without authorization the send button explains why.

Never counts as human demonstration; no real backend is touched.
"""

from __future__ import annotations

import base64
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parents[2]
PNG = base64.b64decode(
    "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNk+A8AAQUBAScY42YAAAAASUVORK5CYII="
)


class State:
    mode = "IDLE"
    sample = 0
    target = [0.0, 0.0, 0.0, 0.0, 0.0]
    actual = [0.0, 0.0, 0.0, 0.0, 0.0]
    gripper_target = 0.025
    gripper_actual = 0.025
    intents: list[dict] = []
    actions: list[dict] = []
    reject_next = False


FIXTURE = State()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_args):
        return

    def _json(self, value, status=200):
        body = json.dumps(value).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/api/v1/health":
            return self._json(
                {"status": "ok", "backend_generation": 1, "recording_status": FIXTURE.mode}
            )
        if path == "/api/v1/capabilities":
            return self._json({"model_sha256": "d" * 64, "g4a2_postures": []})
        if path == "/api/v1/g4a2/observation":
            if FIXTURE.mode != "RECORDING":
                return self._json({"detail": "observation paused"}, 404)
            # arm approaches the last accepted target
            for index in range(5):
                delta = FIXTURE.target[index] - FIXTURE.actual[index]
                FIXTURE.actual[index] += delta * 0.55
                if abs(delta) < 1e-4:
                    FIXTURE.actual[index] = FIXTURE.target[index]
            grip_delta = FIXTURE.gripper_target - FIXTURE.gripper_actual
            FIXTURE.gripper_actual += grip_delta * 0.55
            if abs(grip_delta) < 5e-5:
                FIXTURE.gripper_actual = FIXTURE.gripper_target
            FIXTURE.sample += 1
            return self._json(
                {
                    "run_id": "fixture_run",
                    "episode_id": "fixture_episode",
                    "backend_generation": 1,
                    "sample_id": f"rgbd_chain_{FIXTURE.sample}",
                    "valid_fraction": 0.8,
                    "center_depth_m": 0.25,
                    "rgb_uri": f"/frame.png?s={FIXTURE.sample}",
                    "arm_user_joint_positions_rad": list(FIXTURE.actual),
                    "gripper_width_m": FIXTURE.gripper_actual,
                    "observation_profile_sha256": "1" * 64,
                    "action_profile_sha256": "2" * 64,
                    "quality": "simulated_nominal",
                }
            )
        if path == "/frame.png":
            self.send_response(200)
            self.send_header("Content-Type", "image/png")
            self.send_header("Content-Length", str(len(PNG)))
            self.end_headers()
            self.wfile.write(PNG)
            return
        files = {
            "/g4a2": ROOT / "src/mobilecar/ui/static/g4a2.html",
            "/static/g4a2.js": ROOT / "src/mobilecar/ui/static/g4a2.js",
            "/static/style.css": ROOT / "src/mobilecar/ui/static/style.css",
        }
        if path.startswith("/api/"):
            return self._json({"detail": "fixture not found"}, 404)
        target = files.get(path)
        if target is None or not target.is_file():
            return self._json({"detail": "not found"}, 404)
        body = target.read_bytes()
        content_type = "text/html" if target.suffix == ".html" else "application/javascript" if target.suffix == ".js" else "text/css"
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        length = int(self.headers.get("Content-Length") or 0)
        payload = json.loads(self.rfile.read(length) or b"{}")
        if path == "/api/v1/episodes/start":
            FIXTURE.mode = "RECORDING"
            return self._json({"accepted": True, "episode_id": "fixture_episode"})
        if path == "/api/v1/episodes/stop":
            FIXTURE.mode = "COMPLETE"
            return self._json({"accepted": True, "episode_id": "fixture_episode"})
        if path == "/api/v1/g4a2/session":
            return self._json({"accepted": True, "human_session_id": "fixture_human_session"})
        if path == "/api/v1/g4a1/permission":
            return self._json({"accepted": True, "permission_id": "fixture_permission", "nearfield_epoch": 1})
        if path == "/api/v1/g4a1/revoke":
            return self._json({"accepted": True})
        if path == "/api/v1/g4a2/intent":
            FIXTURE.intents.append(payload)
            return self._json(
                {"accepted": True, "human_intent_id": f"fixture_intent_{len(FIXTURE.intents)}", "display_sample_id": payload.get("display_sample_id")}
            )
        if path == "/api/v1/g4a2/action":
            FIXTURE.actions.append(payload)
            if FIXTURE.reject_next:
                FIXTURE.reject_next = False
                return self._json({"accepted": False, "reason": "fixture rejection: stale frame", "action_seq": payload.get("action_seq")})
            FIXTURE.target = list(payload.get("arm_user_joint_positions_rad") or FIXTURE.target)
            FIXTURE.gripper_target = float(payload.get("gripper_width_m") or FIXTURE.gripper_target)
            return self._json({"accepted": True, "action_seq": payload.get("action_seq"), "control_tick_id": "fixture_tick"})
        return self._json({"detail": "fixture POST not found"}, 404)


def main() -> int:
    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    url = f"http://127.0.0.1:{server.server_address[1]}/g4a2"
    out = ROOT / "var/reports/g4a2r-chain-smoke"
    out.mkdir(parents=True, exist_ok=True)
    result = {"synthetic_fixture": True, "human_evidence": False, "checks": {}}

    def check(name: str, value: bool) -> None:
        result["checks"][name] = bool(value)

    try:
        with sync_playwright() as playwright:
            browser = playwright.chromium.launch(headless=True)
            page = browser.new_page(viewport={"width": 1280, "height": 1000})
            page_errors: list[str] = []
            page.on("pageerror", lambda error: page_errors.append(str(error)))
            page.goto(url, wait_until="domcontentloaded")
            page.wait_for_function("document.querySelector('#g4a2-status')?.textContent.includes('后端')", timeout=10000)
            page.locator("#human-record").click()
            page.wait_for_function("document.querySelector('#recording-lifecycle')?.textContent === '正在录制'", timeout=5000)
            page.wait_for_function("document.querySelector('#frame-lifecycle')?.textContent.trim() === '实时'", timeout=8000)

            # Disabled reason before confirmation / authorization
            reason_before = page.locator("#human-send").get_attribute("title") or ""
            check("disabled_reason_before_confirmation", "尚未确认本人操作" in reason_before or page.locator("#human-send").is_disabled())
            page.locator("#human-confirm").click()
            page.wait_for_function("document.querySelector('#human-session')?.textContent !== '未确认'", timeout=5000)
            reason_before_auth = page.locator("#human-send").get_attribute("title") or ""
            check("disabled_reason_before_authorize", "尚未启用近场权限" in reason_before_auth)
            page.locator("#human-authorize").click()
            page.wait_for_timeout(500)
            send_enabled = not page.locator("#human-send").is_disabled()
            check("send_enabled_after_authorize", send_enabled)
            if not send_enabled:
                print(json.dumps({"error": "send stayed disabled", "title": page.locator("#human-send").get_attribute("title")}, ensure_ascii=False))
                raise SystemExit(2)

            # --- 1. intent binding while the arm is still applying ---------
            page.evaluate(
                """() => {
                  const input = document.querySelector('[data-joint-index="1"]');
                  input.value = '-1.0';
                  input.dispatchEvent(new Event('input', { bubbles: true }));
                }"""
            )
            page.locator("#human-send").click()
            page.wait_for_function("document.querySelector('#g4a2-log')?.textContent.includes('action 1 dispatched')", timeout=5000)
            page.wait_for_timeout(250)
            page.locator("#human-send").click()
            page.wait_for_function("document.querySelector('#g4a2-log')?.textContent.includes('action 2 dispatched')", timeout=5000)
            intents = list(FIXTURE.intents)
            actions = list(FIXTURE.actions)
            check("single_intent_for_unchanged_target", len(intents) == 1)
            check(
                "action_display_bound_to_intent",
                actions[-1]["display_sample_id"] == intents[-1]["display_sample_id"],
            )
            check(
                "action_uses_newer_control_observation",
                actions[-1]["observation_sample_id"] != actions[-1]["display_sample_id"],
            )

            # --- 2. accepted -> applying -> reached -----------------------
            page.wait_for_function("document.querySelector('#chain-lifecycle')?.textContent === '已到达'", timeout=10000)
            check("chain_reached", True)
            chain_text = page.locator("#human-chain-status").inner_text()
            check("reached_text", "已到达" in chain_text)
            page.screenshot(path=out / "chain_reached.png", full_page=True)

            # --- 3. unchanged target --------------------------------------
            actions_before = len(FIXTURE.actions)
            page.locator("#human-send").click()
            page.wait_for_function("document.querySelector('#chain-lifecycle')?.textContent === '目标未改变'", timeout=5000)
            page.wait_for_timeout(300)
            check("unchanged_no_new_action", len(FIXTURE.actions) == actions_before)
            check("unchanged_text", "目标未改变" in page.locator("#human-chain-status").inner_text())

            # --- 4. rejection visible -------------------------------------
            FIXTURE.reject_next = True
            page.evaluate(
                """() => {
                  const input = document.querySelector('[data-joint-index="1"]');
                  input.value = '-1.3';
                  input.dispatchEvent(new Event('input', { bubbles: true }));
                }"""
            )
            page.locator("#human-send").click()
            page.wait_for_function("document.querySelector('#chain-lifecycle')?.textContent === '被拒绝'", timeout=5000)
            rejected_text = page.locator("#human-chain-status").inner_text()
            check("rejection_reason_visible", "stale frame" in rejected_text)
            page.screenshot(path=out / "chain_rejected.png", full_page=True)

            check("no_page_errors", not page_errors)
            result["page_errors"] = page_errors
            result["intents"] = intents
            result["actions"] = actions
            browser.close()
    finally:
        server.shutdown()
    required = (
        "disabled_reason_before_confirmation",
        "disabled_reason_before_authorize",
        "send_enabled_after_authorize",
        "single_intent_for_unchanged_target",
        "action_display_bound_to_intent",
        "action_uses_newer_control_observation",
        "chain_reached",
        "reached_text",
        "unchanged_no_new_action",
        "unchanged_text",
        "rejection_reason_visible",
        "no_page_errors",
    )
    result["passed"] = all(result["checks"].get(name) for name in required)
    (out / "chain_smoke.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"passed": result["passed"], "checks": result["checks"]}, ensure_ascii=False))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
