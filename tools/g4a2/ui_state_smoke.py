#!/usr/bin/env python3
"""Server-side fixture smoke for G4A-2 recording/image lifecycle UI states."""

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
    fail_stop = False
    broken_image = False
    sample = 0
    image_ok = False
    last_action_sample = None


FIXTURE = State()


class StaticHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        path = self.path.split("?", 1)[0]
        files = {
            "/": ROOT / "src/mobilecar/ui/static/index.html",
            "/g4a2": ROOT / "src/mobilecar/ui/static/g4a2.html",
            "/static/g4a2.js": ROOT / "src/mobilecar/ui/static/g4a2.js",
            "/static/style.css": ROOT / "src/mobilecar/ui/static/style.css",
        }
        if path == "/frame.png":
            if FIXTURE.broken_image:
                self.send_response(500)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/png")
            self.send_header("Content-Length", str(len(PNG)))
            self.end_headers()
            self.wfile.write(PNG)
            return
        file = files.get(path)
        if file is None or not file.is_file():
            self.send_response(404)
            self.end_headers()
            return
        data = file.read_bytes()
        content_type = "text/html" if file.suffix == ".html" else "text/css" if file.suffix == ".css" else "application/javascript"
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, *_args):
        return


def _json(route, value, status=200):
    route.fulfill(
        status=status,
        content_type="application/json",
        body=json.dumps(value),
    )


def _observation():
    State.sample += 1
    return {
        "run_id": "fixture_run",
        "episode_id": "fixture_episode",
        "backend_generation": 1,
        "sample_id": f"rgbd_fixture_{State.sample}",
        "valid_fraction": 0.75,
        "center_depth_m": 0.25,
        "rgb_uri": f"/frame.png?s={State.sample}",
        "arm_user_joint_positions_rad": [0, 0, 0, 0, 0],
        "gripper_width_m": 0.025,
        "observation_profile_sha256": "1" * 64,
        "action_profile_sha256": "2" * 64,
    }


def main() -> int:
    state = FIXTURE
    server = ThreadingHTTPServer(("127.0.0.1", 0), StaticHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    url = f"http://127.0.0.1:{server.server_address[1]}/g4a2"
    out = ROOT / "var/reports/g4a2-ui-state-smoke"
    out.mkdir(parents=True, exist_ok=True)
    result = {"synthetic_fixture": True, "human_evidence": False}
    try:
        with sync_playwright() as playwright:
            browser = playwright.chromium.launch(headless=True)
            page = browser.new_page(viewport={"width": 1280, "height": 900})
            browser_messages = []
            page.on("console", lambda message: browser_messages.append(f"console:{message.type}:{message.text}"))
            page.on("pageerror", lambda error: browser_messages.append(f"pageerror:{error}"))

            def route_api(route):
                path = urlparse(route.request.url).path
                if path == "/api/v1/health":
                    _json(route, {"status": "ok", "backend_generation": 1, "recording_status": state.mode})
                elif path == "/api/v1/g4a2/observation":
                    if state.mode == "RECORDING":
                        _json(route, _observation())
                    else:
                        _json(route, {"detail": "observation paused"}, 404)
                elif path == "/api/v1/episodes/start":
                    state.mode = "RECORDING"
                    _json(route, {"accepted": True, "episode_id": "fixture_episode", "recording_status": "RECORDING"})
                elif path == "/api/v1/episodes/stop":
                    time.sleep(0.35)
                    if state.fail_stop:
                        _json(route, {"accepted": False, "reason": "fixture stop failed"})
                    else:
                        state.mode = "COMPLETE"
                        _json(route, {"accepted": True, "episode_id": "fixture_episode", "recording_status": "COMPLETE"})
                elif path == "/api/v1/g4a2/session":
                    _json(route, {"accepted": True, "human_session_id": "fixture_human_session"})
                elif path == "/api/v1/g4a1/permission":
                    _json(route, {"accepted": True, "permission_id": "fixture_permission", "nearfield_epoch": 1})
                elif path == "/api/v1/g4a2/intent":
                    _json(route, {"accepted": True, "human_intent_id": "fixture_intent"})
                elif path == "/api/v1/g4a2/action":
                    state.last_action_sample = json.loads(route.request.post_data or "{}").get("observation_sample_id")
                    _json(route, {"accepted": True, "action_seq": 1, "control_tick_id": "fixture_tick"})
                else:
                    _json(route, {"detail": "fixture not found"}, 404)

            page.route("**/api/v1/**", route_api)
            page.goto(url, wait_until="domcontentloaded")
            page.wait_for_function("document.querySelector('#g4a2-status')?.textContent.includes('后端')", timeout=10000)
            page.locator("#human-record").click()
            page.wait_for_function("document.querySelector('#recording-lifecycle')?.textContent === '正在录制'", timeout=5000)
            try:
                page.wait_for_function("document.querySelector('#frame-lifecycle')?.textContent === '实时'", timeout=5000)
            except Exception as error:
                print(
                    json.dumps(
                        {
                            "frame": page.locator("#frame-lifecycle").inner_text(),
                            "image_status": page.locator("#human-image-status").inner_text(),
                            "recording": page.locator("#recording-lifecycle").inner_text(),
                            "browser_messages": browser_messages[-20:],
                        },
                        indent=2,
                    )
                )
                raise
            result["initial_live"] = True
            page.locator("#human-confirm").click()
            page.locator("#human-send").click()
            page.wait_for_function("document.querySelector('#g4a2-log')?.textContent.includes('action 1 dispatched')", timeout=5000)
            assert state.last_action_sample == page.locator("#human-sample").inner_text()
            result["action_uses_displayed_sample"] = True
            page.locator("#human-stop").click()
            page.wait_for_function("document.querySelector('#recording-lifecycle')?.textContent === '正在结束'", timeout=1000)
            result["finalizing_immediate"] = True
            page.wait_for_function("document.querySelector('#recording-lifecycle')?.textContent === '已结束'", timeout=5000)
            page.wait_for_function("document.querySelector('#human-stop-result')?.textContent.includes('记录已保存')", timeout=5000)
            result["normal_stop_saved"] = True

            state.mode = "IDLE"
            state.broken_image = False
            page.locator("#human-record").click()
            page.wait_for_function("document.querySelector('#frame-lifecycle')?.textContent === '实时'", timeout=5000)
            state.broken_image = True
            try:
                page.wait_for_function("document.querySelector('#frame-lifecycle')?.textContent === '画面过期'", timeout=5000)
            except Exception:
                print(
                    json.dumps(
                        {
                            "sample": page.locator("#human-sample").inner_text(),
                            "fixture_sample_count": state.sample,
                            "frame": page.locator("#frame-lifecycle").inner_text(),
                            "image_status": page.locator("#human-image-status").inner_text(),
                            "recording": page.locator("#recording-lifecycle").inner_text(),
                            "browser_messages": browser_messages[-20:],
                        },
                        indent=2,
                    )
                )
                raise
            result["broken_image_stale"] = True
            old_src = page.locator("#human-image").get_attribute("src")
            assert old_src and "frame.png" in old_src
            state.fail_stop = True
            page.locator("#human-stop").click()
            try:
                page.wait_for_function("document.querySelector('#recording-lifecycle')?.textContent === '结束失败'", timeout=5000)
            except Exception:
                print(
                    json.dumps(
                        {
                            "recording": page.locator("#recording-lifecycle").inner_text(),
                            "result": page.locator("#human-stop-result").inner_text(),
                            "frame": page.locator("#frame-lifecycle").inner_text(),
                            "browser_messages": browser_messages[-20:],
                        },
                        indent=2,
                    )
                )
                raise
            page.wait_for_function("document.querySelector('#human-stop-result')?.textContent.includes('结束失败')", timeout=5000)
            result["stop_failure_visible"] = True
            page.screenshot(path=out / "g4a2_state_failure.png", full_page=True)
            browser.close()
        result["passed"] = all(
            result.get(key) is True
            for key in ("initial_live", "action_uses_displayed_sample", "finalizing_immediate", "normal_stop_saved", "broken_image_stale", "stop_failure_visible")
        )
    finally:
        server.shutdown()
        thread.join(timeout=3)
    (out / "g4a2_state_smoke.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0 if result.get("passed") else 2


if __name__ == "__main__":
    raise SystemExit(main())
