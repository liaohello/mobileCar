#!/usr/bin/env python3
"""Server-only live acceptance for the running G4A-2 collection service.

Drives the real service through a real browser with real API calls and covers:

  1. normal stop saves: start -> live frame -> 正在结束 -> 已结束 + 记录已保存 + Episode ID
  2. stop failure stays visible: intercepted server rejection -> 结束失败 + reason -> retry -> saved
  3. recording can be restarted after a completed/failed stop
  4. deployed bundle identity: hashes of /static/g4a2.js and /static/app.js served live
  5. no human session is ever created: 确认本人操作 is never clicked and never required

Automation acceptance only. The produced Episodes lack the human_session_confirmed
event, so they cannot be discovered as human demonstrations (see
tools/g4a2/discover_h1.py). This script never sends arm/gripper targets, never
enables nearfield permission, and never claims human evidence. Existing Episodes
and runs are left untouched.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import time
import urllib.request
from pathlib import Path

from playwright.sync_api import sync_playwright


def _get(url: str, timeout: float = 10.0) -> tuple[int, bytes]:
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return response.status, response.read()


def _health(base_url: str) -> dict:
    _, body = _get(base_url.rstrip("/") + "/api/v1/health")
    return json.loads(body.decode("utf-8"))


def _asset_sha256(base_url: str, path: str) -> str:
    _, body = _get(base_url.rstrip("/") + path)
    return hashlib.sha256(body).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", default="http://127.0.0.1:8792")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--operator", default="acceptance_automation")
    parser.add_argument("--live-timeout", type=float, default=90.0)
    args = parser.parse_args()

    base = args.base_url.rstrip("/")
    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)
    report: dict = {
        "test": "g4a2_live_acceptance_v1",
        "execution_location": "server_only",
        "base_url": base,
        "operator_id": args.operator,
        "human_demonstration": False,
        "automation_acceptance": True,
        "started_wall_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "checks": {},
        "episodes": [],
        "screenshots": [],
    }

    def check(name: str, value: bool = True) -> None:
        report["checks"][name] = bool(value)

    def shot(page, name: str) -> None:
        path = out / f"{name}.png"
        page.screenshot(path=str(path), full_page=True)
        report["screenshots"].append(path.name)

    def wait_text(page, selector: str, expected: str, timeout: float) -> None:
        page.wait_for_function(
            """(args) => {
              const node = document.querySelector(args.selector);
              return node && node.textContent.trim() === args.expected;
            }""",
            arg={"selector": selector, "expected": expected},
            timeout=int(timeout * 1000),
        )

    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(headless=True)
        page = browser.new_page(viewport={"width": 1280, "height": 900})
        page_errors: list[str] = []
        page.on("pageerror", lambda error: page_errors.append(str(error)))
        try:
            page.goto(base + "/g4a2", wait_until="domcontentloaded", timeout=20000)
            page.wait_for_selector("#g4a2-status", timeout=15000)
            page.wait_for_function(
                "document.querySelector('#g4a2-status')?.textContent.includes('后端')",
                timeout=15000,
            )
            check("page_loaded_with_backend_status")

            report["deployed_bundle"] = {
                "g4a2_js_sha256": _asset_sha256(base, "/static/g4a2.js"),
                "app_js_sha256": _asset_sha256(base, "/static/app.js"),
                "g4a2_html_sha256": _asset_sha256(base, "/static/g4a2.html"),
            }
            page.fill("#human-operator", args.operator)

            # --- Scenario 1: normal stop saves -------------------------
            pre_record_health = _health(base)
            page.locator("#human-record").click()
            wait_text(page, "#recording-lifecycle", "正在录制", 15)
            report.setdefault("states", {})["s1_recording_chip"] = page.locator("#recording-lifecycle").inner_text()
            page.wait_for_function(
                "document.querySelector('#frame-lifecycle')?.textContent.trim() === '实时'",
                timeout=int(args.live_timeout * 1000),
            )
            shot(page, "s1_live")
            health_before = _health(base)
            check("s1_recording_started_new_episode", health_before.get("episode_id") != pre_record_health.get("episode_id"))
            page.locator("#human-stop").click()
            page.wait_for_function(
                "document.querySelector('#recording-lifecycle')?.textContent.trim() === '正在结束'",
                timeout=1500,
            )
            check("s1_finalizing_immediate")
            wait_text(page, "#recording-lifecycle", "已结束", 30)
            page.wait_for_function(
                "document.querySelector('#human-stop-result')?.textContent.includes('记录已保存')",
                timeout=15000,
            )
            stop_result = page.locator("#human-stop-result").inner_text()
            episode_id = None
            if "Episode " in stop_result:
                episode_id = stop_result.split("Episode ", 1)[1].strip()
            report["episodes"].append({"scenario": "s1_normal_stop", "episode_id": episode_id})
            health_after = _health(base)
            check(
                "s1_normal_stop_saved",
                bool(episode_id)
                and health_after.get("recording_status") == "COMPLETE"
                and health_after.get("episode_id") == episode_id
                and health_before.get("episode_id") == episode_id,
            )
            shot(page, "s1_saved")

            # --- Scenario 2: stop failure visible, then real stop ------
            page.locator("#human-record").click()
            wait_text(page, "#recording-lifecycle", "正在录制", 15)
            page.wait_for_function(
                "document.querySelector('#frame-lifecycle')?.textContent.trim() === '实时'",
                timeout=int(args.live_timeout * 1000),
            )

            def fail_stop(route) -> None:
                route.fulfill(
                    status=200,
                    content_type="application/json",
                    body=json.dumps({"accepted": False, "reason": "acceptance_simulated_stop_failure"}),
                )

            page.route("**/api/v1/episodes/stop", fail_stop)
            page.locator("#human-stop").click()
            wait_text(page, "#recording-lifecycle", "结束失败", 15)
            failure_text = page.locator("#human-stop-result").inner_text()
            check(
                "s2_stop_failure_visible",
                "结束失败" in failure_text and "acceptance_simulated_stop_failure" in failure_text,
            )
            shot(page, "s2_stop_failure")
            page.unroute("**/api/v1/episodes/stop")
            page.locator("#human-stop").click()
            wait_text(page, "#recording-lifecycle", "已结束", 30)
            page.wait_for_function(
                "document.querySelector('#human-stop-result')?.textContent.includes('记录已保存')",
                timeout=15000,
            )
            stop_result = page.locator("#human-stop-result").inner_text()
            episode_id = stop_result.split("Episode ", 1)[1].strip() if "Episode " in stop_result else None
            report["episodes"].append({"scenario": "s2_stop_failure_then_saved", "episode_id": episode_id})
            check("s2_retry_stop_saved", bool(episode_id))

            # --- Scenario 3: restart recording after stop --------------
            page.locator("#human-record").click()
            wait_text(page, "#recording-lifecycle", "正在录制", 15)
            page.wait_for_function(
                "document.querySelector('#frame-lifecycle')?.textContent.trim() === '实时'",
                timeout=int(args.live_timeout * 1000),
            )
            shot(page, "s3_restarted_live")
            page.locator("#human-stop").click()
            wait_text(page, "#recording-lifecycle", "已结束", 30)
            page.wait_for_function(
                "document.querySelector('#human-stop-result')?.textContent.includes('记录已保存')",
                timeout=15000,
            )
            stop_result = page.locator("#human-stop-result").inner_text()
            episode_id = stop_result.split("Episode ", 1)[1].strip() if "Episode " in stop_result else None
            report["episodes"].append({"scenario": "s3_restart_recording", "episode_id": episode_id})
            check("s3_restart_recording_saved", bool(episode_id))

            # --- Invariants --------------------------------------------
            session_text = page.locator("#human-session").inner_text()
            check("no_human_session_created", session_text == "未确认")
            check("no_page_errors", not page_errors)
            report["page_errors"] = page_errors
            report["final_health"] = _health(base)
            shot(page, "final")
        except Exception as error:  # noqa: BLE001 - report and fail honestly
            report["error"] = f"{type(error).__name__}: {error}"
            report["page_errors"] = page_errors
            try:
                shot(page, "failure")
            except Exception:  # noqa: BLE001 - best effort evidence
                pass
        finally:
            browser.close()

    required = (
        "page_loaded_with_backend_status",
        "s1_recording_started_new_episode",
        "s1_finalizing_immediate",
        "s1_normal_stop_saved",
        "s2_stop_failure_visible",
        "s2_retry_stop_saved",
        "s3_restart_recording_saved",
        "no_human_session_created",
        "no_page_errors",
    )
    report["passed"] = all(report["checks"].get(name) for name in required)
    report["finished_wall_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    (out / "g4a2_live_acceptance.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps({"passed": report["passed"], "checks": report["checks"]}, ensure_ascii=False))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
