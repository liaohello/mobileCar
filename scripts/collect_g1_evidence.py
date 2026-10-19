#!/usr/bin/env python3
"""Collect acceptance evidence for already-recorded G1 Episodes."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import httpx
from playwright.sync_api import sync_playwright


ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "docs/evidence"


def summary(detail: dict[str, Any]) -> dict[str, Any]:
    events = detail.get("events", [])
    requested = [
        event["payload"]["request"]
        for event in events
        if event.get("event_type") == "requested"
        and isinstance(event.get("payload", {}).get("request"), dict)
    ]
    reached = [
        {"event_seq": event.get("event_seq"), "plan_id": event.get("plan_id"), **event.get("payload", {})}
        for event in events
        if event.get("event_type") == "reached"
    ]
    rejected = [
        {"event_seq": event.get("event_seq"), **event.get("payload", {})}
        for event in events
        if event.get("event_type") in {"rejected", "failed", "cancelled", "contact_violation"}
    ]
    return {
        "episode_id": detail["episode_id"],
        "run_id": detail["run_id"],
        "episode": detail["episode"],
        "manifest": detail["manifest"],
        "validation": detail["validation"],
        "assessment": detail["assessment"],
        "eligibility": detail["eligibility"],
        "event_count": len(events),
        "observation_count": len(detail.get("observations", [])),
        "media": detail.get("media", {}),
        "requested_actions": requested,
        "reached": reached,
        "rejections_or_stops": rejected,
        "read_only": detail.get("read_only"),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="http://127.0.0.1:8787")
    parser.add_argument("--success", default="episode_4414a5ec8ae645d29d8e7df7826160cb")
    parser.add_argument("--failure", default="episode_4cb1700ce2e04a8f9c5276b623ddb43f")
    args = parser.parse_args()
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    with httpx.Client(base_url=args.url, timeout=180.0) as client:
        detail_response = client.get(f"/api/v1/episodes/{args.success}")
        detail_response.raise_for_status()
        detail = detail_response.json()
        validation = client.post(f"/api/v1/episodes/{args.success}/validate")
        validation.raise_for_status()
        assessment = client.get(f"/api/v1/episodes/{args.success}/assessment")
        assessment.raise_for_status()
        eligibility = client.get(f"/api/v1/episodes/{args.success}/eligibility")
        eligibility.raise_for_status()
        failure_detail = client.get(f"/api/v1/episodes/{args.failure}")
        failure_detail.raise_for_status()
        first_rgb = detail.get("latest_rgb_uri")
        if not first_rgb:
            raise RuntimeError("success Episode has no RGB asset")
        asset_response = client.get(f"/api/v1/episodes/{args.success}/assets/{first_rgb}")
        asset_response.raise_for_status()

    positive = summary(detail)
    positive["validation"] = validation.json()
    positive["assessment"] = assessment.json()
    positive["eligibility"] = eligibility.json()
    positive["raw_rgb_check"] = {
        "uri": first_rgb,
        "content_type": asset_response.headers.get("content-type"),
        "bytes": len(asset_response.content),
        "status": asset_response.status_code,
    }
    negative = summary(failure_detail.json())

    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(headless=True)
        page = browser.new_page(viewport={"width": 1600, "height": 1000})
        page.goto(args.url, wait_until="domcontentloaded")
        page.wait_for_function(
            "document.querySelector('#connection-text')?.textContent === '已连接'",
            timeout=15000,
        )
        page.locator("[data-testid='replay-open']").click()
        page.wait_for_selector(f"[data-episode-id='{args.success}']", timeout=15000)
        page.locator(f"[data-episode-id='{args.success}']").click()
        page.wait_for_function(
            "document.querySelector('#replay-report')?.textContent?.includes('COMPLETE')",
            timeout=20000,
        )
        page.screenshot(path=EVIDENCE / "g1_replay_success.png")
        page.locator("#load-assessment").click()
        page.wait_for_function(
            "document.querySelector('#replay-report')?.textContent?.includes('success')",
            timeout=20000,
        )
        page.screenshot(path=EVIDENCE / "g1_replay_oracle_success.png")
        browser.close()

    result = {
        "success_episode": positive,
        "failure_episode": negative,
        "screen_evidence": [
            "g1_initial_recording.png",
            "g1_tcp_above_cube.png",
            "g1_mujoco_lift_hold.png",
            "g1_replay_success.png",
            "g1_replay_oracle_success.png",
        ],
    }
    output = EVIDENCE / "g1_success_and_failure.json"
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(
        json.dumps(
            {
                "output": str(output),
                "success": {
                    "audit": positive["validation"]["audit_status"],
                    "task_result": positive["assessment"]["task_result"],
                    "lift_height_m": positive["assessment"]["lift_height_m"],
                    "hold_duration_s": positive["assessment"]["hold_duration_s"],
                },
                "failure": {
                    "audit": negative["validation"]["audit_status"],
                    "task_result": negative["assessment"]["task_result"],
                    "termination_reason": negative["episode"].get("termination_reason"),
                },
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
