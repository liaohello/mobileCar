#!/usr/bin/env python3
"""Server-only /g4a2 preview regression; every API call is intercepted.

Loads the application's real HTML/JS from --base-url. All observations, media,
human confirmations and command responses are synthetic browser fixtures. This
script never creates a real human session, records an Episode or drives a backend.
It intentionally does not install Playwright or start an application server.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import struct
import time
import traceback
import zlib
from collections import Counter
from pathlib import Path
from urllib.parse import urlsplit

from playwright.async_api import async_playwright


GOOD = "fixture_good_a"
SLOW = "fixture_slow_b"
FAILED = "fixture_failed_c"
RECOVERED = "fixture_recovered_d"


def png_bytes() -> bytes:
    """Create a small, valid 640x480 RGB PNG using only the standard library."""
    def chunk(kind: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + kind + data + struct.pack(
            ">I", zlib.crc32(kind + data) & 0xFFFFFFFF
        )

    pixels = (b"\x00" + bytes((38, 111, 160)) * 640) * 480
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", 640, 480, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(pixels))
        + chunk(b"IEND", b"")
    )


class Fixture:
    def __init__(self, base_url: str) -> None:
        parsed = urlsplit(base_url)
        self.origin = (parsed.scheme, parsed.netloc)
        self.sample = GOOD
        self.png = png_bytes()
        self.slow_started = asyncio.Event()
        self.slow_release = asyncio.Event()
        self.failed_started = asyncio.Event()
        self.active_images = 0
        self.max_active_images = 0
        self.image_requests: Counter[str] = Counter()
        self.posts: list[dict] = []
        self.unexpected: list[dict] = []
        self.confirmation_allowed = False
        self.intent_received = asyncio.Event()

    def observation(self) -> dict:
        return {
            "interface_profile": "g4a2-1.0",
            "run_id": "fixture_run", "episode_id": "fixture_episode",
            "backend_generation": 1, "sample_id": self.sample,
            "stream_id": "rgbd", "capture_control_tick_id": "fixture_tick",
            "capture_sim_time_s": 1.0, "capture_backend_monotonic_s": 10.0,
            "arrival_backend_monotonic_s": 10.01,
            "observation_backend_monotonic_s": 10.02,
            "rgb_uri": f"/api/v1/g4a2/rgbd/{self.sample}/rgb",
            "valid_fraction": 1.0, "center_depth_m": 0.2,
            "arm_user_joint_positions_rad": [0.1, 0.1, 0.1, 0.1, 0.1],
            "gripper_width_m": 0.025,
            "observation_profile_sha256": "a" * 64,
            "action_profile_sha256": "b" * 64,
        }

    async def route(self, route) -> None:
        request = route.request
        parsed = urlsplit(request.url)
        path = parsed.path
        if not path.startswith("/api/"):
            if request.method == "GET" and (parsed.scheme, parsed.netloc) == self.origin:
                await route.continue_()  # Only real application HTML/JS/CSS.
            else:
                self.unexpected.append({"method": request.method, "url": request.url})
                await route.abort("blockedbyclient")
            return

        # No API request is ever continued to the server, even on an error path.
        async def reply(body: dict, status: int = 200) -> None:
            await route.fulfill(status=status, json=body)

        if request.method == "GET" and path == "/api/v1/health":
            await reply({"backend_generation": 1, "recording_status": "RECORDING"})
        elif request.method == "GET" and path == "/api/v1/capabilities":
            # Read-only posture registry lookup from the page boot.
            await reply(
                {
                    "model_sha256": "c" * 64,
                    "g4a2_postures": [
                        {
                            "name": "nearfield_observe",
                            "purpose": "nearfield_observation",
                            "frozen": True,
                            "arm_user_joint_positions_rad": [0.0, 0.0, 0.0, 0.0, 0.0],
                            "gripper_width_m": 0.03,
                        }
                    ],
                }
            )
        elif request.method == "GET" and path == "/api/v1/g4a2/observation":
            await reply(self.observation())
        elif request.method == "GET" and path.startswith("/api/v1/g4a2/rgbd/"):
            sample = path.split("/")[-2]
            self.image_requests[sample] += 1
            self.active_images += 1
            self.max_active_images = max(self.max_active_images, self.active_images)
            try:
                if sample == SLOW:
                    self.slow_started.set()
                    await asyncio.wait_for(self.slow_release.wait(), timeout=3.0)
                if sample == FAILED:
                    self.failed_started.set()
                    await reply({"detail": "synthetic image failure"}, status=503)
                else:
                    await route.fulfill(
                        status=200, content_type="image/png", body=self.png,
                        headers={"Cache-Control": "no-store"},
                    )
            finally:
                self.active_images -= 1
        elif request.method == "POST":
            payload = request.post_data_json or {}
            entry = {"path": path, "payload": payload, "fixture_only": True}
            self.posts.append(entry)
            if path == "/api/v1/g4a2/session":
                if not self.confirmation_allowed:
                    self.unexpected.append({"reason": "automatic_human_confirmation", **entry})
                await reply({"accepted": True, "human_session_id": "synthetic_human_session"})
            elif path == "/api/v1/g4a1/permission":
                await reply({"accepted": True, "permission_id": "fixture_permission", "nearfield_epoch": 1})
            elif path == "/api/v1/g4a2/intent":
                await reply({
                    "accepted": True, "human_intent_id": "fixture_intent",
                    "display_sample_id": payload.get("display_sample_id"),
                })
                self.intent_received.set()
            elif path == "/api/v1/g4a2/action":
                await reply({"accepted": True, "action_seq": payload.get("action_seq"), "control_tick_id": "fixture_tick"})
            elif path == "/api/v1/g4a1/revoke":
                await reply({"accepted": True})
            else:
                self.unexpected.append({"reason": "unexpected_api_mutation", **entry})
                await reply({"detail": "all real API mutations are blocked by fixture"}, status=409)
        else:
            self.unexpected.append({"method": request.method, "path": path})
            await reply({"detail": "unknown API blocked by fixture"}, status=404)


# Observe Image load events without replacing the application's display logic.
# A Proxy adds only a load listener to detached Image objects, whose load event
# otherwise cannot bubble to document. Times use the browser's own clock.
IMAGE_AUDIT_JS = """() => {
  window.__previewFixtureAudit = {loads: []};
  const note = (image) => window.__previewFixtureAudit.loads.push({
    url: image.currentSrc || image.src,
    load_ns: Math.round(performance.now() * 1e6),
    natural_width: image.naturalWidth, natural_height: image.naturalHeight
  });
  const NativeImage = window.Image;
  window.Image = new Proxy(NativeImage, {construct(target, args) {
    const image = Reflect.construct(target, args);
    image.addEventListener('load', () => note(image));
    return image;
  }});
  document.addEventListener('load', (event) => {
    if (event.target instanceof HTMLImageElement) note(event.target);
  }, true);
}"""


async def visual_state(page) -> dict:
    return await page.evaluate("""() => {
      const image = document.getElementById('human-image');
      const box = image.getBoundingClientRect();
      return {
        sample: image.dataset.sampleId || document.getElementById('human-sample').textContent,
        src: image.currentSrc || image.src,
        complete: image.complete, natural_width: image.naturalWidth,
        box: {x: box.x, y: box.y, width: box.width, height: box.height},
        image_status: document.getElementById('human-image-status')?.textContent || '',
        browser_ns: Math.round(performance.now() * 1e6)
      };
    }""")


async def displayed(page, sample: str) -> dict:
    await page.wait_for_function("""sample => {
      const image = document.getElementById('human-image');
      const id = image?.dataset.sampleId || document.getElementById('human-sample')?.textContent;
      return id === sample && image.complete && image.naturalWidth > 0;
    }""", arg=sample, timeout=7000)
    return await visual_state(page)


def same_layout(first: dict, second: dict) -> bool:
    return all(abs(first["box"][key] - second["box"][key]) <= 1.0
               for key in ("x", "y", "width", "height"))


async def check(args, report: dict) -> None:
    fixture = Fixture(args.base_url)
    report["fixture"] = {"source": "synthetic_fixture", "all_api_intercepted": True}
    async with async_playwright() as playwright:
        browser = await playwright.chromium.launch(
            headless=True, args=["--no-sandbox", "--disable-dev-shm-usage"]
        )
        context = await browser.new_context(
            viewport={"width": 1280, "height": 1100}, service_workers="block"
        )
        await context.route("**/*", fixture.route)
        page = await context.new_page()
        page.set_default_timeout(5000)
        await page.add_init_script(f"({IMAGE_AUDIT_JS})();")
        page.on("pageerror", lambda error: report["page_errors"].append(str(error)))
        try:
            await page.goto(args.base_url.rstrip("/") + "/g4a2", wait_until="domcontentloaded", timeout=15000)
            first = await displayed(page, GOOD)
            report["states"]["initial"] = first
            assert first["box"]["width"] > 0 and first["box"]["height"] > 0

            # Trigger handlers as a negative fixture, even if controls are disabled.
            # They must not create/confirm a human session implicitly.
            await page.locator("#human-send").dispatch_event("click")
            await page.locator("#human-stream").dispatch_event("click")
            await asyncio.sleep(0.35)
            assert not fixture.posts, "unconfirmed controls made an API mutation"
            report["checks"]["no_automatic_human_confirmation"] = True

            fixture.sample = SLOW
            await asyncio.wait_for(fixture.slow_started.wait(), timeout=5.0)
            await asyncio.sleep(0.85)  # Longer than two old 300 ms refresh periods.
            during = await visual_state(page)
            report["states"]["slow_pending"] = during
            assert during["sample"] == GOOD and during["natural_width"] > 0
            assert same_layout(first, during), "slow image changed the visible image layout"
            assert fixture.image_requests[SLOW] == 1, "slow sample was requested more than once concurrently"
            assert fixture.max_active_images == 1, "overlapping image requests detected"
            fixture.slow_release.set()
            second = await displayed(page, SLOW)
            assert same_layout(first, second), "loaded image changed the image layout"
            report["checks"]["slow_image_single_flight_and_stable_layout"] = True

            fixture.sample = FAILED
            await asyncio.wait_for(fixture.failed_started.wait(), timeout=5.0)
            await asyncio.sleep(0.65)
            failed = await visual_state(page)
            report["states"]["failed_candidate"] = failed
            assert failed["sample"] == SLOW, "failed sample became display_sample_id"
            assert failed["natural_width"] > 0 and failed["src"] == second["src"]
            assert same_layout(second, failed), "failed image collapsed or moved the layout"
            assert not fixture.posts, "image failure triggered an API mutation"
            report["checks"]["failed_image_retains_last_good_frame"] = True

            fixture.sample = RECOVERED
            recovered = await displayed(page, RECOVERED)
            report["states"]["recovered"] = recovered
            fixture.confirmation_allowed = True
            await page.locator("#human-confirm").click()
            await page.wait_for_function("document.getElementById('human-session').textContent === 'synthetic_human_session'")
            await page.locator("#human-authorize").click()
            await page.locator("#human-send").click()
            await asyncio.wait_for(fixture.intent_received.wait(), timeout=5.0)
            payload = next(item["payload"] for item in fixture.posts if item["path"] == "/api/v1/g4a2/intent")
            loads = await page.evaluate("window.__previewFixtureAudit.loads")
            matches = [item for item in loads if RECOVERED in item["url"]]
            assert matches, "successful browser load event was not observed"
            assert payload["display_sample_id"] == RECOVERED
            display_ns = payload["client_display_monotonic_ns"]
            intent_ns = payload["client_intent_monotonic_ns"]
            assert 0 < min(item["load_ns"] for item in matches) <= display_ns <= intent_ns
            assert all(item["natural_width"] > 0 for item in matches)
            assert len([item for item in fixture.posts if item["path"] == "/api/v1/g4a2/session"]) == 1
            report["intent_trace"] = {"payload": payload, "browser_loads": matches, "fixture_only": True}
            report["checks"]["loaded_display_sample_and_time_precede_intent"] = True
            assert not fixture.unexpected, f"unexpected fixture requests: {fixture.unexpected}"
            assert not report["page_errors"], f"page errors: {report['page_errors']}"
            report["passed"] = True
        finally:
            fixture.slow_release.set()
            report["fixture"].update({
                "image_requests": dict(fixture.image_requests),
                "max_active_image_requests": fixture.max_active_images,
                "intercepted_posts": fixture.posts, "unexpected_requests": fixture.unexpected,
                "real_backend_mutations": 0,
            })
            await context.close()
            await browser.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", required=True, help="Server static preview origin, for example http://127.0.0.1:8794")
    parser.add_argument("--output", type=Path, required=True, help="Server JSON report path")
    args = parser.parse_args()
    origin = urlsplit(args.base_url)
    if origin.scheme not in {"http", "https"} or not origin.netloc or origin.username or origin.password:
        parser.error("--base-url must be an HTTP(S) preview URL without credentials")
    report = {
        "test": "g4a2_preview_fixture_browser_v1", "execution_location": "server_only",
        "base_url": args.base_url, "passed": False, "checks": {}, "states": {}, "page_errors": [],
        "human_demonstration": False,
    }
    started = time.monotonic()
    try:
        asyncio.run(asyncio.wait_for(check(args, report), timeout=55.0))
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        report["traceback"] = traceback.format_exc()
    report["elapsed_wall_s"] = round(time.monotonic() - started, 3)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"passed": report["passed"], "checks": report["checks"], "output": str(args.output)}, ensure_ascii=False))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
