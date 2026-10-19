#!/usr/bin/env python3
"""Server-only EGL and Chromium WebGL capability probe."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402
from OpenGL import GL  # noqa: E402
from playwright.sync_api import sync_playwright  # noqa: E402


WEBGL_QUERY = """
() => {
  const probeCanvas = document.createElement("canvas");
  probeCanvas.width = 64;
  probeCanvas.height = 64;
  const glContext =
    probeCanvas.getContext("webgl2") || probeCanvas.getContext("webgl");
  if (!glContext) {
    return { available: false };
  }
  const debugInfo = glContext.getExtension("WEBGL_debug_renderer_info");
  return {
    available: true,
    vendor: debugInfo
      ? glContext.getParameter(debugInfo.UNMASKED_VENDOR_WEBGL)
      : glContext.getParameter(glContext.VENDOR),
    renderer: debugInfo
      ? glContext.getParameter(debugInfo.UNMASKED_RENDERER_WEBGL)
      : glContext.getParameter(glContext.RENDERER),
    version: glContext.getParameter(glContext.VERSION),
  };
}
"""


def _gl_string(value):
    return value.decode("utf-8", errors="replace") if isinstance(value, bytes) else str(value)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    renderer = mujoco.Renderer(model, height=128, width=128)
    try:
        renderer.update_scene(data, camera="rgbd")
        image = renderer.render().copy()
        mujoco_vendor = _gl_string(GL.glGetString(GL.GL_VENDOR))
        mujoco_renderer = _gl_string(GL.glGetString(GL.GL_RENDERER))
    finally:
        renderer.close()

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
        page = browser.new_page()
        webgl = page.evaluate(WEBGL_QUERY)
        browser_version = browser.version
        browser.close()

    report = {
        "mujoco": {
            "version": mujoco.__version__,
            "shape": list(image.shape),
            "dtype": str(image.dtype),
            "min": int(image.min()),
            "max": int(image.max()),
            "mujoco_gl": os.environ.get("MUJOCO_GL"),
            "vendor": mujoco_vendor,
            "renderer": mujoco_renderer,
        },
        "chromium": {
            "browser_version": browser_version,
            "channel": "chromium",
            "webgl": webgl,
        },
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
