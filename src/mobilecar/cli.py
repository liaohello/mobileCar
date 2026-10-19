"""Command-line entry point for the G1 teleoperation service."""

from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="mobilecar",
        description="LanderPi G1 mobile manipulation teleoperation and raw Episode service",
    )
    parser.add_argument("--config", type=Path, default=None, help="YAML application profile")
    parser.add_argument(
        "--backend",
        choices=("mock", "mujoco"),
        default=None,
        help="override backend.kind from the profile",
    )
    parser.add_argument("--host", default=None, help="bind host override")
    parser.add_argument("--port", type=int, default=None, help="bind port override")
    parser.add_argument(
        "--require-recording",
        action="store_true",
        help="refuse motion until Episode recording start is durably acknowledged",
    )
    parser.add_argument(
        "--no-realtime",
        action="store_true",
        help="run MuJoCo physics as fast as possible for bounded tests",
    )
    parser.add_argument(
        "--print-bootstrap",
        action="store_true",
        help="create the application, print capabilities, and exit",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    # Imports remain lazy so `mobilecar --help` works in a diagnostic-only shell.
    from .app import Runtime, create_app
    from .config import AppConfig, load_config

    config = load_config(args.config)
    if args.print_bootstrap:
        data = copy.deepcopy(config.data)
        if args.backend:
            data["backend"]["kind"] = args.backend
        if args.require_recording:
            data["recording"]["required_for_motion"] = True
        if args.no_realtime:
            data["backend"]["realtime"] = False
        runtime = Runtime(
            AppConfig(config.path, data),
        )
        payload = {
            "config": str(config.path),
            "capabilities": runtime.capabilities.model_dump(mode="json"),
            "snapshot": runtime.backend.snapshot().model_dump(mode="json"),
        }
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        runtime.close()
        return 0

    app = create_app(
        args.config,
        backend_kind=args.backend,
        require_recording=True if args.require_recording else None,
        realtime=False if args.no_realtime else None,
    )
    import uvicorn

    host = args.host or config.ui["host"]
    port = args.port or int(config.ui["port"])
    uvicorn.run(app, host=host, port=port, log_level="info")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
