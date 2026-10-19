#!/usr/bin/env python3
"""Server-only G2B export command."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from mobilecar.data.export.pipeline import run_g2b_export


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--report-root", type=Path, required=True)
    parser.add_argument(
        "--profile",
        type=Path,
        default=Path("configs/export/g2b_export_v1.yaml"),
    )
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--motion-episode-id", required=True)
    parser.add_argument("--stationary-episode-id", required=True)
    parser.add_argument("--stationary-run-id")
    parser.add_argument("--fault-episode-id", required=True)
    parser.add_argument("--fault-run-id", required=True)
    parser.add_argument(
        "--negative-fixture",
        type=Path,
        default=Path("tests/export/fixtures/causal_input_v1.json"),
    )
    args = parser.parse_args()
    report = run_g2b_export(
        run_root=args.run_root,
        dataset_root=args.dataset_root,
        report_root=args.report_root,
        profile_path=args.profile,
        run_id=args.run_id,
        motion_episode_id=args.motion_episode_id,
        stationary_episode_id=args.stationary_episode_id,
        stationary_run_id=args.stationary_run_id or args.run_id,
        fault_episode_id=args.fault_episode_id,
        fault_run_id=args.fault_run_id,
        negative_fixture=args.negative_fixture,
    )
    print(
        json.dumps(
            {
                "report": str(args.report_root / "g2b_report.json"),
                "passed": report.get("passed"),
                "format_valid": report.get("format_valid"),
                "export_valid": report.get("export_valid"),
                "train_qualified": report.get("train_qualified", False),
            },
            indent=2,
        )
    )
    return 0 if report.get("passed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
