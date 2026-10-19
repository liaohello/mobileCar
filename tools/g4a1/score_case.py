#!/usr/bin/env python3
"""Build an external G4A-1 readiness report from a formal case report."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml

from mobilecar.data.g4a1_readiness import build_g4a1_readiness


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--formal-report", type=Path, required=True)
    parser.add_argument("--runs-root", type=Path, required=True)
    parser.add_argument("--quality-profile", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    formal = json.loads(args.formal_report.read_text(encoding="utf-8"))
    profile = yaml.safe_load(args.quality_profile.read_text(encoding="utf-8"))
    report = build_g4a1_readiness(
        formal, runs_root=args.runs_root, quality_profile=profile
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report["summary"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
