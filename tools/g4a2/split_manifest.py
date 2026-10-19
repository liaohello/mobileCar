#!/usr/bin/env python3
"""Validate and print a G4A-2 collection split manifest without mutating sources."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from mobilecar.data.g4a2_qualification import validate_collection_manifest


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--main-cohort", action="store_true")
    args = parser.parse_args()
    document = json.loads(args.manifest.read_text(encoding="utf-8"))
    entries = document.get("entries", document if isinstance(document, list) else [])
    result = validate_collection_manifest(entries, main_cohort=args.main_cohort)
    print(json.dumps(result, indent=2))
    return 0 if result["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
