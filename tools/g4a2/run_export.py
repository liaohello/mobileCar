#!/usr/bin/env python3
"""Read a G4A-2 cohort manifest and write a locked RGB-D dataset externally."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from mobilecar.data.g4a2_export import write_g4a2_cohort
from mobilecar.data.g4a2_qualification import audit_human_episode, validate_collection_manifest


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--runs-root", type=Path, required=True)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--repo-id", default="mobilecar/g4a2-human-rgbd-dev")
    parser.add_argument("--main-cohort", action="store_true")
    args = parser.parse_args()
    document = json.loads(args.manifest.read_text(encoding="utf-8"))
    entries = document.get("entries", document if isinstance(document, list) else [])
    split_audit = validate_collection_manifest(entries, main_cohort=args.main_cohort)
    audits = [
        audit_human_episode(
            args.runs_root,
            entry["run_id"],
            entry["episode_id"],
            require_human=True,
        )
        for entry in entries
    ]
    source_ok = all(item["passed"] for item in audits)
    if not split_audit["passed"] or not source_ok:
        report = {
            "status": "rejected",
            "training_qualified": False,
            "format_valid": False,
            "export_valid": False,
            "split_audit": split_audit,
            "episode_audits": audits,
        }
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2)[:4000])
        return 2
    labels = [str(entry["split"]) for entry in entries]
    written = write_g4a2_cohort(
        entries,
        run_root=args.runs_root,
        dataset_root=args.dataset_root,
        repo_id=args.repo_id,
        split_labels=labels if args.main_cohort else None,
    )
    report = {
        "status": "server_only",
        "format_valid": bool(written.get("format_valid", True)),
        "export_valid": bool(written.get("export_valid")),
        "training_qualified": False,
        "split_audit": split_audit,
        "episode_audits": audits,
        "export": written,
        "declared_limitations": [
            "RGB-only qualification is NOT_EVALUATED",
            "ACT compatibility and policy effect are NOT_EVALUATED",
            "no training or Hub upload was performed",
        ],
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: report[key] for key in ("format_valid", "export_valid", "training_qualified")}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
