#!/usr/bin/env python3
"""Audit exactly three human dev pilot Episodes without modifying source."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from mobilecar.data.export.reader import EpisodeReader
from mobilecar.data.g4a2_export import extract_g4a2_rows
from mobilecar.data.g4a2_qualification import audit_human_episode


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--runs-root", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    entries = manifest.get("entries", manifest if isinstance(manifest, list) else [])
    if len(entries) != 3:
        raise SystemExit(f"H1 requires exactly three entries, got {len(entries)}")
    results = []
    for entry in entries:
        reader = EpisodeReader(args.runs_root, entry["run_id"], entry["episode_id"])
        rows, rejected = extract_g4a2_rows(reader)
        audit = audit_human_episode(
            args.runs_root, entry["run_id"], entry["episode_id"], require_human=True
        )
        result = {
            "run_id": entry["run_id"],
            "episode_id": entry["episode_id"],
            "expected": entry.get("expected", "unknown"),
            "operator_kind": reader.episode.get("operator_kind"),
            "view_source": reader.episode.get("view_source"),
            "audit_status": reader.audit_status,
            "task_result": reader.task_result,
            "human_rows": len(rows),
            "rejected_rows": rejected[:10],
            "source_audit": audit,
            "source_tree_sha256": reader.source_file_hashes(),
            "passed": (
                audit["passed"]
                and reader.audit_status == "COMPLETE"
                and bool(rows)
                and (
                    (entry.get("expected") == "success" and reader.task_result == "success")
                    or (entry.get("expected") in {"failure", "cancelled"} and reader.task_result in {"failure", "cancelled"})
                )
            ),
        }
        results.append(result)
    success_count = sum(1 for item in results if item["task_result"] == "success")
    cancelled_count = sum(1 for item in results if item["task_result"] in {"failure", "cancelled"})
    report = {
        "report_version": "g4a2-h1-audit-1",
        "execution_location": "server_only",
        "purpose": "dev_pilot",
        "training_qualified": False,
        "entries": results,
        "success_count": success_count,
        "failure_or_cancelled_count": cancelled_count,
        "passed": success_count == 2 and cancelled_count == 1 and all(item["passed"] for item in results),
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: report[key] for key in ("passed", "success_count", "failure_or_cancelled_count")}, indent=2))
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
