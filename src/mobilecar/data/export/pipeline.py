"""End-to-end G2B offline export over immutable G2A Episodes."""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .aligner import align_episode
from .lerobot_adapter import environment_identity, write_profile_dataset
from .models import load_export_profile, sha256_json
from .qualification import assess_export_valid, assess_train_qualified
from .reader import EpisodeReader, SourceIntegrityError, read_jsonl


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _atomic_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _read_negative_fixture(path: Path) -> dict[str, Any]:
    fixture = json.loads(path.read_text(encoding="utf-8"))
    cases = fixture.get("cases") or []
    required = {
        "normal_causal",
        "future_arrival",
        "clock_mismatch",
        "cross_episode",
        "missing_media",
        "long_gap",
        "unknown_action",
        "preview_rejected",
        "world_sim_leak",
        "parent_split_leak",
    }
    present = {str(item.get("case_id")) for item in cases}
    accepted = [
        str(item.get("case_id"))
        for item in cases
        if item.get("expected") == "accept"
    ]
    rejected = [
        str(item.get("case_id"))
        for item in cases
        if item.get("expected") == "reject"
    ]
    return {
        "fixture": str(path),
        "all_required_cases": required.issubset(present),
        "accepted_cases": accepted,
        "rejected_cases": rejected,
        "passed": required.issubset(present)
        and accepted == ["normal_causal"]
        and len(rejected) >= len(required) - 1,
    }


def _episode_summary(
    *,
    label: str,
    run_root: Path,
    run_id: str,
    episode_id: str,
    profile_path: Path,
    negative_fixture: Path,
    dataset_root: Path,
    write: bool,
) -> dict[str, Any]:
    result: dict[str, Any] = {
        "label": label,
        "run_id": run_id,
        "episode_id": episode_id,
        "started_wall_utc": _utc_now(),
    }
    profile = load_export_profile(profile_path)
    try:
        reader = EpisodeReader(run_root, run_id, episode_id)
    except SourceIntegrityError as exc:
        result.update({"error": str(exc), "format_valid": False, "export_valid": False})
        return result
    source_before = reader.source_file_hashes()
    source_tree_before = sha256_json(sorted(source_before.items()))
    alignment = align_episode(reader, profile, include_init_and_final=True)
    result["source"] = {
        **reader.identity(),
        "source_file_count": len(source_before),
        "source_tree_sha256": source_tree_before,
    }
    result["alignment"] = dict(alignment.summary)
    result["negative_cases"] = _read_negative_fixture(negative_fixture)
    missing_media = reader.check_required_media(
        row.image for row in alignment.rows
    )
    result["missing_media"] = missing_media
    if missing_media:
        result["format_valid"] = False
        result["export_valid"] = False
        result["error"] = "missing or empty source media"
        return result
    profile_results: dict[str, Any] = {}
    sidecars: dict[str, list[dict[str, Any]]] = {}

    for name in ("d0", "d2"):
        rows = alignment.rows_for(name)
        if not rows:
            profile_results[name] = {
                "format_valid": False,
                "export_valid": False,
                "write_attempted": False,
                "reason": "no_exportable_rows",
            }
            sidecars[name] = []
            continue
        if not write:
            profile_results[name] = {
                "format_valid": False,
                "export_valid": False,
                "write_attempted": False,
                "reason": "analysis_only",
                "rows": len(rows),
            }
            sidecars[name] = []
            continue
        written = write_profile_dataset(
            output_root=dataset_root,
            profile_name=name,
            profile=profile,
            reader=reader,
            alignment=alignment,
            repo_id=str(profile.raw["outputs"][f"{name}_repo_id"]),
            source_file_hashes_before=source_before,
        )
        sidecar_path = Path(written["sidecar"]["sidecar_root"]) / "sidecar.jsonl"
        sidecar_rows = read_jsonl(sidecar_path)
        sidecars[name] = sidecar_rows
        qualification = assess_export_valid(
            profile_name=name,
            profile=profile,
            alignment=alignment,
        )
        profile_results[name] = {
            **written,
            "write_attempted": True,
            "format_valid": bool(written.get("format_valid")),
            "export_valid": bool(qualification["passed"]),
            "qualification": qualification,
            "trace_samples": [
                sidecar_rows[index]
                for index in sorted(
                    {0, len(sidecar_rows) // 2, len(sidecar_rows) - 1}
                )
                if 0 <= index < len(sidecar_rows)
            ],
        }

    source_after = reader.source_file_hashes()
    source_tree_after = sha256_json(sorted(source_after.items()))
    source_unchanged = source_after == source_before
    result["source"]["source_tree_sha256_after"] = source_tree_after
    result["source"]["source_files_unchanged"] = source_unchanged
    result["profiles"] = profile_results
    result["format_valid"] = bool(
        source_unchanged
        and all(
            item.get("format_valid")
            for item in profile_results.values()
            if item.get("write_attempted")
        )
        and any(item.get("write_attempted") for item in profile_results.values())
    )
    result["export_valid"] = bool(
        source_unchanged
        and all(
            item.get("export_valid")
            for item in profile_results.values()
            if item.get("write_attempted")
        )
        and any(item.get("write_attempted") for item in profile_results.values())
    )
    result["train_qualified"] = {}
    for name in ("d0", "d2"):
        item = profile_results.get(name, {})
        result["train_qualified"][name] = assess_train_qualified(
            profile_name=name,
            profile=profile,
            operator_kind=reader.episode.get("operator_kind"),
            task_result=reader.task_result,
            view_source=reader.episode.get("view_source"),
            parent_episode_count=1,
            export_result={"passed": bool(item.get("export_valid"))},
        )
    result["train_qualified_all"] = all(
        item["qualified"] for item in result["train_qualified"].values()
    )
    result["finished_wall_utc"] = _utc_now()
    return result


def run_g2b_export(
    *,
    run_root: str | Path,
    dataset_root: str | Path,
    report_root: str | Path,
    profile_path: str | Path,
    run_id: str,
    motion_episode_id: str,
    stationary_episode_id: str,
    stationary_run_id: str,
    fault_episode_id: str,
    fault_run_id: str,
    negative_fixture: str | Path,
) -> dict[str, Any]:
    root = Path(run_root).resolve()
    dataset = Path(dataset_root).resolve()
    reports = Path(report_root).resolve()
    profile = Path(profile_path).resolve()
    fixture = Path(negative_fixture).resolve()
    report: dict[str, Any] = {
        "schema_version": 1,
        "report_version": "g2b-export-report-1.0",
        "generated_wall_utc": _utc_now(),
        "environment": environment_identity(),
        "profile": {
            "path": str(profile),
            "sha256": load_export_profile(profile).sha256,
        },
        "inputs": {
            "run_id": run_id,
            "motion_episode_id": motion_episode_id,
            "stationary_episode_id": stationary_episode_id,
            "stationary_run_id": stationary_run_id,
            "fault_episode_id": fault_episode_id,
            "fault_run_id": fault_run_id,
        },
    }
    try:
        report["motion"] = _episode_summary(
            label="motion",
            run_root=root,
            run_id=run_id,
            episode_id=motion_episode_id,
            profile_path=profile,
            negative_fixture=fixture,
            dataset_root=dataset,
            write=True,
        )
        report["stationary"] = _episode_summary(
            label="stationary",
            run_root=root,
            run_id=stationary_run_id,
            episode_id=stationary_episode_id,
            profile_path=profile,
            negative_fixture=fixture,
            dataset_root=dataset / "_rejected_stationary",
            write=False,
        )
        report["fault"] = _episode_summary(
            label="fault",
            run_root=root,
            run_id=fault_run_id,
            episode_id=fault_episode_id,
            profile_path=profile,
            negative_fixture=fixture,
            dataset_root=dataset / "_rejected_fault",
            write=False,
        )
        motion = report["motion"]
        stationary = report["stationary"]
        fault = report["fault"]
        report["gates"] = {
            "d0_nonempty": motion["profiles"].get("d0", {}).get("write_attempted", False),
            "d2_nonempty": motion["profiles"].get("d2", {}).get("write_attempted", False),
            "d0_export_valid": motion["profiles"]
            .get("d0", {})
            .get("export_valid", False),
            "d2_export_valid": motion["profiles"]
            .get("d2", {})
            .get("export_valid", False),
            "source_unchanged": motion["source"]["source_files_unchanged"],
            "negative_cases": motion["negative_cases"]["passed"],
            "stationary_rejected": not stationary.get("format_valid", False)
            and not stationary.get("profiles", {}).get("d0", {}).get("write_attempted", False)
            and not stationary.get("profiles", {}).get("d2", {}).get("write_attempted", False),
            "fault_rejected": (
                fault.get("source", {}).get("audit_status") != "COMPLETE"
                and not fault.get("format_valid", False)
            ),
        }
        report["format_valid"] = bool(motion.get("format_valid"))
        report["export_valid"] = bool(
            motion.get("export_valid")
            and report["gates"]["source_unchanged"]
            and report["gates"]["negative_cases"]
            and report["gates"]["stationary_rejected"]
            and report["gates"]["fault_rejected"]
        )
        report["train_qualified"] = bool(
            motion.get("train_qualified_all", False)
        )
        report["train_qualification"] = motion.get("train_qualified", {})
        report["passed"] = bool(report["format_valid"] and report["export_valid"])
    except Exception as exc:
        report["passed"] = False
        report["error"] = f"{type(exc).__name__}: {exc}"
    report["finished_wall_utc"] = _utc_now()
    _atomic_json(reports / "g2b_report.json", report)
    return report
