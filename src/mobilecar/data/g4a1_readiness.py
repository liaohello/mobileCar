"""G4A-1 development-data readiness assessment.

This is deliberately separate from the historical G2B export qualification:
it reports readiness for future collection, not LeRobot export or training.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any


def _read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8")) if path.is_file() else {}


def _read_events(episode_dir: Path) -> list[dict[str, Any]]:
    path = episode_dir / "events.jsonl"
    if not path.is_file():
        return []
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            rows.append(json.loads(line))
    return rows


def evaluate_g4a1_case(
    case_report: dict[str, Any],
    *,
    runs_root: Path,
    quality_profile: dict[str, Any],
) -> dict[str, Any]:
    """Return explicit PASS/FAIL/NOT_EVALUATED gates for one development case."""

    validation = case_report.get("validation", {})
    assessment = case_report.get("assessment", {})
    run_id = str(case_report.get("run_id", ""))
    episode_id = str(case_report.get("episode_id", ""))
    episode_dir = runs_root / run_id / "episodes" / episode_id
    events = _read_events(episode_dir)
    event_types = {str(row.get("event_type")) for row in events}
    action_events = {
        "requested",
        "approved",
        "dispatch",
        "g4a1_action_approved",
        "g4a1_actuator_reference",
    }
    source_values = {
        str((row.get("payload", {}).get("request") or {}).get("source"))
        for row in events
        if row.get("event_type") == "g4a1_action_approved"
    }
    source_values.discard("None")
    format_valid = validation.get("audit_status") == "COMPLETE" and bool(
        validation.get("passed")
    )
    data_complete = format_valid and all(
        int(validation.get("counts", {}).get(key, 0)) > 0
        for key in ("events", "commands", "observations")
    )
    if case_report.get("expected") == "success":
        task_pass = (
            assessment.get("task_result") == "success"
            and float(assessment.get("lift_height_m") or 0.0)
            >= float(quality_profile["physical_success"]["lift_threshold_m"])
            and float(assessment.get("hold_duration_s") or 0.0)
            >= float(quality_profile["physical_success"]["bilateral_hold_s"])
        )
    else:
        task_pass = bool(case_report.get("old_action_rejected")) and format_valid
    observability = (
        float(case_report.get("observation_valid_fraction") or 0.0) > 0.0
        and bool(case_report.get("observation_sample_id"))
    )
    action_causality = action_events <= event_types and bool(source_values)
    source_ok = all(
        source in {"agent_ui", "scripted", "test_injection", "synthetic_fixture"}
        for source in source_values
    )
    parent_split = "NOT_EVALUATED"
    training_qualified = False
    export_valid = "NOT_EVALUATED"
    gates = {
        "format_valid": format_valid,
        "data_complete": data_complete,
        "task_result": task_pass,
        "observability": observability,
        "action_causality": action_causality,
        "source_provenance": source_ok,
        "parent_split": parent_split,
        "training_qualified": training_qualified,
        "export_valid": export_valid,
    }
    reasons = []
    if not task_pass:
        reasons.append("physical task did not pass lift/bilateral-hold gate")
    if not action_causality:
        reasons.append("requested/approved/dispatch/actuator-reference chain incomplete")
    if not source_ok:
        reasons.append("unexpected operator source")
    reasons.append("dev-only G4A-1 data is not training-qualified")
    return {
        "interface_profile": "g4a1-1.0",
        "case_id": case_report.get("id"),
        "run_id": run_id,
        "episode_id": episode_id,
        "operator_source": sorted(source_values),
        "view_source": case_report.get("view_source"),
        "gates": gates,
        "reasons": reasons,
        "source_operator_kind": case_report.get("source"),
        "training_qualified": training_qualified,
        "export_valid": export_valid,
    }


def build_g4a1_readiness(
    formal_report: dict[str, Any],
    *,
    runs_root: str | Path,
    quality_profile: dict[str, Any],
) -> dict[str, Any]:
    root = Path(runs_root)
    cases = [
        evaluate_g4a1_case(
            item,
            runs_root=root,
            quality_profile=quality_profile,
        )
        for item in formal_report.get("cases", [])
    ]
    summary = {
        "case_count": len(cases),
        "format_valid": all(item["gates"]["format_valid"] for item in cases),
        "data_complete": all(item["gates"]["data_complete"] for item in cases),
        "task_evidence_complete": all(
            item["gates"]["task_result"] for item in cases
        ),
        "observability": all(item["gates"]["observability"] for item in cases),
        "action_causality": all(item["gates"]["action_causality"] for item in cases),
        "source_provenance": all(item["gates"]["source_provenance"] for item in cases),
        "parent_split": "NOT_EVALUATED",
        "training_qualified": False,
        "export_valid": "NOT_EVALUATED",
    }
    return {
        "report_version": "g4a1-readiness-1",
        "interface_profile": "g4a1-1.0",
        "execution_location": "server_only",
        "formal_report_success": bool(formal_report.get("success")),
        "summary": summary,
        "cases": cases,
    }
