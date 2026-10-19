"""Dataset eligibility kept separate from audit and physical task result."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from ..contracts import AuditStatus, DataEligibility, TaskResult
from .recorder import _atomic_json
from .validator import validate_episode


def _read_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    value = json.loads(path.read_text(encoding="utf-8"))
    return value if isinstance(value, dict) else {}


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path.is_file():
        return rows
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            text = line.strip()
            if not text:
                continue
            value = json.loads(text)
            if isinstance(value, dict):
                rows.append(value)
    return rows


def evaluate_eligibility(
    root: str | Path,
    episode_id: str,
    *,
    run_id: str | None = None,
    write: bool = True,
) -> DataEligibility:
    root_path = Path(root).resolve()
    if run_id:
        candidates = [root_path / run_id / "episodes" / episode_id]
    else:
        candidates = list(root_path.glob(f"*/episodes/{episode_id}"))
    matches = [path for path in candidates if path.is_dir()]
    if len(matches) != 1:
        raise FileNotFoundError(episode_id)
    episode_dir = matches[0]
    episode = _read_json(episode_dir / "episode.json")
    quality = _read_json(episode_dir / "quality_report.json")
    if not quality.get("audit_status"):
        quality = validate_episode(root_path, episode_id, run_id=run_id).model_dump(mode="json")
    assessment = _read_json(episode_dir / "assessments/task_assessment.json")
    events = _read_jsonl(episode_dir / "events.jsonl")
    observations = _read_jsonl(episode_dir / "observations/index.jsonl")
    kinds = {str(item.get("sensor_kind")) for item in observations}
    phases = {str(event.get("task_phase")) for event in events if event.get("task_phase")}
    action_rows = [
        event
        for event in events
        if event.get("event_type") in {"actuator_reference", "approved", "dispatch", "dispatched"}
    ]
    audit = AuditStatus(quality.get("audit_status", AuditStatus.PENDING.value))
    try:
        task_result = TaskResult(assessment.get("task_result", episode.get("result", "unknown")))
    except ValueError:
        task_result = TaskResult.UNKNOWN

    reasons: list[str] = []
    if audit != AuditStatus.COMPLETE:
        reasons.append("Episode audit is not COMPLETE")
    if not action_rows:
        reasons.append("no approved/dispatch/actuator_reference action records")
    d0 = audit == AuditStatus.COMPLETE and any(
        "base" in json.dumps(event.get("payload", {})).lower() for event in action_rows
    )
    d1 = bool(
        audit == AuditStatus.COMPLETE
        and any(event.get("event_type") in {"subgoal", "decision"} for event in events)
    )
    d2 = bool(
        audit == AuditStatus.COMPLETE
        and action_rows
        and "rgb" in kinds
        and bool({"joint_feedback", "encoders"} & kinds)
    )
    d3 = bool(
        audit == AuditStatus.COMPLETE
        and action_rows
        and len(phases) >= 2
        and task_result
        not in {TaskResult.UNKNOWN, TaskResult.NOT_APPLICABLE}
    )
    wrist_only = bool(
        episode.get("view_source") == "wrist_rgb_only"
        and "rgb" in kinds
        and not ({"depth", "lidar", "imu"} & kinds)
    )
    if not d2:
        reasons.append("D2 requires complete RGB, proprioception and executable action records")
    if episode.get("operator_kind") == "agent_ui":
        reasons.append("agent_ui is development evidence; not a human_teleop source")
    if task_result == TaskResult.UNKNOWN:
        reasons.append("physical task result is UNKNOWN")

    result = DataEligibility(
        episode_id=episode_id,
        audit_status=audit,
        task_result=task_result,
        d0_navigation=d0,
        d1_language_navigation=d1,
        d2_near_field_manipulation=d2,
        d3_combined_task=d3,
        reasons=reasons,
        source_operator_kind=episode.get("operator_kind"),
        view_source=episode.get("view_source"),
        wrist_rgb_only_eligible=wrist_only,
    )
    if write:
        _atomic_json(
            episode_dir / "assessments/eligibility.json",
            result.model_dump(mode="json"),
        )
    return result
