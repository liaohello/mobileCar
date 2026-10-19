"""G4A-2 human cohort provenance, causality and split qualification."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any


def _rows(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    result = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            value = json.loads(line)
            if isinstance(value, dict):
                result.append(value)
    return result


def _finite_vector(value: Any, size: int) -> bool:
    return (
        isinstance(value, list)
        and len(value) == size
        and all(isinstance(item, (int, float)) and math.isfinite(float(item)) for item in value)
    )


def audit_human_episode(
    run_root: str | Path,
    run_id: str,
    episode_id: str,
    *,
    require_human: bool = True,
) -> dict[str, Any]:
    episode_dir = Path(run_root) / run_id / "episodes" / episode_id
    events = _rows(episode_dir / "events.jsonl")
    observations = _rows(episode_dir / "observations/index.jsonl")
    human_events = [row for row in events if row.get("event_type") == "human_intent"]
    linked = [row for row in events if row.get("event_type") == "g4a2_action_linked"]
    approved = [
        row
        for row in events
        if row.get("event_type") == "g4a1_action_approved"
        and (row.get("payload", {}).get("request", {}).get("source") == "human_teleop")
    ]
    normal_path = True
    targets_valid = True
    for row in approved:
        request = row.get("payload", {}).get("request", {})
        if request.get("teacher_plan_id") is not None:
            normal_path = False
        if not _finite_vector(request.get("arm_user_joint_positions_rad"), 5):
            targets_valid = False
        if not isinstance(request.get("gripper_width_m"), (int, float)):
            targets_valid = False
    rgb = [row for row in observations if row.get("sensor_kind") == "rgb"]
    depth = [row for row in observations if row.get("sensor_kind") == "depth"]
    rgb_ids = {str(row.get("sample_id")) for row in rgb}
    depth_by_id = {str(row.get("sample_id")): row for row in depth}
    paired = bool(rgb_ids) and rgb_ids == set(depth_by_id)
    display_ids = {
        str(row.get("payload", {}).get("display_sample_id"))
        for row in linked
        if row.get("payload", {}).get("display_sample_id")
    }
    gates = {
        "human_source": not require_human or bool(approved),
        "human_intent_present": not require_human or bool(human_events),
        "human_action_link": not require_human or bool(linked),
        "ordinary_sixdim_path": normal_path,
        "sixdim_target_valid": targets_valid,
        "rgb_depth_paired": paired,
        "display_sample_linked": not require_human or bool(display_ids),
    }
    return {
        "run_id": run_id,
        "episode_id": episode_id,
        "event_count": len(events),
        "human_intent_count": len(human_events),
        "human_action_link_count": len(linked),
        "rgb_sample_count": len(rgb),
        "depth_sample_count": len(depth),
        "gates": gates,
        "passed": all(gates.values()),
    }


def validate_collection_manifest(
    entries: list[dict[str, Any]],
    *,
    main_cohort: bool = False,
) -> dict[str, Any]:
    """Check split/parent/family isolation and optionally 16/4/4 counts."""

    parents: dict[str, set[str]] = {}
    families: dict[str, set[str]] = {}
    allowed_splits = (
        {"train", "validation", "test"}
        if main_cohort
        else {"dev", "train", "validation", "test"}
    )
    split_counts = {split: 0 for split in allowed_splits}
    reasons: list[str] = []
    for entry in entries:
        parent = str(entry.get("parent_episode_id", ""))
        family = str(entry.get("family_id", ""))
        split = str(entry.get("split", ""))
        source = str(entry.get("operator_kind", ""))
        result = str(entry.get("task_result", ""))
        if not parent or not family or split not in allowed_splits:
            reasons.append("missing_parent_family_or_split")
            continue
        parents.setdefault(parent, set()).add(split)
        families.setdefault(family, set()).add(split)
        split_counts[split] += 1
        if source != "human_teleop":
            reasons.append(f"non_human_source:{entry.get('episode_id')}")
        if entry.get("used_hidden_teacher_plan"):
            reasons.append(f"hidden_teacher_plan:{entry.get('episode_id')}")
        if result in {"failure", "cancelled"} and not entry.get("retained", False):
            reasons.append(f"failure_not_retained:{entry.get('episode_id')}")
    if any(len(values) > 1 for values in parents.values()):
        reasons.append("parent_split_leak")
    if any(len(values) > 1 for values in families.values()):
        reasons.append("family_split_leak")
    if main_cohort:
        if split_counts != {"train": 16, "validation": 4, "test": 4}:
            reasons.append(f"split_counts:{split_counts}")
        family_success: dict[str, int] = {}
        for entry in entries:
            if entry.get("task_result") == "success":
                key = str(entry.get("family_id"))
                family_success[key] = family_success.get(key, 0) + 1
        if len(family_success) != 12 or any(count != 2 for count in family_success.values()):
            reasons.append(f"family_success_counts:{family_success}")
    return {
        "passed": not reasons,
        "split_counts": split_counts,
        "parent_count": len(parents),
        "family_count": len(families),
        "reasons": reasons,
    }
