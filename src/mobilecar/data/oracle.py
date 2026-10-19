"""Isolated world-simulation oracle for the G1 grasp task.

Oracle inputs are explicit diagnostic samples.  They are never copied into the
default policy observation or used to synthesize estimated state.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping

from ..contracts import TaskAssessment, TaskResult
from .recorder import _atomic_json, _utc_now


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


def _episode_dir(root: Path, episode_id: str, run_id: str | None) -> Path:
    if run_id:
        path = root / run_id / "episodes" / episode_id
        if path.is_dir():
            return path
    matches = [path for path in root.glob(f"*/episodes/{episode_id}") if path.is_dir()]
    if len(matches) != 1:
        raise FileNotFoundError(episode_id)
    return matches[0]


def _cube_z(sample: Mapping[str, Any]) -> float | None:
    value = sample.get("cube_position_world_sim_m") or sample.get("cube_position_m")
    if isinstance(value, list) and len(value) >= 3:
        candidate = value[2]
    else:
        candidate = sample.get("cube_z_world_sim_m")
    if isinstance(candidate, (int, float)) and not isinstance(candidate, bool) and math.isfinite(candidate):
        return float(candidate)
    return None


def _bilateral(sample: Mapping[str, Any], minimum_force_n: float) -> bool:
    if bool(sample.get("bilateral_contact")):
        return True
    left = sample.get("left_contact_force_n", sample.get("left_finger_force_n", 0.0))
    right = sample.get("right_contact_force_n", sample.get("right_finger_force_n", 0.0))
    if (
        isinstance(left, (int, float))
        and isinstance(right, (int, float))
        and math.isfinite(float(left))
        and math.isfinite(float(right))
    ):
        return float(left) > minimum_force_n and float(right) > minimum_force_n
    return bool(sample.get("left_finger_contact") and sample.get("right_finger_contact"))


def assess_grasp_oracle(
    root: str | Path | Iterable[Mapping[str, Any]],
    episode_id: str | None = None,
    *,
    run_id: str | None = None,
    samples: Iterable[Mapping[str, Any]] | None = None,
    lift_threshold_m: float = 0.05,
    hold_threshold_s: float = 2.0,
    minimum_force_n: float = 0.0,
    write: bool = True,
) -> TaskAssessment:
    """Assess physical lift and bilateral hold from isolated world samples."""

    if lift_threshold_m <= 0 or hold_threshold_s <= 0:
        raise ValueError("oracle lift and hold thresholds must be positive")
    episode_dir: Path | None = None
    if isinstance(root, (str, Path)):
        path = Path(root)
        if (path / "episode.json").is_file():
            episode_dir = path
            episode_id = episode_id or path.name
        else:
            if episode_id is None:
                raise ValueError("episode_id is required when root is a runs directory")
            episode_dir = _episode_dir(path, episode_id, run_id)
        sample_rows = list(samples or _read_jsonl(episode_dir / "assessments/oracle_samples.jsonl"))
    else:
        if episode_id is None:
            raise ValueError("episode_id is required for an assessment")
        sample_rows = list(root)

    ordered = sorted(
        (dict(sample) for sample in sample_rows),
        key=lambda item: float(item.get("sim_time_s", item.get("backend_monotonic_s", 0.0))),
    )
    if any(
        str(sample.get("oracle_status", "")).upper() == "NOT_APPLICABLE"
        for sample in ordered
    ):
        assessment = TaskAssessment(
            episode_id=episode_id,
            task_result=TaskResult.NOT_APPLICABLE,
            oracle_source="world_sim_isolated_oracle_v1",
            evidence={
                "reason": "target_cube_not_present",
                "total_samples": len(ordered),
                "oracle_access": "GT_DEBUG_ONLY",
            },
        )
        if write and episode_dir is not None:
            _atomic_json(
                episode_dir / "assessments/task_assessment.json",
                assessment.model_dump(mode="json"),
            )
            episode_path = episode_dir / "episode.json"
            if episode_path.is_file():
                metadata = json.loads(episode_path.read_text(encoding="utf-8"))
                metadata["result"] = assessment.task_result
                metadata["task_result_source"] = assessment.oracle_source
                metadata["task_assessed_wall_utc"] = _utc_now()
                _atomic_json(episode_path, metadata)
        return assessment
    times = [
        float(sample.get("sim_time_s", sample.get("backend_monotonic_s", 0.0)))
        for sample in ordered
    ]
    cube_z = [_cube_z(sample) for sample in ordered]
    valid = [
        (time_s, z_value, _bilateral(sample, minimum_force_n))
        for time_s, z_value, sample in zip(times, cube_z, ordered)
        if z_value is not None and math.isfinite(time_s)
    ]
    if not valid:
        assessment = TaskAssessment(
            episode_id=episode_id,
            task_result=TaskResult.UNKNOWN,
            oracle_source="world_sim_isolated_oracle_v1",
            evidence={"reason": "no finite cube world-state samples"},
        )
    else:
        initial_z = valid[0][1]
        lifts = [(time_s, z - initial_z, contact) for time_s, z, contact in valid]
        max_lift = max(lift for _, lift, _ in lifts)
        contacts = [1.0 if contact else 0.0 for _, _, contact in lifts]
        bilateral_fraction = sum(contacts) / len(contacts)

        best_hold = 0.0
        hold_start: float | None = None
        best_hold_start: float | None = None
        best_hold_end: float | None = None
        for time_s, lift, contact in lifts:
            qualifies = contact and lift >= lift_threshold_m
            if qualifies:
                if hold_start is None:
                    hold_start = time_s
                duration = time_s - hold_start
                if duration > best_hold:
                    best_hold = duration
                    best_hold_start = hold_start
                    best_hold_end = time_s
            else:
                hold_start = None
        # A terminal sample represents an interval only through its timestamp;
        # require measured duration, not an optimistic extra sample period.
        success = max_lift >= lift_threshold_m and best_hold >= hold_threshold_s
        assessment = TaskAssessment(
            episode_id=episode_id,
            task_result=TaskResult.SUCCESS if success else TaskResult.FAILURE,
            oracle_source="world_sim_isolated_oracle_v1",
            lift_height_m=max_lift,
            hold_duration_s=best_hold,
            bilateral_contact_fraction=bilateral_fraction,
            evidence={
                "sample_count": len(valid),
                "total_samples": len(ordered),
                "initial_cube_z_m": initial_z,
                "max_cube_z_m": initial_z + max_lift,
                "lift_threshold_m": lift_threshold_m,
                "hold_threshold_s": hold_threshold_s,
                "best_hold_start_sim_s": best_hold_start,
                "best_hold_end_sim_s": best_hold_end,
                "bilateral_contact_required": True,
                "oracle_access": "GT_DEBUG_ONLY",
            },
        )
    if write and episode_dir is not None:
        _atomic_json(episode_dir / "assessments/task_assessment.json", assessment.model_dump(mode="json"))
        episode_path = episode_dir / "episode.json"
        if episode_path.is_file():
            metadata = json.loads(episode_path.read_text(encoding="utf-8"))
            metadata["result"] = assessment.task_result
            metadata["task_result_source"] = assessment.oracle_source
            metadata["task_assessed_wall_utc"] = _utc_now()
            _atomic_json(episode_path, metadata)
    return assessment
