"""Read-only Episode indexing and inspection.

Replay never imports a backend, advances physics, or submits commands.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any


def _read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path}: expected a JSON object")
    return value


def _read_jsonl(path: Path, limit: int | None = None) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            text = line.strip()
            if not text:
                continue
            value = json.loads(text)
            if isinstance(value, dict):
                rows.append(value)
            if limit is not None and len(rows) >= limit:
                break
    return rows


class ReplayReader:
    def __init__(
        self,
        root: str | Path | None = None,
        run_root: str | Path | None = None,
        **_unused: Any,
    ) -> None:
        self.root = Path(root or run_root or "runs").resolve()

    def _episode_dir(self, episode_id: str) -> tuple[Path, Path]:
        matches = [path for path in self.root.glob(f"*/episodes/{episode_id}") if path.is_dir()]
        if len(matches) != 1:
            raise KeyError(episode_id)
        episode_dir = matches[0].resolve()
        if not episode_dir.is_relative_to(self.root):
            raise KeyError(episode_id)
        return episode_dir.parent.parent, episode_dir

    def list_episodes(self) -> dict[str, Any]:
        episodes: list[dict[str, Any]] = []
        for path in sorted(self.root.glob("*/episodes/*/episode.json")):
            try:
                episode = _read_json(path)
            except (OSError, ValueError, json.JSONDecodeError):
                continue
            episode_dir = path.parent
            quality_path = episode_dir / "quality_report.json"
            assessment_path = episode_dir / "assessments/task_assessment.json"
            quality = _read_json(quality_path) if quality_path.is_file() else {}
            assessment = _read_json(assessment_path) if assessment_path.is_file() else {}
            episodes.append(
                {
                    "episode_id": episode.get("episode_id", episode_dir.name),
                    "run_id": episode.get("run_id", path.parents[2].name),
                    "task_language": episode.get("task_language"),
                    "operator_kind": episode.get("operator_kind"),
                    "view_source": episode.get("view_source"),
                    "recording_status": episode.get("recording_status", "UNKNOWN"),
                    "audit_status": quality.get("audit_status", "PENDING"),
                    "task_result": assessment.get("task_result", episode.get("result", "unknown")),
                    "termination_reason": episode.get("termination_reason"),
                    "segment_count": len(episode.get("segments", [])),
                    "complete_tail": (episode_dir / "episode_complete.json").is_file(),
                }
            )
        return {"contract_version": "mobilecar-1.0", "episodes": episodes}

    def read_episode(self, episode_id: str) -> dict[str, Any]:
        run_dir, episode_dir = self._episode_dir(episode_id)
        manifest_path = run_dir / "manifest.json"
        episode_path = episode_dir / "episode.json"
        if not manifest_path.is_file() or not episode_path.is_file():
            raise FileNotFoundError(episode_id)
        manifest = _read_json(manifest_path)
        episode = _read_json(episode_path)
        events = _read_jsonl(episode_dir / "events.jsonl")
        observations_index = _read_jsonl(episode_dir / "observations/index.jsonl")
        quality_path = episode_dir / "quality_report.json"
        assessment_path = episode_dir / "assessments/task_assessment.json"
        eligibility_path = episode_dir / "assessments/eligibility.json"
        validation = _read_json(quality_path) if quality_path.is_file() else {
            "audit_status": "PENDING",
            "errors": [],
            "warnings": ["validation has not been run"],
        }
        assessment = _read_json(assessment_path) if assessment_path.is_file() else None
        eligibility = _read_json(eligibility_path) if eligibility_path.is_file() else None
        media: dict[str, list[str]] = {"rgb": [], "depth": [], "masks": []}
        for kind, folder in (("rgb", "rgb"), ("depth", "depth"), ("masks", "masks")):
            media[kind] = sorted(
                path.relative_to(episode_dir).as_posix()
                for path in (episode_dir / "assets" / folder).glob("*")
                if path.is_file()
            )
        return {
            "contract_version": "mobilecar-1.0",
            "run_id": run_dir.name,
            "episode_id": episode_id,
            "episode": episode,
            "manifest": manifest,
            "events": events,
            "observations": observations_index,
            "validation": validation,
            "assessment": assessment or {"task_result": episode.get("result", "unknown")},
            "eligibility": eligibility,
            "media": media,
            "latest_rgb_uri": media["rgb"][-1] if media["rgb"] else None,
            "read_only": True,
        }

    def read_jsonl(self, episode_id: str, relative_path: str) -> list[dict[str, Any]]:
        _, episode_dir = self._episode_dir(episode_id)
        path = (episode_dir / relative_path).resolve()
        if not path.is_relative_to(episode_dir) or not path.is_file():
            raise FileNotFoundError(relative_path)
        return _read_jsonl(path)

    def resolve_asset(self, episode_id: str, relative_path: str) -> Path:
        _, episode_dir = self._episode_dir(episode_id)
        path = (episode_dir / relative_path).resolve()
        if not path.is_relative_to(episode_dir) or not path.is_file():
            raise FileNotFoundError(relative_path)
        return path

    def close(self) -> None:
        """Replay owns no physical resources; retained for application cleanup."""

