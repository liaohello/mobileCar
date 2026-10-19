#!/usr/bin/env python3
"""Discover human G4A-2 pilot Episodes without modifying source or creating data."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    episodes = []
    for episode_json in sorted(args.runs_root.glob("*/episodes/*/episode.json")):
        try:
            episode = json.loads(episode_json.read_text(encoding="utf-8"))
        except Exception:
            continue
        if episode.get("operator_kind") != "human_teleop":
            continue
        if episode.get("view_source") != "link4_rgbd_proprio":
            continue
        events = episode_json.parent / "events.jsonl"
        event_types = {
            json.loads(line).get("event_type")
            for line in events.read_text(encoding="utf-8").splitlines()
            if line.strip()
        }
        if "human_session_confirmed" not in event_types:
            continue
        result = str(episode.get("result", "unknown"))
        episodes.append(
            {
                "run_id": episode.get("run_id"),
                "episode_id": episode.get("episode_id"),
                "expected": "success" if result == "success" else "failure",
                "task_result": result,
                "episode_json": str(episode_json.relative_to(args.runs_root)),
            }
        )
    output = {
        "status": "discovered",
        "source": "human_teleop",
        "view_source": "link4_rgbd_proprio",
        "count": len(episodes),
        "entries": episodes,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"count": len(episodes), "episodes": episodes}, indent=2))
    return 0 if len(episodes) >= 3 else 2


if __name__ == "__main__":
    raise SystemExit(main())
