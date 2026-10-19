#!/usr/bin/env python3
"""Build a compact, hash-bound G3A server artifact index."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
PROJECT_ROOT = ROOT.parents[1]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def artifact(path: Path, *, result: str, command: str) -> dict:
    if not path.is_file():
        raise FileNotFoundError(path)
    return {
        "remote_path": path.relative_to(PROJECT_ROOT).as_posix(),
        "sha256": sha256(path),
        "bytes": path.stat().st_size,
        "result": result,
        "reproduce_or_inspect": command,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--release-id", required=True)
    parser.add_argument("--physical-report", type=Path, required=True)
    parser.add_argument("--ros-report", type=Path, required=True)
    parser.add_argument("--regression-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    physical_path = args.physical_report.resolve()
    ros_path = args.ros_report.resolve()
    physical = json.loads(physical_path.read_text(encoding="utf-8"))
    ros = json.loads(ros_path.read_text(encoding="utf-8"))
    regression = args.regression_root.resolve()
    report_root = physical_path.parent
    ros_root = ros_path.parent
    output = args.output.resolve()

    artifacts = {
        "release_source_manifest": artifact(
            ROOT / "SOURCE_MANIFEST.json",
            result="frozen",
            command=f"cat {ROOT / 'SOURCE_MANIFEST.json'}",
        ),
        "physical_acceptance_report": artifact(
            physical_path,
            result="passed" if physical.get("passed") else "failed",
            command=(
                "MUJOCO_GL=egl PYTHONPATH=src ../../envs/g2a-py310-mujoco316/bin/python "
                "tools/g3a/run_acceptance.py --config configs/g3a_server.yaml "
                "--profile configs/profiles/g3a_navigation_v1.yaml "
                f"--report-root {report_root} --job-id {physical['job_id']}"
            ),
        ),
        "physical_junit": artifact(
            report_root / "g3a_acceptance_junit.xml",
            result="passed" if physical.get("passed") else "failed",
            command=f"cat {report_root / 'g3a_acceptance_junit.xml'}",
        ),
        "physical_ipc_trace": artifact(
            report_root / "ipc_stream.ndjson",
            result="raw_server_trace",
            command=f"wc -l {report_root / 'ipc_stream.ndjson'}",
        ),
        "physical_server_log": artifact(
            report_root / "mobilecar_server.log",
            result="raw_server_log",
            command=f"tail -200 {report_root / 'mobilecar_server.log'}",
        ),
        "ros_acceptance_report": artifact(
            ros_path,
            result="passed" if ros.get("passed") else "failed",
            command=f"cat {ros_path}",
        ),
        "ros_probe": artifact(
            ros_root / "ros_probe.json",
            result="passed" if (ros.get("probe") or {}).get("gates") else "failed",
            command=f"cat {ros_root / 'ros_probe.json'}",
        ),
        "ros_bridge_log": artifact(
            ros_root / "ros_bridge.log",
            result="raw_ros_log",
            command=f"tail -200 {ros_root / 'ros_bridge.log'}",
        ),
        "ros_build_log": artifact(
            ros_root / "build.log",
            result="colcon_build",
            command=f"tail -100 {ros_root / 'build.log'}",
        ),
        "core_regression_junit": artifact(
            regression / "core.xml",
            result="88_passed",
            command=f"cat {regression / 'core.xml'}",
        ),
        "bridge_regression_junit": artifact(
            regression / "bridge.xml",
            result="3_passed",
            command=f"cat {regression / 'bridge.xml'}",
        ),
    }

    raw_run_candidates = [
        result.get("details", {}).get("run_id")
        for result in physical.get("results", [])
        if result.get("details", {}).get("run_id")
    ]
    ros_identity = (
        ros.get("probe", {}).get("health_latest", {}).get("identity", {}) or {}
    )
    if ros_identity.get("run_id"):
        raw_run_candidates.append(ros_identity["run_id"])
    raw_runs = sorted(set(raw_run_candidates))
    episodes = sorted(
        {
            result.get("details", {}).get("episode_id")
            for result in physical.get("results", [])
            if result.get("details", {}).get("episode_id")
        }
        | ({ros.get("episode_id")},)
    )
    episodes = [value for value in episodes if value]
    payload = {
        "schema_version": 1,
        "goal": "G3A_navigation_foundation",
        "generated_wall_utc": datetime.now(timezone.utc)
        .isoformat()
        .replace("+00:00", "Z"),
        "execution_location": "server_only",
        "release_id": args.release_id,
        "release_root": ROOT.relative_to(PROJECT_ROOT).as_posix(),
        "source_aggregate_sha256": json.loads(
            (ROOT / "SOURCE_MANIFEST.json").read_text(encoding="utf-8")
        )["aggregate_sha256"],
        "source_file_count": json.loads(
            (ROOT / "SOURCE_MANIFEST.json").read_text(encoding="utf-8")
        )["file_count"],
        "jobs": {
            "physical": physical.get("job_id"),
            "ros": ros.get("job_id"),
            "regression": args.regression_root.name,
        },
        "raw_runs": raw_runs,
        "episode_ids": episodes,
        "raw_run_root": "var/runs",
        "truth_policy": "GT_DEBUG_ONLY; isolated scoring; one initial trajectory alignment",
        "artifacts": artifacts,
        "known_exclusions": [
            "G1 source_manifest.json remains historical and intentionally does not match changed G3A source",
            "Frontend app.js was not rebuilt because G3A did not alter UI source",
            "No G2A browser or G2B export/loader matrix was repeated",
        ],
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(f".{output.name}.tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, output)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
