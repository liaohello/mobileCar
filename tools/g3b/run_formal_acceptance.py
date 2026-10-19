#!/usr/bin/env python3
"""Run the frozen G3B ten-case set and aggregate independent evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from uuid import uuid4

import yaml

from mobilecar.config import load_config
from mobilecar.contracts.g3b import MapPackageManifest
from mobilecar.data.validator import validate_episode
from mobilecar_mapping_localization.map_package import (
    compatibility_diff,
    load_manifest,
    occupancy_goal_check,
)


ROOT = Path(__file__).resolve().parents[2]
PROJECT_ROOT = ROOT.parents[1] if ROOT.parent.name == "releases" else ROOT
PYTHON = PROJECT_ROOT / "envs/g2a-py310-mujoco316/bin/python"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def atomic_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{uuid4().hex}.tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def project_path(value: str | Path) -> Path:
    path = Path(value)
    return path.resolve() if path.is_absolute() else (PROJECT_ROOT / path).resolve()


def current_compatibility(config_path: Path) -> dict[str, Any]:
    config = load_config(config_path).data
    backend = config["backend"]
    profile_path = Path(config["g3b"]["profile_path"]).resolve()
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    navigation_profile_path = Path(
        backend["navigation_profile_path"]
    ).resolve()
    navigation_profile = yaml.safe_load(
        navigation_profile_path.read_text(encoding="utf-8")
    )
    scan_hash = hashlib.sha256(
        json.dumps(
            navigation_profile["sensors"]["scan"],
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
    ).hexdigest()
    return {
        "scene_id": Path(backend["scene_path"]).stem,
        "model_sha256": sha256(Path(backend["model_path"]).resolve()),
        "navigation_profile_sha256": sha256(navigation_profile_path),
        "sensor_profile_sha256": sha256(
            Path(backend["observation_profile_path"]).resolve()
        ),
        "scan_config_sha256": scan_hash,
        "resolution_m": float(profile["map_package"]["resolution_m"]),
        "map_frame": str(profile["map_package"]["map_frame"]),
        "odom_frame": str(profile["map_package"]["odom_frame"]),
        "base_frame": str(profile["map_package"]["base_frame"]),
    }


def case_command(
    case: dict[str, Any],
    scenario: dict[str, Any],
    report_root: Path,
    domain_id: int,
) -> list[str]:
    goal = case["goal"]
    command = [
        str(PYTHON),
        "tools/g3b/run_single_goal.py",
        "--config",
        scenario["config"],
        "--map-root",
        str(project_path(scenario["map_root"])),
        "--map-id",
        scenario["map_id"],
        "--goal-id",
        case["id"],
        "--goal-x",
        str(goal["x_m"]),
        "--goal-y",
        str(goal["y_m"]),
        "--goal-yaw",
        str(goal["yaw_rad"]),
        "--arm-mode",
        case["arm_mode"],
        "--special-case",
        case.get("special_case", "normal"),
        "--report-root",
        str(report_root / "cases" / case["id"]),
        "--domain-id",
        str(domain_id),
        "--goal-timeout-wall",
        "180",
    ]
    if "post_goal" in case:
        post = case["post_goal"]
        command.extend(
            [
                "--post-goal-x",
                str(post["x_m"]),
                "--post-goal-y",
                str(post["y_m"]),
                "--post-goal-yaw",
                str(post["yaw_rad"]),
            ]
        )
    if "fault_trigger_progress_m" in case:
        command.extend(
            [
                "--fault-trigger-progress-m",
                str(case["fault_trigger_progress_m"]),
            ]
        )
    if case.get("special_case") == "localization_failure":
        command.append("--enable-localization-test-faults")
    return command


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--cases",
        type=Path,
        default=Path("configs/g3b/cases_v1.yaml"),
    )
    parser.add_argument("--report-root", type=Path, required=True)
    parser.add_argument("--domain-base", type=int, default=100)
    parser.add_argument("--case-timeout-s", type=int, default=360)
    args = parser.parse_args()
    report_root = args.report_root.resolve()
    report_root.mkdir(parents=True, exist_ok=True)
    cases_path = args.cases.resolve()
    cases = yaml.safe_load(cases_path.read_text(encoding="utf-8"))
    formal_cases = cases.get("formal_cases", [])
    if len(formal_cases) != 10 or len({item["id"] for item in formal_cases}) != 10:
        raise RuntimeError("the frozen G3B set must contain exactly ten unique cases")
    if cases.get("frozen_for_release") is not True:
        raise RuntimeError("formal case manifest is not frozen for release")
    cases_digest = sha256(cases_path)
    frozen_path = report_root / "cases_frozen.yaml"
    shutil.copyfile(cases_path, frozen_path)
    report: dict[str, Any] = {
        "schema_version": 1,
        "report_version": "g3b-formal-1",
        "interface_profile": "g3b-1.0",
        "generated_wall_utc": utc_now(),
        "execution_location": "server_only",
        "cases_manifest_path": str(cases_path),
        "cases_manifest_sha256": cases_digest,
        "frozen_cases_copy": str(frozen_path),
        "formal_gates": cases["formal_gates"],
        "expected_case_count": 10,
        "cases": [],
        "prechecks": {},
        "data": {},
        "training_qualified": False,
        "success": False,
    }
    try:
        prechecks: dict[str, Any] = {}
        scenarios = cases["scenarios"]
        for scenario_id, scenario in scenarios.items():
            map_root = project_path(scenario["map_root"])
            manifest = load_manifest(map_root)
            MapPackageManifest.model_validate(manifest)
            current = current_compatibility((ROOT / scenario["config"]).resolve())
            compatibility = compatibility_diff(manifest, current)
            goals = [
                item["goal"]
                for item in formal_cases
                if item["scenario"] == scenario_id
            ]
            goal_checks = [
                occupancy_goal_check(
                    map_root, goal["x_m"], goal["y_m"]
                )
                for goal in goals
            ]
            prechecks[scenario_id] = {
                "map_id": scenario["map_id"],
                "manifest_path": str(map_root / "manifest.json"),
                "manifest_sha256": sha256(map_root / "manifest.json"),
                "compatibility": compatibility,
                "goal_checks": goal_checks,
                "map_free_thresh": float(
                    yaml.safe_load(
                        (map_root / "map.yaml").read_text(encoding="utf-8")
                    )["free_thresh"]
                ),
            }
        report["prechecks"] = prechecks
        if not all(
            item["compatibility"]["compatible"]
            and all(goal["accepted"] for goal in item["goal_checks"])
            and item["map_free_thresh"] <= 0.196
            for item in prechecks.values()
        ):
            raise RuntimeError("map compatibility or frozen goal precheck failed")

        for index, case in enumerate(formal_cases):
            scenario = scenarios[case["scenario"]]
            command = case_command(
                case,
                scenario,
                report_root,
                args.domain_base + index,
            )
            started = time.monotonic()
            process_result = subprocess.run(
                command,
                cwd=ROOT,
                env=os.environ.copy(),
                text=True,
                capture_output=True,
                timeout=args.case_timeout_s,
            )
            duration = time.monotonic() - started
            case_root = report_root / "cases" / case["id"]
            case_report_path = case_root / "g3b_single_goal_report.json"
            case_report = (
                json.loads(case_report_path.read_text(encoding="utf-8"))
                if case_report_path.is_file()
                else {}
            )
            console_path = case_root / "runner_console.log"
            console_path.parent.mkdir(parents=True, exist_ok=True)
            console_path.write_text(
                process_result.stdout
                + ("\nSTDERR\n" + process_result.stderr if process_result.stderr else ""),
                encoding="utf-8",
            )
            report["cases"].append(
                {
                    "id": case["id"],
                    "scenario": case["scenario"],
                    "category": case["category"],
                    "arm_mode": case["arm_mode"],
                    "special_case": case.get("special_case", "normal"),
                    "command": command,
                    "exit_code": process_result.returncode,
                    "duration_wall_s": duration,
                    "success": bool(case_report.get("success", False)),
                    "gates": case_report.get("gates", {}),
                    "action_status": case_report.get("action_status"),
                    "first_action_status": case_report.get("first_action_status"),
                    "second_action_status": case_report.get("second_action_status"),
                    "position_error_m": case_report.get("position_error_m"),
                    "yaw_error_deg": case_report.get("yaw_error_deg"),
                    "settled_duration_s": case_report.get("settled_duration_s"),
                    "unexpected_contacts": case_report.get(
                        "unexpected_contacts", []
                    ),
                    "episode_id": case_report.get("episode_id"),
                    "run_id": case_report.get("run_id"),
                    "report_path": str(case_report_path),
                    "report_sha256": (
                        sha256(case_report_path)
                        if case_report_path.is_file()
                        else None
                    ),
                    "special_case_evidence": case_report.get(
                        "special_case_evidence"
                    ),
                    "backend_rejection_receipt": case_report.get(
                        "backend_rejection_receipt"
                    ),
                    "map_to_odom_runtime": case_report.get(
                        "map_to_odom_runtime"
                    ),
                    "scan_policy_evidence": case_report.get(
                        "scan_policy_evidence"
                    ),
                    "goal_supervisor_status": case_report.get(
                        "goal_supervisor_status"
                    ),
                }
            )
            atomic_json(report_root / "g3b_formal_report.json", report)
            time.sleep(1.0)

        recording_root = (ROOT / config_recording_root(cases)).resolve()
        data_rows = []
        for case in report["cases"]:
            if not case.get("episode_id") or not case.get("run_id"):
                data_rows.append(
                    {
                        "case_id": case["id"],
                        "format_valid": False,
                        "reason": "missing_episode_identity",
                    }
                )
                continue
            try:
                validation = validate_episode(
                    recording_root,
                    case["episode_id"],
                    run_id=case["run_id"],
                ).model_dump(mode="json")
                data_rows.append(
                    {
                        "case_id": case["id"],
                        "episode_id": case["episode_id"],
                        "run_id": case["run_id"],
                        "audit_status": validation.get("audit_status"),
                        "format_valid": (
                            validation.get("audit_status") == "COMPLETE"
                            and not validation.get("errors")
                        ),
                        "errors": validation.get("errors", []),
                        "warnings": validation.get("warnings", []),
                    }
                )
            except Exception as exc:
                data_rows.append(
                    {
                        "case_id": case["id"],
                        "episode_id": case["episode_id"],
                        "run_id": case["run_id"],
                        "format_valid": False,
                        "reason": f"{type(exc).__name__}: {exc}",
                    }
                )
        report["data"] = {
            "episodes": data_rows,
            "format_valid": all(item.get("format_valid") for item in data_rows),
            "data_complete": all(item.get("format_valid") for item in data_rows),
            "training_qualified": False,
            "training_qualification_basis": (
                "G3B navigation Episodes are not human manipulation demonstrations "
                "and are outside the training-qualified scope."
            ),
        }
        report["case_success_count"] = sum(
            1 for item in report["cases"] if item["success"]
        )
        report["action_success_count"] = sum(
            1 for item in report["cases"] if item.get("action_status") == 4
        )
        report["success"] = (
            report["case_success_count"] == 10
            and report["data"]["format_valid"]
            and report["training_qualified"] is False
        )
    except Exception:
        report["error"] = traceback.format_exc()
    finally:
        atomic_json(report_root / "g3b_formal_report.json", report)
    print(
        json.dumps(
            {
                "success": report.get("success", False),
                "case_success_count": report.get("case_success_count", 0),
                "report": str(report_root / "g3b_formal_report.json"),
            },
            indent=2,
        )
    )
    return 0 if report.get("success") else 1


def config_recording_root(cases: dict[str, Any]) -> str:
    roots = {
        load_config((ROOT / item["config"]).resolve()).data["recording"]["root"]
        for item in cases["scenarios"].values()
    }
    if len(roots) != 1:
        raise RuntimeError("all formal scenarios must share one recording root")
    return roots.pop()


if __name__ == "__main__":
    raise SystemExit(main())
