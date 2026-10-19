#!/usr/bin/env python3
"""Run required G3B negative, sensor-policy and RViz interaction subchecks."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from uuid import uuid4

import yaml

from mobilecar_mapping_localization.map_package import (
    canonical_manifest_digest,
    compatibility_diff,
    load_manifest,
    load_occupancy_package,
    occupancy_goal_check,
)
from run_formal_acceptance import current_compatibility, project_path
from run_single_goal import PYTHON, ROOT, atomic_json


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def unknown_coordinate(map_root: Path) -> tuple[float, float]:
    package = load_occupancy_package(map_root)
    image = package["image"]
    width = int(image["width"])
    height = int(image["height"])
    origin_x, origin_y, _yaw = package["origin_xy_yaw"]
    resolution = float(package["resolution_m"])
    pixels = image["pixels"]
    for row in range(height):
        for column in range(width):
            gray = pixels[row * width + column]
            probability = 1.0 - gray / float(image["maximum"])
            if package["free_thresh"] <= probability <= package["occupied_thresh"]:
                return (
                    origin_x + (column + 0.5) * resolution,
                    origin_y + (height - row - 0.5) * resolution,
                )
    raise RuntimeError("no unknown occupancy cell available for negative goal test")


def tampered_map(source: Path, target: Path) -> Path:
    target.mkdir(parents=True, exist_ok=True)
    source_manifest = load_manifest(source)
    for asset in source_manifest["assets"]:
        source_path = source / asset["relative_path"]
        target_path = target / asset["relative_path"]
        target_path.parent.mkdir(parents=True, exist_ok=True)
        if asset["role"] == "map_manifest":
            continue
        if not target_path.exists():
            target_path.symlink_to(source_path.resolve())
    manifest = json.loads(
        (source / "manifest.json").read_text(encoding="utf-8")
    )
    manifest["scene_id"] = "incompatible_scene_sha_probe"
    manifest["assets"] = [
        item for item in manifest["assets"] if item["role"] != "map_manifest"
    ]
    digest = canonical_manifest_digest(manifest)
    digest_path = target / "manifest.sha256"
    digest_path.write_text(f"{digest}  manifest.json\n", encoding="utf-8")
    manifest["assets"].append(
        {
            "role": "map_manifest",
            "relative_path": "manifest.sha256",
            "sha256": sha256(digest_path),
            "bytes": digest_path.stat().st_size,
        }
    )
    (target / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return target


def run_negative(
    *,
    name: str,
    scenario: dict[str, Any],
    report_root: Path,
    domain_id: int,
    x_m: float,
    y_m: float,
    frame_id: str = "map",
    map_root: Path | None = None,
) -> dict[str, Any]:
    case_root = report_root / "cases" / name
    command = [
        str(PYTHON),
        "tools/g3b/run_single_goal.py",
        "--config",
        scenario["config"],
        "--map-root",
        str(
            map_root.resolve()
            if map_root is not None
            else project_path(scenario["map_root"])
        ),
        "--map-id",
        scenario["map_id"],
        "--goal-id",
        name,
        "--goal-frame",
        frame_id,
        "--goal-x",
        str(x_m),
        "--goal-y",
        str(y_m),
        "--goal-yaw",
        "0.0",
        "--arm-mode",
        "hold",
        "--special-case",
        "normal",
        "--expect-goal-rejection",
        "--report-root",
        str(case_root),
        "--domain-id",
        str(domain_id),
    ]
    started = time.monotonic()
    process = subprocess.run(
        command,
        cwd=ROOT,
        env=os.environ.copy(),
        text=True,
        capture_output=True,
        timeout=240,
    )
    report_path = case_root / "g3b_single_goal_report.json"
    case_report = (
        json.loads(report_path.read_text(encoding="utf-8"))
        if report_path.is_file()
        else {}
    )
    return {
        "name": name,
        "command": command,
        "exit_code": process.returncode,
        "duration_wall_s": time.monotonic() - started,
        "stdout": process.stdout,
        "stderr": process.stderr,
        "report_path": str(report_path),
        "report_sha256": sha256(report_path) if report_path.is_file() else None,
        "success": bool(case_report.get("success")),
        "gates": case_report.get("gates", {}),
        "goal_supervisor_status": case_report.get("goal_supervisor_status"),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--cases",
        type=Path,
        default=Path("configs/g3b/cases_v1.yaml"),
    )
    parser.add_argument("--formal-report", type=Path, required=True)
    parser.add_argument("--report-root", type=Path, required=True)
    parser.add_argument("--domain-base", type=int, default=120)
    args = parser.parse_args()
    report_root = args.report_root.resolve()
    report_root.mkdir(parents=True, exist_ok=True)
    cases = yaml.safe_load(args.cases.read_text(encoding="utf-8"))
    scenario = cases["scenarios"]["room_a"]
    map_root = project_path(scenario["map_root"])
    report: dict[str, Any] = {
        "schema_version": 1,
        "report_version": "g3b-subchecks-1",
        "generated_wall_utc": utc_now(),
        "execution_location": "server_only",
        "cases_manifest_sha256": sha256(args.cases.resolve()),
        "success": False,
    }
    try:
        formal = json.loads(args.formal_report.read_text(encoding="utf-8"))
        a01 = next(
            item for item in formal["cases"] if item["id"] == "A01_hold_room_a"
        )
        profile = yaml.safe_load(
            (ROOT / "configs/profiles/g3b_navigation_v1.yaml").read_text(
                encoding="utf-8"
            )
        )
        nav2 = yaml.safe_load(
            (
                ROOT
                / "ros2_ws/src/mobilecar_nav_bringup/config/nav2_g3b.yaml"
            ).read_text(encoding="utf-8")
        )
        localization_launch = (
            ROOT
            / "ros2_ws/src/mobilecar_mapping_localization/launch/localization.launch.py"
        ).read_text(encoding="utf-8")
        map_tf = a01.get("map_to_odom_runtime") or {}
        scan_policy = a01.get("scan_policy_evidence") or {}
        static_checks = {
            "footprint": {
                "profile_polygon": profile["nav2"]["footprint_polygon_m"],
                "local_footprint": nav2["local_costmap"]["local_costmap"][
                    "ros__parameters"
                ]["footprint"],
                "global_footprint": nav2["global_costmap"]["global_costmap"][
                    "ros__parameters"
                ]["footprint"],
                "local_obstacle_footprint_critic": (
                    "ObstacleFootprint"
                    in nav2["controller_server"]["ros__parameters"]["FollowPath"][
                        "critics"
                    ]
                ),
                "passed": (
                    nav2["local_costmap"]["local_costmap"]["ros__parameters"][
                        "footprint"
                    ]
                    == "[[-0.16,-0.18],[0.46,-0.18],[0.46,0.18],[-0.16,0.18]]"
                    and nav2["global_costmap"]["global_costmap"]["ros__parameters"][
                        "footprint"
                    ]
                    == "[[-0.16,-0.18],[0.46,-0.18],[0.46,0.18],[-0.16,0.18]]"
                    and "ObstacleFootprint"
                    in nav2["controller_server"]["ros__parameters"]["FollowPath"][
                        "critics"
                    ]
                ),
            },
            "duplicate_map_to_odom": {
                "amcl_reference_present": "amcl" in localization_launch.lower(),
                "slam_localization_node_count": localization_launch.count(
                    "localization_slam_toolbox_node"
                ),
                "runtime_message_count": map_tf.get("message_count"),
                "runtime_unique_stamps": map_tf.get("unique_stamp_count"),
                "runtime_duplicate_stamps": map_tf.get(
                    "duplicate_stamp_count"
                ),
                "runtime_conflicting_stamps": map_tf.get(
                    "conflicting_stamp_count"
                ),
                "runtime_identical_repeats": map_tf.get(
                    "identical_repeat_count"
                ),
                "runtime_minor_same_stamp_updates": map_tf.get(
                    "minor_same_stamp_update_count"
                ),
                "runtime_max_same_stamp_translation_delta_m": map_tf.get(
                    "max_same_stamp_translation_delta_m"
                ),
                "runtime_max_same_stamp_rotation_delta_rad": map_tf.get(
                    "max_same_stamp_rotation_delta_rad"
                ),
                "runtime_observed_rate_hz": map_tf.get("observed_rate_hz"),
                "passed": (
                    "amcl" not in localization_launch.lower()
                    and localization_launch.count(
                        "localization_slam_toolbox_node"
                    )
                    == 1
                    and map_tf.get("message_count", 0) > 0
                    and map_tf.get("conflicting_stamp_count") == 0
                    and map_tf.get("observed_rate_hz") is not None
                    and map_tf["observed_rate_hz"] <= 15.0
                ),
                "limitation": (
                    "ROS /tf does not expose publisher identity; runtime rate/"
                    "duplicate stamps plus source ownership are checked."
                ),
            },
            "nan_inf_policy": {
                **scan_policy,
                "passed": (
                    scan_policy.get("actual_scan", {}).get("beam_count", 0) > 0
                    and scan_policy.get("nav2_inf_is_valid") is False
                    and scan_policy.get("invalid_or_self_occluded_is_free")
                    is False
                ),
            },
        }
        report["static_checks"] = static_checks

        invalid_reports = [
            run_negative(
                name="invalid_frame_odom",
                scenario=scenario,
                report_root=report_root,
                domain_id=args.domain_base,
                x_m=0.0,
                y_m=0.0,
                frame_id="odom",
            ),
            run_negative(
                name="outside_saved_map",
                scenario=scenario,
                report_root=report_root,
                domain_id=args.domain_base + 1,
                x_m=99.0,
                y_m=99.0,
            ),
        ]
        unknown_x, unknown_y = unknown_coordinate(map_root)
        invalid_reports.append(
            run_negative(
                name="unknown_map_cell",
                scenario=scenario,
                report_root=report_root,
                domain_id=args.domain_base + 2,
                x_m=unknown_x,
                y_m=unknown_y,
            )
        )
        incompatible_root = tampered_map(
            map_root, report_root / "incompatible_map_package"
        )
        incompatible_report = run_negative(
            name="incompatible_map_runtime_rejection",
            scenario=scenario,
            report_root=report_root,
            domain_id=args.domain_base + 3,
            x_m=0.0,
            y_m=0.0,
            map_root=incompatible_root,
        )
        invalid_reports.append(incompatible_report)
        report["invalid_goal_checks"] = invalid_reports

        gui_root = report_root / "rviz_gui"
        gui_command = [
            str(PYTHON),
            "tools/g3b/run_rviz_gui.py",
            "--config",
            scenario["config"],
            "--map-root",
            str(map_root),
            "--map-id",
            scenario["map_id"],
            "--report-root",
            str(gui_root),
            "--domain-id",
            str(args.domain_base + 4),
            "--display-number",
            "96",
        ]
        gui_process = subprocess.run(
            gui_command,
            cwd=ROOT,
            env=os.environ.copy(),
            text=True,
            capture_output=True,
            timeout=300,
        )
        gui_report_path = gui_root / "g3b_rviz_gui_report.json"
        gui_report = (
            json.loads(gui_report_path.read_text(encoding="utf-8"))
            if gui_report_path.is_file()
            else {}
        )
        report["gui_rviz"] = {
            "command": gui_command,
            "exit_code": gui_process.returncode,
            "stdout": gui_process.stdout,
            "stderr": gui_process.stderr,
            "report_path": str(gui_report_path),
            "success": bool(gui_report.get("success")),
            "gates": gui_report.get("gates", {}),
            "screenshot_path": gui_report.get("screenshot_path"),
        }
        report["success"] = (
            all(item["passed"] for item in static_checks.values())
            and all(item["success"] for item in invalid_reports)
            and report["gui_rviz"]["success"]
        )
    except Exception:
        report["error"] = traceback.format_exc()
    finally:
        atomic_json(report_root / "g3b_subchecks_report.json", report)
    print(
        json.dumps(
            {
                "success": report.get("success", False),
                "report": str(report_root / "g3b_subchecks_report.json"),
            },
            indent=2,
        )
    )
    return 0 if report.get("success") else 1


if __name__ == "__main__":
    raise SystemExit(main())
