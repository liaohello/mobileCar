from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
MAPPING_ROOT = ROOT / "ros2_ws/src/mobilecar_mapping_localization"
sys.path.insert(0, str(MAPPING_ROOT))

from mobilecar_mapping_localization.map_package import (  # noqa: E402
    build_manifest,
    canonical_manifest_digest,
    compatibility_diff,
    load_manifest,
    occupancy_goal_check,
)


HASH = "a" * 64


def make_package(root: Path) -> Path:
    pixels = bytes(
        [
            254,
            254,
            0,
            254,
            254,
            205,
            0,
            254,
            254,
            205,
            254,
            254,
            254,
            254,
            254,
            254,
        ]
    )
    (root / "map.pgm").write_bytes(b"P5\n4 4\n255\n" + pixels)
    (root / "map.yaml").write_text(
        yaml.safe_dump(
            {
                "image": "map.pgm",
                "mode": "trinary",
                "resolution": 0.05,
                "origin": [-0.1, -0.1, 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    (root / "posegraph.posegraph").write_bytes(b"pose-graph")
    (root / "posegraph.data").write_bytes(b"pose-data")
    build_manifest(
        package_root=root,
        profile_id="g3b-nav2-omni-map-v1",
        profile_version="1.0",
        profile_sha256=HASH,
        map_id="map_fixture",
        scene_id="scene_fixture",
        model_sha256=HASH,
        navigation_profile_sha256=HASH,
        sensor_profile_sha256=HASH,
        scan_config_sha256=HASH,
        resolution_m=0.05,
        origin_xy_yaw=[-0.1, -0.1, 0.0],
        occupied_thresh=0.65,
        free_thresh=0.196,
        created_run_id="run_fixture",
        created_episode_id="episode_fixture",
        created_navigation_epoch=1,
        initial_prior={
            "x_m": 0.0,
            "y_m": 0.0,
            "yaw_rad": 0.0,
            "covariance": [0.0] * 36,
        },
    )
    return root


def test_manifest_round_trip_and_canonical_digest(tmp_path):
    root = make_package(tmp_path)
    manifest = load_manifest(root)
    digest = (root / "manifest.sha256").read_text(encoding="utf-8").split()[0]
    assert digest == canonical_manifest_digest(manifest)
    roles = {asset["role"] for asset in manifest["assets"]}
    assert roles == {
        "occupancy_yaml",
        "occupancy_image",
        "pose_graph",
        "pose_graph_data",
        "map_manifest",
    }


def test_missing_pose_graph_is_rejected(tmp_path):
    root = make_package(tmp_path)
    (root / "posegraph.posegraph").unlink()
    with pytest.raises(FileNotFoundError):
        load_manifest(root)


def test_compatibility_diff_is_field_scoped(tmp_path):
    manifest = load_manifest(make_package(tmp_path))
    current = {
        field: manifest.get(field) for field in manifest["compatibility_fields"]
    }
    current["resolution_m"] = 0.10
    diff = compatibility_diff(manifest, current)
    assert diff["compatible"] is False
    assert set(diff["mismatches"]) == {"resolution_m"}
    assert diff["whole_source_aggregate_used"] is False


def test_unknown_occupied_and_outside_goals_are_not_free(tmp_path):
    root = make_package(tmp_path)
    assert occupancy_goal_check(root, -0.05, -0.025)["classification"] == "unknown"
    assert occupancy_goal_check(root, 0.00, 0.075)["classification"] == "occupied"
    outside = occupancy_goal_check(root, 1.0, 1.0)
    assert outside["classification"] == "outside_map"
    assert outside["accepted"] is False
    assert occupancy_goal_check(root, 0.00, -0.025)["accepted"] is True


def test_manifest_asset_hash_tamper_is_rejected(tmp_path):
    root = make_package(tmp_path)
    manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
    asset = next(
        item for item in manifest["assets"] if item["role"] == "pose_graph_data"
    )
    (root / asset["relative_path"]).write_bytes(b"tampered")
    with pytest.raises(ValueError, match="hash mismatch"):
        load_manifest(root)
