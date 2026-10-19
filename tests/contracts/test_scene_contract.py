import json
from pathlib import Path

from mobilecar.contracts.models import RobotSnapshot, SceneManifest


ROOT = Path(__file__).resolve().parents[2]


def test_scene_manifest_fixture_round_trip():
    raw = json.loads((ROOT / "configs/examples/scene_fixture.json").read_text())
    scene = SceneManifest.model_validate(raw)
    assert scene.camera_ids == ["rgbd"]
    assert scene.links[0].name == "base_link"
    assert SceneManifest.model_validate_json(scene.model_dump_json()) == scene


def test_snapshot_fixture_round_trip():
    raw = json.loads((ROOT / "configs/examples/snapshot_fixture.json").read_text())
    snapshot = RobotSnapshot.model_validate(raw)
    assert snapshot.gripper.width_m == 0.025
    assert snapshot.arm.model_joint_positions_rad[3] == -1.5707963268
    assert RobotSnapshot.model_validate_json(snapshot.model_dump_json()) == snapshot

