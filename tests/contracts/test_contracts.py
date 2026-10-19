import json
from pathlib import Path

import pytest
from pydantic import ValidationError

from mobilecar.contracts.models import (
    Capabilities,
    CommandRequest,
    CommandStatus,
    CommandType,
    EventRecord,
    OperatorKind,
    RecordingStatus,
    RobotSnapshot,
    SceneManifest,
    SensorObservation,
)


ROOT = Path(__file__).resolve().parents[2]


def test_command_fixture_round_trip():
    raw = json.loads((ROOT / "configs/examples/command_set_base_twist.json").read_text())
    request = CommandRequest.model_validate(raw)
    assert request.command_type == CommandType.SET_BASE_TWIST
    assert request.operator_kind == OperatorKind.AGENT_UI
    assert request.payload["vx_m_s"] == 0.03
    assert CommandRequest.model_validate_json(request.model_dump_json()) == request


def test_missing_base_channel_is_not_zero_filled():
    raw = json.loads((ROOT / "configs/examples/command_set_base_twist.json").read_text())
    raw["payload"].pop("vy_m_s")
    with pytest.raises(ValidationError, match="missing payload fields"):
        CommandRequest.model_validate(raw)


def test_arm_names_and_order_are_fixed():
    raw = json.loads((ROOT / "configs/examples/command_set_base_twist.json").read_text())
    raw["command_type"] = "set_arm_joint_goal"
    raw["payload"] = {"joint_names": ["joint1", "joint2", "joint3", "joint4"], "joint_positions_rad": [0] * 4}
    with pytest.raises(ValidationError):
        CommandRequest.model_validate(raw)


def test_reset_requires_confirmation():
    raw = json.loads((ROOT / "configs/examples/command_set_base_twist.json").read_text())
    raw["command_type"] = "reset"
    raw["payload"] = {}
    with pytest.raises(ValidationError, match="confirm"):
        CommandRequest.model_validate(raw)


def test_public_status_values_preserve_dispatch_semantics():
    expected = {
        "RECEIVED",
        "REJECTED",
        "EXPIRED_BEFORE_DISPATCH",
        "DISPATCHED",
        "APPLIED",
        "REACHED",
        "CANCELLED",
        "FAILED",
        "LEASE_EXPIRED",
    }
    assert {item.value for item in CommandStatus} == expected
    assert RecordingStatus.DRAINING.value == "DRAINING"


def test_shared_snapshot_and_scene_fixtures_parse():
    snapshot = RobotSnapshot.model_validate_json(
        (ROOT / "configs/examples/snapshot_fixture.json").read_text()
    )
    scene = SceneManifest.model_validate_json(
        (ROOT / "configs/examples/scene_fixture.json").read_text()
    )
    capabilities = Capabilities.model_validate_json(
        (ROOT / "configs/examples/capabilities_fixture.json").read_text()
    )
    assert snapshot.arm.user_joint_positions_rad == [0.0] * 5
    assert scene.camera_ids == ["rgbd"]
    assert scene.model_sha256 == capabilities.model_sha256


def test_shared_event_and_observation_fixtures_parse():
    event = EventRecord.model_validate_json(
        (ROOT / "configs/examples/event_fixture.json").read_text()
    )
    observation = SensorObservation.model_validate_json(
        (ROOT / "configs/examples/sensor_observation_fixture.json").read_text()
    )
    assert event.event_type == "requested"
    assert observation.sensor_kind == "imu"
