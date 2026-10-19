import json
import hashlib
from pathlib import Path

import pytest
import yaml
from pydantic import ValidationError

from mobilecar.contracts.navigation import (
    G3A_INTERFACE_PROFILE,
    G3A_NAVIGATION_PROFILE_ID,
    ImuMeasurement,
    LaserReturnStatus,
    LaserScanObservation,
    NavigationClockV1,
    NavigationControlMessage,
)


ROOT = Path(__file__).resolve().parents[2]


def _clock() -> NavigationClockV1:
    return NavigationClockV1(
        run_id="run_fixture",
        episode_id="episode_fixture",
        backend_generation=1,
        navigation_epoch=0,
        control_tick_id="tick_000000001",
        capture_sim_time_s=1.0,
        capture_backend_monotonic_s=100.0,
    )


def test_g3a_profile_is_frozen():
    profile = yaml.safe_load(
        (ROOT / "configs/profiles/g3a_navigation_v1.yaml").read_text(
            encoding="utf-8"
        )
    )
    assert profile["interface_profile"] == G3A_INTERFACE_PROFILE
    assert profile["profile_id"] == G3A_NAVIGATION_PROFILE_ID
    assert profile["sensors"]["wheel"]["nominal_hz"] == 50.0
    assert profile["sensors"]["imu"]["nominal_hz"] == 50.0
    assert profile["sensors"]["scan"]["nominal_hz"] == 10.0
    assert profile["sensors"]["scan"]["beam_count"] == 360
    assert profile["sensors"]["scan"]["time_increment_s"] == 0.0
    assert profile["estimate"]["source"] == "wheel_gyro_planar_fusion"
    assert profile["control"]["source"] == "ros_navigation_test"
    assert profile["acceptance"]["stationary_planar_drift_m"] == 0.01
    assert profile["acceptance"]["map_publication"] == "forbidden"
    model = ROOT / "assets/landerpi/model/robot_navigation_diagnostic.xml"
    assert hashlib.sha256(model.read_bytes()).hexdigest() == profile["model_sha256"]


def test_laserscan_requires_ranges_only_for_hits():
    scan = LaserScanObservation(
        profile_id=G3A_NAVIGATION_PROFILE_ID,
        profile_version="1.0",
        profile_sha256="0" * 64,
        clock=_clock(),
        scan_id="scan_1",
        angle_min_rad=0.0,
        angle_max_rad=0.3,
        angle_increment_rad=0.1,
        range_min_m=0.1,
        range_max_m=8.0,
        ranges_m=[1.0, None, None, None],
        statuses=[
            LaserReturnStatus.HIT,
            LaserReturnStatus.NO_RETURN,
            LaserReturnStatus.INVALID,
            LaserReturnStatus.SELF_OCCLUDED,
        ],
        beam_count=4,
        valid_count=1,
        status_counts={
            "HIT": 1,
            "NO_RETURN": 1,
            "INVALID": 1,
            "SELF_OCCLUDED": 1,
        },
    )
    assert scan.ranges_m[0] == 1.0
    assert scan.ranges_m[1:] == [None, None, None]

    invalid_payload = scan.model_dump(mode="json")
    invalid_payload["ranges_m"] = [1.0, 4.0, None, None]
    with pytest.raises(ValidationError):
        LaserScanObservation(**invalid_payload)


def test_imu_missing_payload_is_explicitly_invalid():
    valid = ImuMeasurement(
        sample_id="imu_1",
        clock=_clock(),
        rotation_imu_to_base_wxyz=[1.0, 0.0, 0.0, 0.0],
        angular_velocity_rad_s=[0.0, 0.0, 0.0],
        linear_acceleration_m_s2=[0.0, 0.0, 9.81],
        gyro_valid=True,
        acceleration_valid=True,
        quality="simulated_nominal",
        source="mujoco_navigation_imu_sensor",
    )
    assert valid.angular_velocity_rad_s == [0.0, 0.0, 0.0]

    invalid = valid.model_copy(
        update={
            "angular_velocity_rad_s": None,
            "gyro_valid": False,
            "quality": "sensor_sample_failed",
        }
    )
    assert invalid.gyro_valid is False
    assert invalid.angular_velocity_rad_s is None

    with pytest.raises(ValidationError):
        ImuMeasurement(
            **{
                **valid.model_dump(mode="json"),
                "angular_velocity_rad_s": None,
                "gyro_valid": True,
            }
        )


def test_navigation_control_has_one_source_and_finite_velocity():
    message = NavigationControlMessage(
        message_type="cmd_vel",
        run_id="run",
        episode_id="episode",
        backend_generation=1,
        navigation_epoch=2,
        session_id="ros_nav_test",
        vx_m_s=0.02,
        vy_m_s=-0.01,
        wz_rad_s=0.1,
    )
    payload = message.model_dump(mode="json")
    assert payload["source"] == "ros_navigation_test"
    assert "world_sim" not in json.dumps(payload)

    with pytest.raises(ValidationError):
        NavigationControlMessage(
            **{**payload, "vx_m_s": float("nan")}
        )
    with pytest.raises(ValidationError):
        NavigationControlMessage(
            **{
                **payload,
                "message_type": "release",
                "vx_m_s": 0.01,
            }
        )
