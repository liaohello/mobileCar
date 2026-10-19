import json
from pathlib import Path

import yaml

from mobilecar.contracts.g2a import (
    ActionProfile,
    G2A_INTERFACE_PROFILE,
    G2A_OBSERVATION_PROFILE_ID,
    ObservationProfile,
    ObservationTimingV1,
    PolicyObservationV1,
)
from mobilecar.contracts.models import SensorObservation


ROOT = Path(__file__).resolve().parents[2]


def test_g2a_profile_is_frozen():
    profile = ObservationProfile.model_validate(
        yaml.safe_load(
            (ROOT / "configs/profiles/g2a_rgbd_5hz.yaml").read_text(encoding="utf-8")
        )
    )
    assert profile.interface_profile == G2A_INTERFACE_PROFILE
    assert profile.profile_id == G2A_OBSERVATION_PROFILE_ID
    assert (profile.image["width"], profile.image["height"]) == (640, 480)
    assert profile.image["nominal_capture_hz"] == 5.0
    assert profile.acceptance["min_average_wall_hz"] == 4.5
    assert profile.acceptance["max_p99_interval_s"] == 0.4
    start, end = profile.acceptance["analysis_window_s"]
    assert end - start == 50.0


def test_g2a_action_profile_is_frozen():
    profile = ActionProfile.model_validate(
        yaml.safe_load(
            (ROOT / "configs/profiles/g2a_action_teleop_v1.yaml").read_text(
                encoding="utf-8"
            )
        )
    )
    assert profile.profile_id == "g2a-teleop-action-1.0"
    assert profile.profile_version == "1.0"
    assert profile.requirements["approved_or_dispatched_required"] is True
    assert profile.requirements["actuator_reference_required"] is True
    assert profile.requirements["raw_observation_presence_does_not_define_action_profile"] is True
    assert "rgb_uri" in profile.observation_requirements["d2"]


def test_timing_and_sensor_observation_are_compatible():
    timing = ObservationTimingV1(
        capture_sim_time_s=12.3,
        capture_backend_monotonic_s=100.0,
        capture_control_tick_id="tick_123",
        feedback_sample_id="joint_123",
        render_complete_backend_monotonic_s=100.1,
        publish_backend_monotonic_s=100.11,
        arrival_backend_monotonic_s=100.12,
    )
    observation = SensorObservation(
        run_id="run",
        episode_id="episode",
        stream_id="rgbd",
        sample_id="frame",
        sensor_kind="rgb",
        measurement_time_s=100.0,
        arrival_time_s=100.12,
        clock_id="backend_monotonic",
        frame_id="rgbd_optical",
        source="fixture",
        quality="valid",
        capture_sim_time_s=timing.capture_sim_time_s,
        capture_backend_monotonic_s=timing.capture_backend_monotonic_s,
        capture_control_tick_id=timing.capture_control_tick_id,
        feedback_sample_id=timing.feedback_sample_id,
        timing=timing.model_dump(mode="json"),
    )
    assert observation.capture_control_tick_id == "tick_123"
    assert "world_sim" not in observation.model_dump_json()


def test_policy_observation_has_no_world_truth_fields():
    observation = PolicyObservationV1(
        run_id="run",
        episode_id="episode",
        stream_id="rgbd",
        sample_id="frame",
        measurement_time_s=1.0,
        clock_id="backend_monotonic",
        frame_id="rgbd_optical",
        base_twist_body_m_s=[0.0, 0.0, 0.0],
        arm_user_joint_positions_rad=[0.0] * 5,
        gripper_width_m=0.025,
        source="mujoco_sensor",
        view_source="third_person_3d",
        quality="valid",
    )
    payload = observation.model_dump(mode="json")
    assert "diagnostics" not in payload
    assert "world_sim" not in json.dumps(payload)
