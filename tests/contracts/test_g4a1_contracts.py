from __future__ import annotations

import pytest
from pydantic import ValidationError

from mobilecar.contracts.g4a1 import G4A1ActionRequest


def _action(**overrides):
    payload = {
        "interface_profile": "g4a1-1.0",
        "run_id": "run_fixture",
        "episode_id": "episode_fixture",
        "backend_generation": 1,
        "nearfield_epoch": 2,
        "permission_id": "permission_fixture",
        "session_id": "session_fixture",
        "action_frame_id": "action_fixture",
        "action_seq": 1,
        "observation_sample_id": "rgbd_fixture",
        "observation_capture_control_tick_id": "tick_fixture",
        "observation_arrival_backend_monotonic_s": 10.0,
        "decision_backend_monotonic_s": 10.1,
        "issued_backend_monotonic_s": 10.2,
        "expires_backend_monotonic_s": 10.6,
        "observation_profile_sha256": "1" * 64,
        "action_profile_sha256": "2" * 64,
        "source": "test_injection",
        "joint_order": [f"joint{i}" for i in range(1, 6)],
        "units": ["rad", "rad", "rad", "rad", "rad", "m"],
        "arm_user_joint_positions_rad": [0.0, 0.0, 0.0, 0.0, 0.0],
        "gripper_width_m": 0.025,
    }
    payload.update(overrides)
    return payload


def test_sixdim_action_round_trip_and_target_order():
    request = G4A1ActionRequest.model_validate(_action())
    assert request.target == [0.0, 0.0, 0.0, 0.0, 0.0, 0.025]


@pytest.mark.parametrize(
    "override",
    [
        {"joint_order": ["joint2", "joint1", "joint3", "joint4", "joint5"]},
        {"units": ["deg", "deg", "deg", "deg", "deg", "m"]},
        {"arm_user_joint_positions_rad": [0.0] * 4},
        {"gripper_width_m": float("nan")},
        {"decision_backend_monotonic_s": 9.9},
        {"expires_backend_monotonic_s": 10.2},
    ],
)
def test_sixdim_action_rejects_malformed_or_stale_binding(override):
    with pytest.raises(ValidationError):
        G4A1ActionRequest.model_validate(_action(**override))
