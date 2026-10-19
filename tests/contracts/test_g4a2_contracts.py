from __future__ import annotations

import pytest
from pydantic import ValidationError

from mobilecar.contracts.g4a2 import (
    G4A2ActionRequest,
    HumanIntentRequest,
    HumanSessionRequest,
)


def test_human_session_requires_explicit_confirmation():
    with pytest.raises(ValidationError):
        HumanSessionRequest(
            run_id="run",
            episode_id="episode",
            backend_generation=1,
            session_id="session",
            operator_id="operator",
            operator_kind="human_teleop",
            source="human_teleop",
            view_source="link4_rgbd_proprio",
            explicit_human_confirmation=False,
        )


def test_human_intent_and_ordinary_action_forbid_hidden_plan():
    request = G4A2ActionRequest(
        run_id="run",
        episode_id="episode",
        backend_generation=1,
        nearfield_epoch=1,
        permission_id="permission",
        session_id="session",
        human_session_id="human_session",
        human_intent_id="human_intent",
        action_frame_id="frame",
        action_seq=1,
        observation_sample_id="rgbd",
        display_sample_id="rgbd",
        assist_mode="manual",
        control_generator="human_sixdim_sender",
        joint_order=[f"joint{i}" for i in range(1, 6)],
        units=["rad", "rad", "rad", "rad", "rad", "m"],
        arm_user_joint_positions_rad=[0.0] * 5,
        gripper_width_m=0.025,
    )
    assert request.teacher_plan_id is None
    with pytest.raises(ValidationError):
        G4A2ActionRequest(**{**request.model_dump(), "teacher_plan_id": "hidden"})


def test_human_intent_keeps_client_clocks_separate():
    request = HumanIntentRequest(
        human_session_id="human_session",
        run_id="run",
        episode_id="episode",
        backend_generation=1,
        session_id="session",
        display_sample_id="rgbd",
        input_type="slider",
        assist_mode="manual",
        control_generator="human_sixdim_sender",
        client_display_monotonic_ns=100,
        client_intent_monotonic_ns=200,
        raw_input={"q1": 0.0},
    )
    assert request.client_display_monotonic_ns == 100
    assert request.client_intent_monotonic_ns == 200
