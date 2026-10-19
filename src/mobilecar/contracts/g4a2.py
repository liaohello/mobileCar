"""G4A-2 human intent and ordinary six-dimensional action contracts."""

from __future__ import annotations

import math
from typing import Any, Literal

from pydantic import Field, model_validator

from .models import StrictModel


G4A2_INTERFACE_PROFILE = "g4a2-1.0"


class HumanSessionRequest(StrictModel):
    interface_profile: Literal["g4a2-1.0"] = G4A2_INTERFACE_PROFILE
    run_id: str
    episode_id: str
    backend_generation: int = Field(ge=1)
    session_id: str
    operator_id: str
    operator_kind: Literal["human_teleop"]
    source: Literal["human_teleop"]
    view_source: Literal["link4_rgbd_proprio"]
    explicit_human_confirmation: bool
    input_device: str = "server_browser"
    assist_mode: Literal["manual", "planner_assist"] = "manual"

    @model_validator(mode="after")
    def require_confirmation(self) -> "HumanSessionRequest":
        if not self.explicit_human_confirmation:
            raise ValueError("explicit human confirmation is required")
        return self


class HumanSessionReceipt(StrictModel):
    interface_profile: Literal["g4a2-1.0"] = G4A2_INTERFACE_PROFILE
    accepted: bool
    reason: str = ""
    human_session_id: str | None = None
    run_id: str
    episode_id: str
    backend_generation: int
    session_id: str
    operator_id: str
    received_backend_monotonic_s: float


class HumanIntentRequest(StrictModel):
    interface_profile: Literal["g4a2-1.0"] = G4A2_INTERFACE_PROFILE
    human_session_id: str
    run_id: str
    episode_id: str
    backend_generation: int
    session_id: str
    display_sample_id: str
    input_type: str
    assist_mode: Literal["manual", "planner_assist"]
    control_generator: str
    client_display_monotonic_ns: int | None = None
    client_intent_monotonic_ns: int | None = None
    raw_input: dict[str, Any] = Field(default_factory=dict)


class HumanIntentReceipt(StrictModel):
    interface_profile: Literal["g4a2-1.0"] = G4A2_INTERFACE_PROFILE
    accepted: bool
    reason: str = ""
    human_intent_id: str | None = None
    human_event_seq: int = 0
    human_session_id: str
    display_sample_id: str
    server_receive_backend_monotonic_s: float
    client_display_monotonic_ns: int | None = None
    client_intent_monotonic_ns: int | None = None


class G4A2ActionRequest(StrictModel):
    interface_profile: Literal["g4a2-1.0"] = G4A2_INTERFACE_PROFILE
    run_id: str
    episode_id: str
    backend_generation: int
    nearfield_epoch: int
    permission_id: str
    session_id: str
    human_session_id: str
    human_intent_id: str
    action_frame_id: str
    action_seq: int = Field(ge=1)
    observation_sample_id: str
    display_sample_id: str
    assist_mode: Literal["manual", "planner_assist"]
    control_generator: str
    joint_order: list[str]
    units: list[str]
    arm_user_joint_positions_rad: list[float]
    gripper_width_m: float
    teacher_plan_id: None = None

    @model_validator(mode="after")
    def validate_target(self) -> "G4A2ActionRequest":
        if self.joint_order != [f"joint{i}" for i in range(1, 6)]:
            raise ValueError("joint_order must be joint1..joint5")
        if self.units != ["rad", "rad", "rad", "rad", "rad", "m"]:
            raise ValueError("units must be rad/rad/rad/rad/rad/m")
        if len(self.arm_user_joint_positions_rad) != 5:
            raise ValueError("five arm targets are required")
        if not all(
            math.isfinite(float(value))
            for value in [*self.arm_user_joint_positions_rad, self.gripper_width_m]
        ):
            raise ValueError("six-dimensional target must be finite")
        return self


class G4A2ActionReceipt(StrictModel):
    interface_profile: Literal["g4a2-1.0"] = G4A2_INTERFACE_PROFILE
    accepted: bool
    reason: str = ""
    human_intent_id: str
    action_frame_id: str
    action_seq: int
    human_session_id: str
    requested_backend_monotonic_s: float
    decision_backend_monotonic_s: float
    dispatch_backend_monotonic_s: float | None = None
    control_tick_id: str | None = None
    target: list[float] | None = None
