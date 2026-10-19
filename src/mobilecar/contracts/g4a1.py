"""G4A-1 near-field permission, observation and six-dimensional action contract."""

from __future__ import annotations

import math
from typing import Any, Literal

from pydantic import Field, model_validator

from .models import StrictModel


G4A1_INTERFACE_PROFILE = "g4a1-1.0"


class NearfieldPermissionRequest(StrictModel):
    interface_profile: Literal["g4a1-1.0"] = G4A1_INTERFACE_PROFILE
    run_id: str
    episode_id: str
    backend_generation: int = Field(ge=1)
    session_id: str
    operator_id: str
    operator_kind: str
    source: str
    view_source: Literal["link4_rgbd_proprio"] = "link4_rgbd_proprio"
    observation_profile_id: str
    observation_profile_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    action_profile_id: str
    action_profile_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")


class NearfieldPermissionReceipt(StrictModel):
    interface_profile: Literal["g4a1-1.0"] = G4A1_INTERFACE_PROFILE
    accepted: bool
    reason: str = ""
    permission_id: str | None = None
    run_id: str
    episode_id: str
    backend_generation: int
    nearfield_epoch: int
    session_id: str
    observation_profile_sha256: str | None = None
    action_profile_sha256: str | None = None
    requested_backend_monotonic_s: float
    granted_backend_monotonic_s: float | None = None
    expires_backend_monotonic_s: float | None = None


class G4A1ActionRequest(StrictModel):
    interface_profile: Literal["g4a1-1.0"] = G4A1_INTERFACE_PROFILE
    run_id: str
    episode_id: str
    backend_generation: int = Field(ge=1)
    nearfield_epoch: int = Field(ge=1)
    permission_id: str
    session_id: str
    action_frame_id: str
    action_seq: int = Field(ge=1)
    observation_sample_id: str
    observation_capture_control_tick_id: str
    observation_arrival_backend_monotonic_s: float
    decision_backend_monotonic_s: float
    issued_backend_monotonic_s: float
    expires_backend_monotonic_s: float
    observation_profile_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    action_profile_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    source: Literal["agent_ui", "scripted", "test_injection", "human_teleop"]
    teacher_plan_id: str | None = None
    joint_order: list[str]
    units: list[str]
    arm_user_joint_positions_rad: list[float]
    gripper_width_m: float

    @model_validator(mode="after")
    def validate_dimensions(self) -> "G4A1ActionRequest":
        if self.joint_order != [f"joint{i}" for i in range(1, 6)]:
            raise ValueError("joint_order must be joint1..joint5")
        expected = ["rad", "rad", "rad", "rad", "rad", "m"]
        if self.units != expected:
            raise ValueError(f"units must be {expected}")
        if len(self.arm_user_joint_positions_rad) != 5:
            raise ValueError("arm_user_joint_positions_rad must contain five values")
        values = [*self.arm_user_joint_positions_rad, self.gripper_width_m]
        if not all(
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(float(value))
            for value in values
        ):
            raise ValueError("six-dimensional action targets must be finite")
        clocks = (
            self.observation_arrival_backend_monotonic_s,
            self.decision_backend_monotonic_s,
            self.issued_backend_monotonic_s,
            self.expires_backend_monotonic_s,
        )
        if not all(math.isfinite(value) for value in clocks):
            raise ValueError("action clocks must be finite")
        if not (
            self.observation_arrival_backend_monotonic_s
            <= self.decision_backend_monotonic_s
            <= self.issued_backend_monotonic_s
            < self.expires_backend_monotonic_s
        ):
            raise ValueError("action causal clocks are out of order")
        return self

    @property
    def target(self) -> list[float]:
        return [*self.arm_user_joint_positions_rad, self.gripper_width_m]


class G4A1ActionReceipt(StrictModel):
    interface_profile: Literal["g4a1-1.0"] = G4A1_INTERFACE_PROFILE
    accepted: bool
    reason: str = ""
    action_frame_id: str
    action_seq: int
    permission_id: str
    run_id: str
    episode_id: str
    backend_generation: int
    nearfield_epoch: int
    session_id: str
    observation_sample_id: str
    requested_backend_monotonic_s: float
    approved_backend_monotonic_s: float | None = None
    dispatch_backend_monotonic_s: float | None = None
    actuator_reference_backend_monotonic_s: float | None = None
    control_tick_id: str | None = None
    target: list[float] | None = None


class G4A1Observation(StrictModel):
    interface_profile: Literal["g4a1-1.0"] = G4A1_INTERFACE_PROFILE
    run_id: str
    episode_id: str
    backend_generation: int
    nearfield_epoch: int
    sample_id: str
    stream_id: str = "rgbd"
    capture_control_tick_id: str
    capture_sim_time_s: float
    capture_backend_monotonic_s: float
    arrival_backend_monotonic_s: float
    observation_backend_monotonic_s: float
    rgb_uri: str
    depth_uri: str
    valid_mask_uri: str
    rgb_sha256: str
    depth_sha256: str
    valid_mask_sha256: str
    valid_fraction: float
    center_depth_m: float | None = None
    arm_user_joint_positions_rad: list[float]
    arm_user_joint_velocities_rad_s: list[float]
    gripper_width_m: float
    camera_intrinsics: dict[str, Any]
    T_base_camera: list[list[float]]
    T_base_link4: list[list[float]]
    T_base_tcp: list[list[float]]
    observation_profile_sha256: str
    action_profile_sha256: str
    quality: str


class G4A1Status(StrictModel):
    interface_profile: Literal["g4a1-1.0"] = G4A1_INTERFACE_PROFILE
    enabled: bool
    run_id: str
    episode_id: str
    backend_generation: int
    nearfield_epoch: int
    permission_active: bool
    permission_id: str | None = None
    session_id: str | None = None
    stopped: bool
    last_action_seq: int = 0
    faults: list[str] = Field(default_factory=list)
