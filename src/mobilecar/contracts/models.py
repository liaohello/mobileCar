"""Canonical mobilecar-1.0 message and snapshot models.

All public SI fields use metres, radians and seconds. MuJoCo's internal wxyz
quaternion order is never exposed here; public quaternions are xyzw.
"""

from __future__ import annotations

import math
from enum import Enum
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid", use_enum_values=True)


class OperatorKind(str, Enum):
    HUMAN_TELEOP = "human_teleop"
    AGENT_UI = "agent_ui"
    SCRIPTED = "scripted"
    AUTONOMOUS_POLICY = "autonomous_policy"
    SYNTHETIC_FIXTURE = "synthetic_fixture"


class CommandType(str, Enum):
    SET_BASE_TWIST = "set_base_twist"
    ZERO_BASE = "zero_base"
    SET_ARM_JOINT_GOAL = "set_arm_joint_goal"
    SET_GRIPPER_WIDTH_GOAL = "set_gripper_width_goal"
    PREVIEW_TCP_TARGET = "preview_tcp_target"
    EXECUTE_TCP_TARGET = "execute_tcp_target"
    CANCEL = "cancel"
    STOP = "stop"
    RESET = "reset"
    HEARTBEAT = "heartbeat"
    SET_TASK_PHASE = "set_task_phase"
    START_RECORDING = "start_recording"
    STOP_RECORDING = "stop_recording"


class CommandStatus(str, Enum):
    RECEIVED = "RECEIVED"
    REJECTED = "REJECTED"
    EXPIRED_BEFORE_DISPATCH = "EXPIRED_BEFORE_DISPATCH"
    DISPATCHED = "DISPATCHED"
    APPLIED = "APPLIED"
    REACHED = "REACHED"
    CANCELLED = "CANCELLED"
    FAILED = "FAILED"
    LEASE_EXPIRED = "LEASE_EXPIRED"


class RecordingStatus(str, Enum):
    IDLE = "IDLE"
    STARTING = "STARTING"
    RECORDING = "RECORDING"
    DRAINING = "DRAINING"
    COMPLETE = "COMPLETE"
    FAILED = "FAILED"


class AuditStatus(str, Enum):
    PENDING = "PENDING"
    COMPLETE = "COMPLETE"
    INCOMPLETE = "INCOMPLETE"
    FAILED = "FAILED"


class TaskResult(str, Enum):
    SUCCESS = "success"
    FAILURE = "failure"
    PARTIAL = "partial"
    CANCELLED = "cancelled"
    NOT_APPLICABLE = "not_applicable"
    UNKNOWN = "unknown"


class TaskPhase(str, Enum):
    SEARCH = "SEARCH"
    NAVIGATE = "NAVIGATE"
    OBSERVE = "OBSERVE"
    APPROACH = "APPROACH"
    MANIPULATE = "MANIPULATE"
    VERIFY = "VERIFY"


class TerminationReason(str, Enum):
    USER_STOP = "USER_STOP"
    OPERATOR_FINISHED = "OPERATOR_FINISHED"
    RECORDING_FAILED = "RECORDING_FAILED"
    LEASE_EXPIRED = "LEASE_EXPIRED"
    ACTION_TIMEOUT = "ACTION_TIMEOUT"
    EPISODE_TIMEOUT = "EPISODE_TIMEOUT"
    RESET = "RESET"
    BACKEND_CLOSED = "BACKEND_CLOSED"
    UNKNOWN = "UNKNOWN"


class SensorKind(str, Enum):
    RGB = "rgb"
    DEPTH = "depth"
    LIDAR = "lidar"
    IMU = "imu"
    ENCODERS = "encoders"
    JOINT_FEEDBACK = "joint_feedback"
    IMAGE_TRANSFORMS = "image_transforms"


class ControlChannel(str, Enum):
    BASE = "base"
    ARM = "arm"


class CommandRequest(StrictModel):
    contract_version: Literal["mobilecar-1.0"] = "mobilecar-1.0"
    run_id: str = Field(min_length=1)
    backend_generation: int = Field(ge=1)
    episode_id: str = Field(min_length=1)
    session_id: str = Field(min_length=1)
    operator_id: str = Field(min_length=1)
    operator_kind: OperatorKind
    view_source: str = Field(default="third_person_3d", min_length=1)
    command_id: str = Field(min_length=1)
    command_seq: int = Field(ge=1)
    command_type: CommandType
    requested_ttl_s: float = Field(gt=0, le=30)
    payload: dict[str, Any] = Field(default_factory=dict)

    @model_validator(mode="after")
    def validate_payload(self) -> "CommandRequest":
        payload = self.payload
        command = CommandType(self.command_type)
        if command == CommandType.SET_BASE_TWIST:
            _require_keys(payload, {"frame_id", "vx_m_s", "vy_m_s", "wz_rad_s"})
            if payload["frame_id"] != "base_link":
                raise ValueError("base twist frame_id must be base_link")
            _finite_vector(payload, ("vx_m_s", "vy_m_s", "wz_rad_s"))
        elif command == CommandType.SET_ARM_JOINT_GOAL:
            _require_keys(payload, {"joint_names", "joint_positions_rad"})
            names = payload["joint_names"]
            values = payload["joint_positions_rad"]
            if list(names) != [f"joint{i}" for i in range(1, 6)]:
                raise ValueError("joint_names must be joint1..joint5 in order")
            if not isinstance(values, list) or len(values) != 5:
                raise ValueError("joint_positions_rad must contain five values")
            if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in values):
                raise ValueError("joint_positions_rad must be finite")
        elif command == CommandType.SET_GRIPPER_WIDTH_GOAL:
            _require_keys(payload, {"width_m"})
            _finite_vector(payload, ("width_m",))
        elif command in (CommandType.PREVIEW_TCP_TARGET, CommandType.EXECUTE_TCP_TARGET):
            _require_keys(payload, {"frame_id", "position_m"})
            if payload["frame_id"] not in {"world_sim", "base_link", "arm_base"}:
                raise ValueError("unsupported TCP target frame")
            _vec3(payload["position_m"], "position_m")
            if "orientation_xyzw" in payload and payload["orientation_xyzw"] is not None:
                values = payload["orientation_xyzw"]
                if not isinstance(values, list) or len(values) != 4:
                    raise ValueError("orientation_xyzw must contain four values")
                if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in values):
                    raise ValueError("orientation_xyzw must be finite")
                if abs(sum(float(v) ** 2 for v in values) - 1.0) > 1e-3:
                    raise ValueError("orientation_xyzw must be normalized")
            if command == CommandType.EXECUTE_TCP_TARGET:
                _require_keys(payload, {"plan_id"})
        elif command == CommandType.CANCEL:
            _require_keys(payload, {"reason"})
        elif command == CommandType.STOP:
            payload.setdefault("latch", True)
        elif command == CommandType.RESET:
            if payload.get("confirm") is not True:
                raise ValueError("reset requires payload.confirm=true")
        elif command == CommandType.SET_TASK_PHASE:
            _require_keys(payload, {"phase"})
            TaskPhase(payload["phase"])
        elif command in (CommandType.HEARTBEAT, CommandType.ZERO_BASE):
            pass
        return self


class SafetyDecision(StrictModel):
    approved: bool
    safety_revision: str = "g1-safety-1"
    reason: str = ""
    clipped: dict[str, Any] = Field(default_factory=dict)
    effective_payload: dict[str, Any] = Field(default_factory=dict)
    allowed_contacts: list[str] = Field(default_factory=list)


class PlanPreview(StrictModel):
    plan_id: str
    command_id: str
    target_revision: int = Field(ge=1)
    frame_id: str
    position_m: list[float]
    orientation_xyzw: list[float] | None = None
    waypoints_model_rad: list[list[float]]
    times_s: list[float]
    predicted_position_error_m: float = Field(ge=0)
    predicted_orientation_error_rad: float = Field(ge=0)
    joint_margin_rad: list[float]
    collision_free: bool
    collision_details: list[str] = Field(default_factory=list)
    created_backend_monotonic_s: float
    expires_backend_monotonic_s: float
    status: str = "PREVIEWED"


class CommandResponse(StrictModel):
    contract_version: Literal["mobilecar-1.0"] = "mobilecar-1.0"
    command_id: str
    command_seq: int
    status: CommandStatus
    reason: str = ""
    backend_monotonic_s: float
    sim_time_s: float
    decision: SafetyDecision | None = None
    plan: PlanPreview | None = None
    control_tick_id: str | None = None


class Capabilities(StrictModel):
    contract_version: Literal["mobilecar-1.0"] = "mobilecar-1.0"
    backend_kind: str
    arm_joint_names: list[str]
    arm_user_limits_rad: list[list[float]]
    arm_velocity_limits_rad_s: list[float]
    gripper_min_width_m: float
    gripper_max_width_m: float
    base_linear_limit_m_s: float
    base_yaw_limit_rad_s: float
    tcp_position_tolerance_m: float
    tcp_orientation_is_soft: bool = True
    supported_commands: list[CommandType]
    sensor_streams: list[SensorKind]
    model_id: str
    model_sha256: str
    scene_id: str
    camera_parent_body: Literal["link4"] = "link4"
    tcp_parent_body: Literal["guide_rod_link"] = "guide_rod_link"
    ground_truth_policy_visible: bool = False
    g4a2_postures: list[dict[str, Any]] = Field(default_factory=list)


class BaseState(StrictModel):
    frame_id: Literal["base_link"] = "base_link"
    position_world_sim_m: list[float]
    orientation_world_sim_xyzw: list[float]
    vx_m_s: float
    vy_m_s: float
    wz_rad_s: float
    requested_vx_m_s: float = 0.0
    requested_vy_m_s: float = 0.0
    requested_wz_rad_s: float = 0.0
    stopped: bool = False
    stop_latched: bool = False


class ArmState(StrictModel):
    user_joint_positions_rad: list[float]
    user_joint_targets_rad: list[float]
    model_joint_positions_rad: list[float]
    model_joint_targets_rad: list[float]
    joint_velocities_rad_s: list[float]
    joint_torques_nm: list[float]
    joint_margin_rad: list[float]
    model_limits_rad: list[list[float]]
    coordinates_status: str


class GripperState(StrictModel):
    width_m: float
    target_width_m: float
    model_drive_position_rad: float
    model_drive_target_rad: float
    left_contact_force_n: float = 0.0
    right_contact_force_n: float = 0.0
    bilateral_contact: bool = False


class TCPState(StrictModel):
    frame_id: Literal["world_sim"] = "world_sim"
    position_m: list[float]
    orientation_xyzw: list[float]
    position_error_m: float | None = None
    orientation_error_rad: float | None = None
    held: bool = False


class SensorState(StrictModel):
    kind: SensorKind
    available: bool
    fresh: bool
    sample_id: str | None = None
    stream_id: str | None = None
    measurement_time_s: float | None = None
    arrival_time_s: float | None = None
    clock_id: str = "backend_monotonic"
    valid_fraction: float | None = None
    asset_uri: str | None = None
    quality: str = "unknown"
    capture_sim_time_s: float | None = None
    capture_backend_monotonic_s: float | None = None
    capture_control_tick_id: str | None = None
    feedback_sample_id: str | None = None
    observation_age_s: float | None = None
    profile_id: str | None = None


class RecordingState(StrictModel):
    status: RecordingStatus = RecordingStatus.IDLE
    episode_id: str | None = None
    recording_segment_id: str | None = None
    command_capture_ready: bool = False
    queued_events: int = 0
    dropped_events: int = 0
    error: str | None = None


class RobotSnapshot(StrictModel):
    contract_version: Literal["mobilecar-1.0"] = "mobilecar-1.0"
    run_id: str
    backend_generation: int
    episode_id: str
    episode_started_backend_monotonic_s: float
    backend_monotonic_s: float
    sim_time_s: float
    feedback_time_s: float
    wall_utc: str
    clock_id: str = "backend_monotonic"
    control_tick_id: str
    task_phase: TaskPhase = TaskPhase.OBSERVE
    base: BaseState
    arm: ArmState
    gripper: GripperState
    tcp: TCPState
    sensors: list[SensorState]
    plan: PlanPreview | None = None
    active_session_id: str | None = None
    active_operator_id: str | None = None
    active_operator_kind: OperatorKind | None = None
    base_owner_session_id: str | None = None
    arm_owner_session_id: str | None = None
    safety_latched: bool = False
    recording: RecordingState = Field(default_factory=RecordingState)
    contact_summary: dict[str, Any] = Field(default_factory=dict)
    diagnostics: dict[str, Any] = Field(default_factory=dict)
    model_joint_positions_rad: dict[str, float] = Field(default_factory=dict)


class SensorObservation(StrictModel):
    contract_version: Literal["mobilecar-1.0"] = "mobilecar-1.0"
    run_id: str
    episode_id: str
    stream_id: str
    sample_id: str
    sensor_kind: SensorKind
    measurement_time_s: float
    arrival_time_s: float
    clock_id: str
    frame_id: str
    source: str
    quality: str
    value: Any = None
    asset_uri: str | None = None
    valid_mask_uri: str | None = None
    metadata: dict[str, Any] = Field(default_factory=dict)
    capture_sim_time_s: float | None = None
    capture_backend_monotonic_s: float | None = None
    capture_control_tick_id: str | None = None
    feedback_sample_id: str | None = None
    timing: Any = None


class EventRecord(StrictModel):
    contract_version: Literal["mobilecar-1.0"] = "mobilecar-1.0"
    run_id: str
    episode_id: str
    event_seq: int | None = Field(default=None, ge=1)
    event_type: str = Field(min_length=1)
    wall_utc: str
    backend_monotonic_s: float | None = None
    sim_time_s: float | None = None
    command_id: str | None = None
    control_tick_id: str | None = None
    plan_id: str | None = None
    task_phase: TaskPhase | None = None
    payload: dict[str, Any] = Field(default_factory=dict)


class EpisodeStartRequest(StrictModel):
    task_language: str = Field(default="Drive to the cube, grasp it, lift at least 50 mm and hold for 2 s.")
    operator_id: str
    operator_kind: OperatorKind
    view_source: str = "third_person_3d"
    map_mode: str = "unavailable"
    require_recording: bool = False


class EpisodeControlResponse(StrictModel):
    accepted: bool
    reason: str = ""
    episode_id: str | None = None
    recording_segment_id: str | None = None
    recording_status: RecordingStatus = RecordingStatus.IDLE


class ValidationReport(StrictModel):
    contract_version: Literal["mobilecar-1.0"] = "mobilecar-1.0"
    episode_id: str
    audit_status: AuditStatus
    passed: bool
    errors: list[str] = Field(default_factory=list)
    warnings: list[str] = Field(default_factory=list)
    counts: dict[str, int] = Field(default_factory=dict)
    actual_rates_hz: dict[str, float] = Field(default_factory=dict)
    max_gap_s: dict[str, float] = Field(default_factory=dict)
    coverage: dict[str, float] = Field(default_factory=dict)
    profile_id: str | None = None
    profile_version: str | None = None
    profile_sha256: str | None = None
    timing: Any = None
    gates: dict[str, bool] = Field(default_factory=dict)
    gate_reasons: list[str] = Field(default_factory=list)
    generated_wall_utc: str


class TaskAssessment(StrictModel):
    contract_version: Literal["mobilecar-1.0"] = "mobilecar-1.0"
    episode_id: str
    task_result: TaskResult
    oracle_source: str
    lift_height_m: float | None = None
    hold_duration_s: float | None = None
    bilateral_contact_fraction: float | None = None
    evidence: dict[str, Any] = Field(default_factory=dict)
    human_rating: int | None = Field(default=None, ge=1, le=5)
    human_notes: str | None = None


class DataEligibility(StrictModel):
    contract_version: Literal["mobilecar-1.0"] = "mobilecar-1.0"
    episode_id: str
    audit_status: AuditStatus
    task_result: TaskResult
    d0_navigation: bool = False
    d1_language_navigation: bool = False
    d2_near_field_manipulation: bool = False
    d3_combined_task: bool = False
    reasons: list[str] = Field(default_factory=list)
    source_operator_kind: OperatorKind | None = None
    view_source: str | None = None
    wrist_rgb_only_eligible: bool = False
    profile_id: str | None = None
    profile_version: str | None = None
    profile_sha256: str | None = None
    action_profile_id: str | None = None
    action_profile_version: str | None = None
    action_profile_sha256: str | None = None
    gates: dict[str, bool] = Field(default_factory=dict)


class SceneTransform(StrictModel):
    """Rigid, parent-from-child transform in public SI/xyzw conventions."""

    position_m: list[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0], min_length=3, max_length=3)
    orientation_xyzw: list[float] = Field(
        default_factory=lambda: [0.0, 0.0, 0.0, 1.0], min_length=4, max_length=4
    )


class SceneGeom(StrictModel):
    name: str = Field(min_length=1)
    geom_type: Literal[
        "box",
        "sphere",
        "ellipsoid",
        "cylinder",
        "capsule",
        "plane",
        "mesh",
        "heightfield",
        "unknown",
    ]
    size_m: list[float] = Field(default_factory=list)
    local_transform: SceneTransform = Field(default_factory=SceneTransform)
    rgba: list[float] = Field(
        default_factory=lambda: [0.65, 0.68, 0.72, 1.0], min_length=4, max_length=4
    )
    mesh_asset_id: str | None = None
    collision: bool = False
    group: int = 0

    @property
    def geom_id(self) -> str:
        """Compatibility alias used by the renderer-neutral scene contract."""

        return self.name


class SceneLink(StrictModel):
    name: str = Field(min_length=1)
    parent_name: str | None = None
    joint_name: str | None = None
    joint_type: Literal["free", "ball", "slide", "hinge", "fixed", "unknown"] = "fixed"
    joint_axis: list[float] = Field(
        default_factory=lambda: [0.0, 0.0, 1.0], min_length=3, max_length=3
    )
    local_transform: SceneTransform = Field(default_factory=SceneTransform)
    geoms: list[SceneGeom] = Field(default_factory=list)


class MeshAsset(StrictModel):
    asset_id: str = Field(min_length=1)
    uri: str = Field(min_length=1)
    sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    media_type: str = "model/stl"


SceneMeshAsset = MeshAsset


class SceneManifest(StrictModel):
    """Versioned view of the exact compiled MuJoCo scene."""

    contract_version: Literal["mobilecar-1.0"] = "mobilecar-1.0"
    model_id: str = Field(min_length=1)
    model_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    scene_id: str = Field(min_length=1)
    links: list[SceneLink]
    world_links: list[SceneLink] = Field(default_factory=list)
    mesh_assets: list[MeshAsset] = Field(default_factory=list)
    camera_ids: list[str] = Field(default_factory=list)
    tcp_site: str = Field(default="grasp_tcp", min_length=1)
    source: str = "mujoco_model_compile"


def _require_keys(payload: dict[str, Any], keys: set[str]) -> None:
    missing = sorted(keys - payload.keys())
    if missing:
        raise ValueError(f"missing payload fields: {missing}")


def _finite_vector(payload: dict[str, Any], keys: tuple[str, ...]) -> None:
    for key in keys:
        value = payload[key]
        if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
            raise ValueError(f"{key} must be a finite number")


def _vec3(value: Any, name: str) -> None:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{name} must contain three values")
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in value):
        raise ValueError(f"{name} must be finite")
