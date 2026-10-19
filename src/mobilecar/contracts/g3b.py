"""Versioned G3B mapping, localization, Nav2 goal and safety contracts."""

from __future__ import annotations

import math
from enum import Enum
from typing import Literal

from pydantic import Field, model_validator

from .models import StrictModel
from .navigation import NavigationClockV1


G3B_INTERFACE_PROFILE = "g3b-1.0"
G3B_NAVIGATION_PROFILE_ID = "g3b-nav2-omni-map-v1"


class NavigationMode(str, Enum):
    MAPPING = "MAPPING"
    LOCALIZATION = "LOCALIZATION"


class G3BGoalState(str, Enum):
    IDLE = "IDLE"
    VALIDATING = "VALIDATING"
    WAITING_READY = "WAITING_READY"
    AUTHORIZING = "AUTHORIZING"
    ACTIVE = "ACTIVE"
    CANCEL_REQUESTED = "CANCEL_REQUESTED"
    TAKEOVER = "TAKEOVER"
    STOPPED = "STOPPED"
    SUCCEEDED = "SUCCEEDED"
    ABORTED = "ABORTED"
    REJECTED = "REJECTED"


class G3BControlSource(str, Enum):
    NAV2_GOAL = "nav2_goal"


class G3BControlType(str, Enum):
    AUTHORIZE = "authorize"
    CMD_VEL = "cmd_vel"
    REVOKE = "revoke"
    RELEASE = "release"
    STOP = "stop"


class MapAsset(StrictModel):
    role: Literal[
        "occupancy_yaml",
        "occupancy_image",
        "pose_graph",
        "pose_graph_data",
        "map_manifest",
        "rviz_config",
    ]
    relative_path: str = Field(min_length=1)
    sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    bytes: int = Field(ge=1)

    @model_validator(mode="after")
    def validate_path(self) -> "MapAsset":
        parts = self.relative_path.replace("\\", "/").split("/")
        if self.relative_path.startswith("/") or ".." in parts:
            raise ValueError("map asset path must be relative and cannot escape package")
        return self


class MapInitialPrior(StrictModel):
    x_m: float
    y_m: float
    yaw_rad: float
    covariance: list[float]

    @model_validator(mode="after")
    def validate_prior(self) -> "MapInitialPrior":
        if len(self.covariance) != 36:
            raise ValueError("initial prior covariance must contain 36 values")
        if not all(
            math.isfinite(value)
            for value in (
                self.x_m,
                self.y_m,
                self.yaw_rad,
                *self.covariance,
            )
        ):
            raise ValueError("initial prior must contain finite values")
        return self


class MapPackageManifest(StrictModel):
    schema_version: Literal[1] = 1
    interface_profile: Literal["g3b-1.0"] = G3B_INTERFACE_PROFILE
    profile_id: str
    profile_version: str
    profile_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    map_id: str = Field(min_length=1)
    scene_id: str = Field(min_length=1)
    model_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    navigation_profile_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    sensor_profile_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    scan_config_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    map_frame: Literal["map"] = "map"
    odom_frame: Literal["odom"] = "odom"
    base_frame: Literal["base_link"] = "base_link"
    resolution_m: float = Field(gt=0.0)
    origin_xy_yaw: list[float]
    occupied_thresh: float = Field(gt=0.0, lt=1.0)
    free_thresh: float = Field(gt=0.0, lt=1.0)
    created_run_id: str
    created_episode_id: str
    created_navigation_epoch: int = Field(ge=0)
    serialized_pose_graph: Literal[True] = True
    initial_prior: MapInitialPrior | None = None
    assets: list[MapAsset]
    compatibility_fields: list[str] = Field(
        default_factory=lambda: [
            "scene_id",
            "model_sha256",
            "navigation_profile_sha256",
            "sensor_profile_sha256",
            "scan_config_sha256",
            "resolution_m",
            "map_frame",
            "odom_frame",
            "base_frame",
        ]
    )
    created_wall_utc: str | None = None

    @model_validator(mode="after")
    def validate_manifest(self) -> "MapPackageManifest":
        if len(self.origin_xy_yaw) != 3 or not all(
            math.isfinite(value) for value in self.origin_xy_yaw
        ):
            raise ValueError("origin_xy_yaw must contain three finite values")
        if self.free_thresh >= self.occupied_thresh:
            raise ValueError("free_thresh must be below occupied_thresh")
        roles = [asset.role for asset in self.assets]
        required = {
            "occupancy_yaml",
            "occupancy_image",
            "pose_graph",
            "pose_graph_data",
            "map_manifest",
        }
        if not required.issubset(roles):
            raise ValueError("map package is missing occupancy, pose graph or manifest assets")
        if len(roles) != len(set(roles)):
            raise ValueError("map asset roles must be unique")
        return self


class G3BControlMessage(StrictModel):
    schema_version: Literal[2] = 2
    interface_profile: Literal["g3b-1.0"] = G3B_INTERFACE_PROFILE
    message_type: G3BControlType
    source: G3BControlSource = G3BControlSource.NAV2_GOAL
    request_id: str = Field(min_length=1)
    goal_id: str = Field(min_length=1)
    goal_generation: int = Field(ge=1)
    map_id: str = Field(min_length=1)
    run_id: str
    episode_id: str
    backend_generation: int = Field(ge=1)
    navigation_epoch: int = Field(ge=0)
    session_id: str = Field(min_length=1)
    lease_ttl_s: float = Field(default=0.25, gt=0.0, le=2.0)
    vx_m_s: float = 0.0
    vy_m_s: float = 0.0
    wz_rad_s: float = 0.0
    reason: str = ""

    @model_validator(mode="after")
    def validate_control(self) -> "G3BControlMessage":
        values = (self.vx_m_s, self.vy_m_s, self.wz_rad_s)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("G3B velocities must be finite")
        if self.message_type in {
            G3BControlType.AUTHORIZE,
            G3BControlType.REVOKE,
            G3BControlType.RELEASE,
            G3BControlType.STOP,
        } and any(abs(value) > 1e-12 for value in values):
            raise ValueError("non-velocity G3B control must request zero velocity")
        return self


class ControlReceipt(StrictModel):
    schema_version: Literal[2] = 2
    interface_profile: Literal["g3b-1.0"] = G3B_INTERFACE_PROFILE
    message_type: Literal["control_receipt"] = "control_receipt"
    request_id: str
    goal_id: str
    goal_generation: int = Field(ge=1)
    map_id: str
    session_id: str
    run_id: str
    episode_id: str
    backend_generation: int = Field(ge=1)
    navigation_epoch: int = Field(ge=0)
    received: bool
    authorized: bool
    accepted: bool
    backend_status: str
    reason: str = ""
    control_tick_id: str | None = None
    backend_monotonic_s: float | None = None
    sim_time_s: float | None = None
    dispatch_state: Literal[
        "RECEIVED",
        "AUTHORIZED",
        "DISPATCHED",
        "REJECTED",
        "CANCELLED",
        "EXPIRED",
    ]


class LocalizationStatus(str, Enum):
    READY = "READY"
    INITIALIZING = "INITIALIZING"
    STATIONARY_HOLD = "STATIONARY_HOLD"
    STALE = "STALE"
    LOST = "LOST"
    FAILED = "FAILED"
    STOPPED = "STOPPED"


class NavigationSystemHealth(StrictModel):
    schema_version: Literal[1] = 1
    interface_profile: Literal["g3b-1.0"] = G3B_INTERFACE_PROFILE
    profile_id: str
    profile_version: str
    profile_sha256: str
    sequence: int = Field(ge=0)
    clock: NavigationClockV1
    mode: NavigationMode
    status: LocalizationStatus
    stop_required: bool
    command_gate_open: bool
    scan_fresh: bool
    odom_fresh: bool
    localization_node_alive: bool
    map_to_odom_fresh: bool
    pose_match_fresh: bool
    nav2_active: bool
    scan_age_s: float | None = None
    odom_age_s: float | None = None
    localization_update_age_s: float | None = None
    map_to_odom_age_s: float | None = None
    detection_latency_s: float | None = None
    reasons: list[str] = Field(default_factory=list)

    @model_validator(mode="after")
    def validate_health(self) -> "NavigationSystemHealth":
        ages = (
            self.scan_age_s,
            self.odom_age_s,
            self.localization_update_age_s,
            self.map_to_odom_age_s,
            self.detection_latency_s,
        )
        if any(age is not None and age < 0.0 for age in ages):
            raise ValueError("health ages must be nonnegative")
        if self.stop_required and self.command_gate_open:
            raise ValueError("stop-required health cannot keep the speed gate open")
        return self


class GoalSupervisorStatus(StrictModel):
    schema_version: Literal[1] = 1
    interface_profile: Literal["g3b-1.0"] = G3B_INTERFACE_PROFILE
    goal_id: str | None = None
    goal_generation: int = Field(default=0, ge=0)
    map_id: str | None = None
    session_id: str | None = None
    state: G3BGoalState = G3BGoalState.IDLE
    action_accepted: bool = False
    backend_authorized: bool = False
    command_gate_open: bool = False
    active_action_name: str | None = None
    reasons: list[str] = Field(default_factory=list)
    updated_sim_time_s: float | None = None
    updated_backend_monotonic_s: float | None = None
