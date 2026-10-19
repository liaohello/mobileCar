"""Versioned G3A navigation measurement, estimate and health contracts."""

from __future__ import annotations

import math
from enum import Enum
from typing import Literal

from pydantic import Field, model_validator

from .models import StrictModel


G3A_INTERFACE_PROFILE = "g3a-1.0"
G3A_NAVIGATION_PROFILE_ID = "g3a-nav-50hz-laser10-v1"


class LaserReturnStatus(str, Enum):
    HIT = "HIT"
    NO_RETURN = "NO_RETURN"
    INVALID = "INVALID"
    SELF_OCCLUDED = "SELF_OCCLUDED"


class NavigationHealthStatus(str, Enum):
    HEALTHY = "HEALTHY"
    DEGRADED = "DEGRADED"
    STALE = "STALE"
    STOPPED = "STOPPED"
    FAILED = "FAILED"


class NavigationControlSource(str, Enum):
    ROS_NAVIGATION_TEST = "ros_navigation_test"


class NavigationClockV1(StrictModel):
    run_id: str
    episode_id: str
    backend_generation: int = Field(ge=1)
    navigation_epoch: int = Field(ge=0)
    control_tick_id: str
    capture_sim_time_s: float
    capture_backend_monotonic_s: float
    clock_id: Literal["backend_monotonic"] = "backend_monotonic"


class LaserScanObservation(StrictModel):
    interface_profile: Literal["g3a-1.0"] = G3A_INTERFACE_PROFILE
    profile_id: str
    profile_version: str
    profile_sha256: str
    clock: NavigationClockV1
    scan_id: str
    frame_id: Literal["laser_frame"] = "laser_frame"
    mounting_body: Literal["lidar_link"] = "lidar_link"
    angle_min_rad: float
    angle_max_rad: float
    angle_increment_rad: float = Field(gt=0)
    range_min_m: float = Field(gt=0)
    range_max_m: float = Field(gt=0)
    time_increment_s: Literal[0.0] = 0.0
    ranges_m: list[float | None]
    statuses: list[LaserReturnStatus]
    beam_count: int = Field(gt=0)
    valid_count: int = Field(ge=0)
    status_counts: dict[str, int] = Field(default_factory=dict)
    transform_version: Literal["g3a-laser-transform-1"] = "g3a-laser-transform-1"

    @model_validator(mode="after")
    def validate_scan(self) -> "LaserScanObservation":
        if not (
            len(self.ranges_m) == len(self.statuses) == self.beam_count
        ):
            raise ValueError("range/status/beam lengths must match")
        if self.range_min_m >= self.range_max_m:
            raise ValueError("range_min_m must be below range_max_m")
        if not 0 <= self.valid_count <= self.beam_count:
            raise ValueError("valid_count outside beam count")
        expected_last = (
            self.angle_min_rad
            + (self.beam_count - 1) * self.angle_increment_rad
        )
        if abs(expected_last - self.angle_max_rad) > 1e-9:
            raise ValueError("angle_max_rad must equal final beam angle")
        for range_m, status in zip(self.ranges_m, self.statuses):
            if status == LaserReturnStatus.HIT:
                if (
                    not isinstance(range_m, (int, float))
                    or not math.isfinite(float(range_m))
                    or not self.range_min_m <= float(range_m) <= self.range_max_m
                ):
                    raise ValueError("HIT requires a finite in-range distance")
            elif range_m is not None:
                raise ValueError("non-HIT return must not carry a fabricated range")
        return self


class ImuMeasurement(StrictModel):
    interface_profile: Literal["g3a-1.0"] = G3A_INTERFACE_PROFILE
    sample_id: str
    clock: NavigationClockV1
    frame_id: Literal["imu_link"] = "imu_link"
    base_frame_id: Literal["base_link"] = "base_link"
    rotation_imu_to_base_wxyz: list[float]
    angular_velocity_rad_s: list[float] | None = None
    linear_acceleration_m_s2: list[float] | None = None
    gyro_valid: bool
    acceleration_valid: bool
    orientation_available: Literal[False] = False
    quality: str
    source: str

    @model_validator(mode="after")
    def validate_imu(self) -> "ImuMeasurement":
        if len(self.rotation_imu_to_base_wxyz) != 4:
            raise ValueError("IMU rotation must have four values")
        if self.gyro_valid != (self.angular_velocity_rad_s is not None):
            raise ValueError("gyro validity and payload disagree")
        if self.acceleration_valid != (self.linear_acceleration_m_s2 is not None):
            raise ValueError("acceleration validity and payload disagree")
        for values in (
            self.angular_velocity_rad_s,
            self.linear_acceleration_m_s2,
        ):
            if values is not None and (
                len(values) != 3
                or not all(math.isfinite(float(value)) for value in values)
            ):
                raise ValueError("IMU vectors must be finite 3-vectors")
        return self


class WheelMeasurement(StrictModel):
    interface_profile: Literal["g3a-1.0"] = G3A_INTERFACE_PROFILE
    sample_id: str
    clock: NavigationClockV1
    frame_id: Literal["base_link"] = "base_link"
    wheel_names: list[str]
    wheel_angles_rad: list[float]
    wheel_velocities_rad_s: list[float]
    wheel_radius_m: list[float]
    wheel_rotation_sign: list[float]
    valid: bool
    quality: str
    source: str

    @model_validator(mode="after")
    def validate_wheels(self) -> "WheelMeasurement":
        lengths = {
            len(self.wheel_names),
            len(self.wheel_angles_rad),
            len(self.wheel_velocities_rad_s),
            len(self.wheel_radius_m),
            len(self.wheel_rotation_sign),
        }
        if lengths != {4}:
            raise ValueError("four wheel measurements are required")
        if not all(
            math.isfinite(float(value))
            for value in (
                *self.wheel_angles_rad,
                *self.wheel_velocities_rad_s,
                *self.wheel_radius_m,
                *self.wheel_rotation_sign,
            )
        ):
            raise ValueError("wheel values must be finite")
        return self


class NavigationJointMeasurement(StrictModel):
    interface_profile: Literal["g3a-1.0"] = G3A_INTERFACE_PROFILE
    sample_id: str
    clock: NavigationClockV1
    joint_names: list[str]
    model_positions_rad: list[float]
    available: bool
    quality: str

    @model_validator(mode="after")
    def validate_joints(self) -> "NavigationJointMeasurement":
        if len(self.joint_names) != len(self.model_positions_rad):
            raise ValueError("joint names and positions must match")
        if not all(math.isfinite(float(value)) for value in self.model_positions_rad):
            raise ValueError("joint positions must be finite")
        return self


class NavigationObservation(StrictModel):
    interface_profile: Literal["g3a-1.0"] = G3A_INTERFACE_PROFILE
    profile_id: str
    profile_version: str
    profile_sha256: str
    sequence: int = Field(ge=0)
    clock: NavigationClockV1
    wheels: WheelMeasurement
    imu: ImuMeasurement
    joints: NavigationJointMeasurement
    scan: LaserScanObservation | None = None

    @model_validator(mode="after")
    def validate_clock_identity(self) -> "NavigationObservation":
        for item in (self.wheels.clock, self.imu.clock, self.joints.clock):
            if item != self.clock:
                raise ValueError("navigation measurements must share one capture clock")
        if self.scan is not None and self.scan.clock != self.clock:
            raise ValueError("scan must share the navigation capture clock")
        return self


class EstimatedBaseState(StrictModel):
    interface_profile: Literal["g3a-1.0"] = G3A_INTERFACE_PROFILE
    profile_id: str
    profile_version: str
    profile_sha256: str
    sequence: int = Field(ge=0)
    clock: NavigationClockV1
    odom_frame_id: Literal["odom"] = "odom"
    base_frame_id: Literal["base_link"] = "base_link"
    x_m: float
    y_m: float
    yaw_rad: float
    vx_body_m_s: float
    vy_body_m_s: float
    wz_body_rad_s: float
    pose_covariance: list[float]
    twist_covariance: list[float]
    source: Literal["wheel_gyro_planar_fusion"] = "wheel_gyro_planar_fusion"
    wheel_sample_id: str
    imu_sample_id: str
    wheel_age_s: float
    imu_age_s: float
    health: NavigationHealthStatus

    @model_validator(mode="after")
    def validate_estimate(self) -> "EstimatedBaseState":
        if len(self.pose_covariance) != 36 or len(self.twist_covariance) != 36:
            raise ValueError("ROS covariance arrays must contain 36 values")
        if not all(
            math.isfinite(float(value))
            for value in (
                self.x_m,
                self.y_m,
                self.yaw_rad,
                self.vx_body_m_s,
                self.vy_body_m_s,
                self.wz_body_rad_s,
                self.wheel_age_s,
                self.imu_age_s,
                *self.pose_covariance,
                *self.twist_covariance,
            )
        ):
            raise ValueError("estimate and covariance values must be finite")
        if self.wheel_age_s < 0.0 or self.imu_age_s < 0.0:
            raise ValueError("measurement ages must be nonnegative")
        return self


class NavigationHealth(StrictModel):
    interface_profile: Literal["g3a-1.0"] = G3A_INTERFACE_PROFILE
    profile_id: str
    profile_version: str
    profile_sha256: str
    sequence: int = Field(ge=0)
    clock: NavigationClockV1
    status: NavigationHealthStatus
    wheels_available: bool
    imu_gyro_available: bool
    imu_acceleration_available: bool
    scan_available: bool
    wheel_age_s: float | None = None
    imu_age_s: float | None = None
    scan_age_s: float | None = None
    odom_publish_rate_hz: float | None = None
    scan_publish_rate_hz: float | None = None
    rejection_reasons: list[str] = Field(default_factory=list)
    stop_required: bool = False

    @model_validator(mode="after")
    def validate_health(self) -> "NavigationHealth":
        ages = (self.wheel_age_s, self.imu_age_s, self.scan_age_s)
        if any(age is not None and age < 0.0 for age in ages):
            raise ValueError("measurement ages must be nonnegative")
        rates = (self.odom_publish_rate_hz, self.scan_publish_rate_hz)
        if any(rate is not None and rate < 0.0 for rate in rates):
            raise ValueError("publish rates must be nonnegative")
        if self.status == NavigationHealthStatus.STOPPED and not self.stop_required:
            raise ValueError("STOPPED health must require a stop")
        return self


class NavigationStreamEnvelope(StrictModel):
    schema_version: Literal[1] = 1
    message_type: Literal["observation", "estimate", "health", "heartbeat"]
    run_id: str
    episode_id: str
    backend_generation: int = Field(ge=1)
    navigation_epoch: int = Field(ge=0)
    backend_monotonic_s: float
    payload: dict


class NavigationControlMessage(StrictModel):
    schema_version: Literal[1] = 1
    message_type: Literal["cmd_vel", "release", "stop"]
    source: NavigationControlSource = NavigationControlSource.ROS_NAVIGATION_TEST
    run_id: str
    episode_id: str
    backend_generation: int = Field(ge=1)
    navigation_epoch: int = Field(ge=0)
    session_id: str
    lease_ttl_s: float = Field(default=0.25, gt=0.0, le=2.0)
    vx_m_s: float = 0.0
    vy_m_s: float = 0.0
    wz_rad_s: float = 0.0
    reason: str = ""

    @model_validator(mode="after")
    def validate_control(self) -> "NavigationControlMessage":
        if not all(
            math.isfinite(value) for value in (self.vx_m_s, self.vy_m_s, self.wz_rad_s)
        ):
            raise ValueError("control velocities must be finite")
        if self.message_type != "cmd_vel" and any(
            abs(value) > 1e-12
            for value in (self.vx_m_s, self.vy_m_s, self.wz_rad_s)
        ):
            raise ValueError("release/stop messages must command zero velocity")
        return self
