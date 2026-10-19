"""G2A observation timing, profile and policy-input contracts.

These models extend mobilecar-1.0 without changing existing wire identities.
Historical G1 records remain valid; G2A producers declare the explicit profile
version and clock semantics below.
"""

from __future__ import annotations

from enum import Enum
from typing import Any, Literal

from pydantic import Field

from .models import StrictModel


G2A_INTERFACE_PROFILE = "g2a-1.0"
G2A_OBSERVATION_PROFILE_ID = "g2a-rgbd-5hz-original"


class TimingClock(str, Enum):
    SIM = "sim"
    BACKEND_MONOTONIC = "backend_monotonic"
    DEVICE = "device"
    WALL_UTC = "wall_utc"


class ObservationProfile(StrictModel):
    schema_version: Literal[1] = 1
    interface_profile: Literal["g2a-1.0"] = G2A_INTERFACE_PROFILE
    profile_id: Literal["g2a-rgbd-5hz-original"] = G2A_OBSERVATION_PROFILE_ID
    profile_version: Literal["1.0"] = "1.0"
    contract_version: Literal["mobilecar-1.0"] = "mobilecar-1.0"
    scene_id: str
    model_sha256: str
    runtime: dict[str, Any]
    image: dict[str, Any]
    fast_streams: dict[str, Any]
    acceptance: dict[str, Any]
    required_streams: list[str]
    policy_input_allowlist: list[str]


class ActionProfile(StrictModel):
    schema_version: Literal[1] = 1
    contract_version: Literal["mobilecar-1.0"] = "mobilecar-1.0"
    profile_id: Literal["g2a-teleop-action-1.0"] = "g2a-teleop-action-1.0"
    profile_version: Literal["1.0"] = "1.0"
    description: str
    required_lifecycle: dict[str, Any]
    groups: dict[str, Any]
    observation_requirements: dict[str, list[str]]
    requirements: dict[str, bool]


class ObservationTimingV1(StrictModel):
    interface_profile: Literal["g2a-1.0"] = G2A_INTERFACE_PROFILE
    capture_sim_time_s: float
    capture_backend_monotonic_s: float
    capture_control_tick_id: str
    feedback_sample_id: str | None = None
    feedback_time_s: float | None = None
    feedback_clock: TimingClock = TimingClock.BACKEND_MONOTONIC
    render_start_backend_monotonic_s: float | None = None
    render_complete_backend_monotonic_s: float | None = None
    publish_backend_monotonic_s: float | None = None
    arrival_backend_monotonic_s: float | None = None
    persist_backend_monotonic_s: float | None = None
    capture_to_render_complete_ms: float | None = None
    render_complete_to_publish_ms: float | None = None
    publish_to_arrival_ms: float | None = None
    arrival_to_persist_ms: float | None = None
    observation_age_ms: float | None = None


class LatencySummary(StrictModel):
    count: int = Field(ge=0)
    p50_ms: float | None = None
    p95_ms: float | None = None
    p99_ms: float | None = None
    max_ms: float | None = None


class ObservationTimingSummary(StrictModel):
    interface_profile: Literal["g2a-1.0"] = G2A_INTERFACE_PROFILE
    profile_id: str
    profile_version: str
    profile_sha256: str
    declared_window_s: list[float]
    planned_samples: int = Field(ge=0)
    generated_samples: int = Field(ge=0)
    delivered_samples: int = Field(ge=0)
    persisted_samples: int = Field(ge=0)
    unique_frames: int = Field(ge=0)
    skipped_samples: int = Field(ge=0)
    skip_reasons: dict[str, int] = Field(default_factory=dict)
    wall_rate_hz: float | None = None
    sim_rate_hz: float | None = None
    real_time_factor: float | None = None
    interval_p50_s: float | None = None
    interval_p95_s: float | None = None
    interval_p99_s: float | None = None
    interval_max_s: float | None = None
    window_metrics: dict[str, Any] = Field(default_factory=dict)
    latency: dict[str, LatencySummary] = Field(default_factory=dict)
    stream_rates_hz: dict[str, float] = Field(default_factory=dict)
    stream_counts: dict[str, int] = Field(default_factory=dict)
    gates: dict[str, bool] = Field(default_factory=dict)
    gate_reasons: list[str] = Field(default_factory=list)


class PolicyObservationV1(StrictModel):
    """Allowlisted policy input; world_sim/GT diagnostics are deliberately absent."""

    interface_profile: Literal["g2a-1.0"] = G2A_INTERFACE_PROFILE
    contract_version: Literal["mobilecar-1.0"] = "mobilecar-1.0"
    run_id: str
    episode_id: str
    stream_id: str
    sample_id: str
    measurement_time_s: float
    clock_id: str
    frame_id: str
    rgb_uri: str | None = None
    depth_uri: str | None = None
    valid_mask_uri: str | None = None
    base_twist_body_m_s: list[float] | None = None
    arm_user_joint_positions_rad: list[float] | None = None
    gripper_width_m: float | None = None
    available_sensors: list[str] = Field(default_factory=list)
    source: str
    view_source: str
    quality: str


class G2AQualityReport(StrictModel):
    interface_profile: Literal["g2a-1.0"] = G2A_INTERFACE_PROFILE
    report_version: Literal["g2a-quality-1.0"] = "g2a-quality-1.0"
    episode_id: str
    run_id: str
    input_sha256: str
    input_hash_scope: list[str] = Field(default_factory=list)
    profile_id: str
    profile_version: str
    profile_sha256: str
    action_profile_id: str
    action_profile_version: str
    action_profile_sha256: str
    audit_status: str
    passed: bool
    timing: ObservationTimingSummary
    gates: dict[str, bool] = Field(default_factory=dict)
    gate_reasons: list[str] = Field(default_factory=list)
    generated_wall_utc: str
    output_scope: Literal["external_report_only"] = "external_report_only"
    known_limitations: list[str] = Field(default_factory=list)
