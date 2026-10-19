"""Shared, backend-free contracts for the G2B read-only export pipeline."""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import yaml


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def sha256_json(value: Any) -> str:
    encoded = json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _finite_vector(values: Sequence[Any], size: int, label: str) -> tuple[float, ...]:
    if len(values) != size:
        raise ValueError(f"{label} must contain {size} values")
    result = []
    for value in values:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{label} must contain finite numbers")
        number = float(value)
        if not math.isfinite(number):
            raise ValueError(f"{label} must contain finite numbers")
        result.append(number)
    return tuple(result)


@dataclass(frozen=True)
class SourceIdentity:
    run_id: str
    episode_id: str
    recording_segment_id: str
    interface_profile: str
    observation_profile_id: str
    observation_profile_sha256: str
    action_profile_id: str
    action_profile_sha256: str


@dataclass(frozen=True)
class ImageSample:
    sample_id: str
    stream_id: str
    recording_segment_id: str
    capture_sim_time_s: float
    capture_backend_monotonic_s: float
    publish_backend_monotonic_s: float
    arrival_backend_monotonic_s: float
    persist_backend_monotonic_s: float
    control_tick_id: str
    feedback_sample_id: str
    rgb_uri: str
    depth_uri: str
    valid_mask_uri: str
    width: int
    height: int
    run_id: str = ""
    episode_id: str = ""
    rgb: Mapping[str, Any] = field(default_factory=dict)
    depth: Mapping[str, Any] = field(default_factory=dict)
    persist: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class ObservationSample:
    sample_id: str
    sensor_kind: str
    stream_id: str
    recording_segment_id: str
    control_tick_id: str
    capture_sim_time_s: float
    capture_backend_monotonic_s: float
    arrival_backend_monotonic_s: float
    value: Mapping[str, Any]
    raw: Mapping[str, Any] = field(default_factory=dict)
    run_id: str = ""
    episode_id: str = ""
    feedback_sample_id: str = ""


@dataclass(frozen=True)
class ActionDispatch:
    command_id: str
    command_type: str
    group: str
    backend_monotonic_s: float
    sim_time_s: float
    control_tick_id: str | None
    requested_event: Mapping[str, Any]
    approved_event: Mapping[str, Any]
    dispatch_event: Mapping[str, Any]


@dataclass(frozen=True)
class ActuatorReference:
    sequence: int
    backend_monotonic_s: float
    sim_time_s: float
    control_tick_id: str | None
    payload: Mapping[str, Any]
    raw: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class ProfileRow:
    profile: str
    state: tuple[float, ...]
    action: tuple[float, ...]
    action_units: tuple[str, ...]
    action_frame_id: str
    dispatch_provenance: Mapping[str, Any]
    effective_intervals: Mapping[str, Any]
    observation_sources: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class AlignedRow:
    grid_index: int
    decision_sim_time_s: float
    decision_backend_monotonic_s: float
    image: ImageSample
    joint_feedback: ObservationSample | None
    encoder: ObservationSample | None
    imu: ObservationSample | None
    actuator_reference: ActuatorReference
    image_age_s: float
    image_arrival_age_s: float
    feedback_age_s: float
    image_feedback_capture_skew_s: float
    profiles: Mapping[str, ProfileRow]
    reasons: tuple[str, ...] = ()


@dataclass(frozen=True)
class RowDecision:
    include: bool
    reasons: tuple[str, ...]
    segment_id: str = "dev"
    details: Mapping[str, Any] = field(default_factory=dict)


@dataclass
class AlignmentResult:
    rows: list[AlignedRow]
    rejected: list[dict[str, Any]]
    summary: Mapping[str, Any]
    source_identity: Mapping[str, Any]
    profile_max_gap_slots: int = 1

    def rows_for(self, profile_name: str) -> list[AlignedRow]:
        key = profile_name.lower()
        return [row for row in self.rows if key in row.profiles]

    def contiguous_segments(self, profile_name: str) -> list[list[AlignedRow]]:
        """Split at rejected slots and at declared long accepted gaps."""

        rows = self.rows_for(profile_name)
        if not rows:
            return []
        segments: list[list[AlignedRow]] = []
        current: list[AlignedRow] = [rows[0]]
        for previous, row in zip(rows, rows[1:]):
            grid_gap = row.grid_index - previous.grid_index
            long_gap = (
                grid_gap > self.profile_max_gap_slots + 1
                or "accepted_after_long_gap_split" in row.reasons
            )
            if long_gap:
                segments.append(current)
                current = []
            current.append(row)
        segments.append(current)
        return segments


@dataclass(frozen=True)
class ExportProfile:
    path: Path
    raw: Mapping[str, Any]
    sha256: str

    @property
    def profile_id(self) -> str:
        return str(self.raw["profile_id"])

    @property
    def profile_version(self) -> str:
        value = self.raw.get("profile_version", "1.0")
        return str(value)

    @property
    def grid_rate_hz(self) -> float:
        return float(self.raw["grid"]["rate_hz"])

    @property
    def image_max_age_s(self) -> float:
        return float(self.raw["causality"]["image_max_age_s"])

    @property
    def feedback_max_age_s(self) -> float:
        return float(self.raw["causality"]["feedback_max_age_s"])

    @property
    def max_capture_skew_s(self) -> float:
        return float(self.raw["causality"]["max_image_feedback_capture_skew_s"])

    @property
    def max_gap_slots(self) -> int:
        return int(self.raw["causality"]["max_gap_slots"])

    @property
    def image_key(self) -> str:
        return str(self.raw["features"]["image"]["key"])

    @property
    def model_sha256(self) -> str:
        return str(self.raw["coordinates"]["model_sha256"])

    @property
    def robot_config_sha256(self) -> str:
        return str(self.raw["coordinates"]["robot_config_sha256"])

    @property
    def arm_spec_sha256(self) -> str:
        return str(self.raw["coordinates"]["arm_spec_sha256"])

    def action_spec(self, profile_name: str) -> Mapping[str, Any]:
        return self.raw["actions"][profile_name.lower()]

    def feature_spec(self, profile_name: str) -> Mapping[str, Any]:
        return self.raw["features"][profile_name.lower()]


def load_export_profile(path: str | Path) -> ExportProfile:
    profile_path = Path(path).expanduser().resolve()
    raw = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("export profile must be a YAML mapping")
    profile = ExportProfile(
        path=profile_path,
        raw=raw,
        sha256=sha256_file(profile_path),
    )
    if profile.profile_version != "1.0":
        raise ValueError(f"unsupported export profile version {profile.profile_version}")
    if abs(profile.grid_rate_hz - 5.0) > 1e-12:
        raise ValueError("G2B v1 requires a 5 Hz export grid")
    for name in ("d0", "d2"):
        feature = profile.feature_spec(name)
        action = profile.action_spec(name)
        if not feature.get("state_names") or not feature.get("action_names"):
            raise ValueError(f"{name} feature names are incomplete")
        if not action.get("layer") or not action.get("units"):
            raise ValueError(f"{name} action semantics are incomplete")
    image = profile.raw["features"]["image"]
    if image.get("sensor_parent") != "link4" or image.get("storage") != "video":
        raise ValueError("G2B v1 image must be link4 RGB video")
    if image.get("color") != "RGB8" or image.get("policy_input") is not True:
        raise ValueError("G2B v1 image feature must be policy-visible RGB8")
    if image.get("width") != 640 or image.get("height") != 480:
        raise ValueError("G2B v1 image profile must remain 640x480")
    if profile.raw["actions"]["d0"].get("measurement_or_feedback_as_action") is not False:
        raise ValueError("D0 actions cannot be measurement feedback")
    if profile.raw["actions"]["d0"].get("derived_from_wheel_target") is not False:
        raise ValueError("D0 must preserve the declared body-reference derivation")
    if profile.raw["actions"]["d2"].get("preview_or_rejected_as_action") is not False:
        raise ValueError("D2 cannot promote preview/rejected commands")
    if profile.raw["lerobot"].get("version") != "0.4.4":
        raise ValueError("G2B writer version must be lerobot==0.4.4")
    if profile.raw["lerobot"].get("dataset_codebase_version") != "v3.0":
        raise ValueError("G2B dataset format must be LeRobot v3.0")
    return profile


def validate_numeric_profile_row(
    *,
    state: Iterable[float],
    action: Iterable[float],
    state_size: int,
    action_size: int,
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    state_values = _finite_vector(tuple(state), state_size, "state")
    action_values = _finite_vector(tuple(action), action_size, "action")
    return state_values, action_values
