"""G2A profile-based quality, policy allowlist and eligibility."""

from __future__ import annotations

import hashlib
import json
import math
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Mapping

import yaml

from ..config import REPO_ROOT
from ..contracts import (
    ActionProfile,
    AuditStatus,
    DataEligibility,
    G2AQualityReport,
    LatencySummary,
    ObservationProfile,
    ObservationTimingSummary,
    PolicyObservationV1,
    TaskResult,
    ValidationReport,
)
from .recorder import _atomic_json
from .validator import validate_episode


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _read_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    value = json.loads(path.read_text(encoding="utf-8"))
    return value if isinstance(value, dict) else {}


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path.is_file():
        return rows
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            text = line.strip()
            if not text:
                continue
            value = json.loads(text)
            if isinstance(value, dict):
                rows.append(value)
    return rows


def _find_episode(root: Path, episode_id: str, run_id: str | None) -> tuple[Path, Path]:
    candidates = (
        [root / run_id / "episodes" / episode_id]
        if run_id
        else list(root.glob(f"*/episodes/{episode_id}"))
    )
    matches = [path for path in candidates if path.is_dir()]
    if len(matches) != 1:
        raise FileNotFoundError(episode_id)
    episode_dir = matches[0].resolve()
    return episode_dir.parent.parent, episode_dir


def _percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * q
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1 - fraction) + ordered[upper] * fraction


def _latency(values: Iterable[float | None]) -> LatencySummary:
    clean = [float(value) for value in values if value is not None and math.isfinite(float(value))]
    return LatencySummary(
        count=len(clean),
        p50_ms=_percentile(clean, 0.50),
        p95_ms=_percentile(clean, 0.95),
        p99_ms=_percentile(clean, 0.99),
        max_ms=max(clean) if clean else None,
    )


def load_observation_profile(path: str | Path) -> tuple[ObservationProfile, dict[str, Any], str]:
    profile_path = Path(path)
    if not profile_path.is_absolute():
        profile_path = REPO_ROOT / profile_path
    profile_path = profile_path.resolve()
    raw = json.loads(json.dumps(yaml.safe_load(profile_path.read_text(encoding="utf-8"))))
    profile = ObservationProfile.model_validate(raw)
    return profile, raw, _sha256(profile_path)


def load_action_profile(path: str | Path) -> tuple[ActionProfile, str]:
    profile_path = Path(path)
    if not profile_path.is_absolute():
        profile_path = REPO_ROOT / profile_path
    profile_path = profile_path.resolve()
    raw = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    return ActionProfile.model_validate(raw), _sha256(profile_path)


def _input_hash(episode_dir: Path) -> str:
    paths = sorted(
        path
        for path in episode_dir.rglob("*")
        if path.is_file()
        and not path.name.startswith(".")
    )
    digest = hashlib.sha256()
    for path in paths:
        relative = path.relative_to(episode_dir).as_posix()
        if not relative.startswith(
            (
                "manifest",
                "episode.json",
                "episode_complete.json",
                "events.jsonl",
                "commands.jsonl",
                "control.jsonl",
                "observations/",
                "transforms.jsonl",
                "initial_snapshot.json",
                "assets/",
                "assessments/",
                "quality_report.json",
            )
        ):
            continue
        digest.update(relative.encode())
        digest.update(b"\0")
        digest.update(_sha256(path).encode())
    return digest.hexdigest()


def _unique_persisted_samples(
    episode_dir: Path,
    persisted: Iterable[Mapping[str, Any]],
) -> tuple[set[tuple[str, str]], dict[str, Any]]:
    samples: set[tuple[str, str]] = set()
    uris: set[str] = set()
    rgb_hashes: dict[str, list[tuple[str, str]]] = defaultdict(list)
    missing: list[str] = []
    for row in persisted:
        key = (str(row.get("stream_id")), str(row.get("sample_id")))
        required = [row.get("rgb_uri"), row.get("depth_uri"), row.get("valid_mask_uri")]
        if not all(isinstance(value, str) and value for value in required):
            missing.append(f"{key[0]}:{key[1]}")
            continue
        resolved = [(episode_dir / str(value)).resolve() for value in required]
        if not all(
            path.is_relative_to(episode_dir.resolve()) and path.is_file() and path.stat().st_size > 0
            for path in resolved
        ):
            missing.append(f"{key[0]}:{key[1]}")
            continue
        samples.add(key)
        uris.update(str(value) for value in required)
        digest = _sha256(resolved[0])
        rgb_hashes[digest].append(key)
    duplicate_groups = [keys for keys in rgb_hashes.values() if len(keys) > 1]
    return samples, {
        "unique_asset_uris": len(uris),
        "missing_or_invalid_assets": missing,
        "pixel_identical_groups": len(duplicate_groups),
        "pixel_identical_samples": sum(len(keys) for keys in duplicate_groups),
    }


def _window_metrics(
    persisted_rows: list[Mapping[str, Any]],
    unique_samples: set[tuple[str, str]],
    *,
    start_s: float,
    end_s: float,
) -> dict[str, Any]:
    rows = [
        row
        for row in persisted_rows
        if isinstance(row.get("capture_sim_time_s"), (int, float))
        and start_s <= float(row["capture_sim_time_s"]) < end_s
    ]
    keys = {
        (str(row.get("stream_id")), str(row.get("sample_id")))
        for row in rows
    } & unique_samples
    times = sorted(
        float(row["capture_backend_monotonic_s"])
        for row in rows
        if isinstance(row.get("capture_backend_monotonic_s"), (int, float))
    )
    intervals = [b - a for a, b in zip(times, times[1:]) if b > a]
    duration = max(end_s - start_s, 1e-9)
    return {
        "range_s": [start_s, end_s],
        "generated_hint": len({(str(row.get("stream_id")), str(row.get("sample_id"))) for row in rows}),
        "persisted": len(rows),
        "unique": len(keys),
        "wall_rate_hz": len(keys) / duration,
        "interval_p50_s": _percentile(intervals, 0.50),
        "interval_p95_s": _percentile(intervals, 0.95),
        "interval_p99_s": _percentile(intervals, 0.99),
        "interval_max_s": max(intervals) if intervals else None,
    }


def assess_g2a_quality(
    root: str | Path,
    episode_id: str,
    *,
    profile_path: str | Path = "configs/profiles/g2a_rgbd_5hz.yaml",
    action_profile_path: str | Path = "configs/profiles/g2a_action_teleop_v1.yaml",
    run_id: str | None = None,
    output_root: str | Path | None = None,
    write: bool = True,
    validation: ValidationReport | None = None,
) -> G2AQualityReport:
    root_path = Path(root).resolve()
    run_dir, episode_dir = _find_episode(root_path, episode_id, run_id)
    profile, raw_profile, profile_sha = load_observation_profile(profile_path)
    action_profile, action_profile_sha = load_action_profile(action_profile_path)
    general = validation or validate_episode(
        root_path, episode_id, run_id=run_dir.name, write_report=False
    )
    general_audit = AuditStatus(general.audit_status)
    episode = _read_json(episode_dir / "episode.json")
    events = _read_jsonl(episode_dir / "events.jsonl")
    observations: list[dict[str, Any]] = []
    for path in sorted((episode_dir / "observations").glob("*.jsonl")):
        if path.name in {"index.jsonl", "persist.jsonl"}:
            continue
        observations.extend(_read_jsonl(path))
    persist_rows = _read_jsonl(episode_dir / "observations/persist.jsonl")

    capture_by_key: dict[tuple[str, str], dict[str, dict[str, Any]]] = defaultdict(dict)
    for observation in observations:
        kind = str(observation.get("sensor_kind"))
        if kind in {"rgb", "depth"}:
            key = (str(observation.get("stream_id")), str(observation.get("sample_id")))
            capture_by_key[key][kind] = observation

    start_s, end_s = (float(value) for value in raw_profile["acceptance"]["analysis_window_s"])
    init_s, init_end_s = (
        float(value) for value in raw_profile["acceptance"]["init_window_s"]
    )
    final_start_s, final_end_s = (
        float(value) for value in raw_profile["acceptance"]["final_window_s"]
    )
    episode_end_s = float(raw_profile["acceptance"]["episode_duration_s"])
    planned = int(raw_profile["acceptance"]["planned_samples_in_window"])
    target_rate = float(raw_profile["acceptance"]["min_average_wall_hz"])
    target_p99 = float(raw_profile["acceptance"]["max_p99_interval_s"])
    min_unique = int(raw_profile["acceptance"]["min_persisted_unique_samples"])

    generated_episode: dict[tuple[str, str], dict[str, dict[str, Any]]] = {}
    delivered_episode: dict[tuple[str, str], dict[str, dict[str, Any]]] = {}
    for key, kinds in capture_by_key.items():
        rgb = kinds.get("rgb")
        if rgb is None:
            continue
        capture_sim = rgb.get("capture_sim_time_s")
        if (
            not isinstance(capture_sim, (int, float))
            or not 0.0 <= float(capture_sim) < episode_end_s
        ):
            continue
        generated_episode[key] = kinds
        if set(kinds) == {"rgb", "depth"}:
            delivered_episode[key] = kinds

    generated_samples = {
        key: kinds
        for key, kinds in generated_episode.items()
        if start_s <= float(kinds["rgb"]["capture_sim_time_s"]) < end_s
    }
    delivered_samples = {
        key: kinds
        for key, kinds in delivered_episode.items()
        if start_s <= float(kinds["rgb"]["capture_sim_time_s"]) < end_s
    }

    persist_by_key = {
        (str(row.get("stream_id")), str(row.get("sample_id"))): row
        for row in persist_rows
    }
    persisted_episode: list[dict[str, Any]] = []
    for key in delivered_episode:
        row = persist_by_key.get(key)
        if row is None:
            continue
        capture_sim = row.get("capture_sim_time_s")
        if isinstance(capture_sim, (int, float)) and 0.0 <= float(capture_sim) < episode_end_s:
            persisted_episode.append(row)
    persisted_rows = [
        row
        for row in persisted_episode
        if start_s <= float(row["capture_sim_time_s"]) < end_s
    ]

    persisted_capture_times = sorted(
        float(row["capture_backend_monotonic_s"])
        for row in persisted_rows
        if isinstance(row.get("capture_backend_monotonic_s"), (int, float))
    )
    intervals = [
        b - a
        for a, b in zip(persisted_capture_times, persisted_capture_times[1:])
        if b > a
    ]
    unique_sample_keys, asset_diagnostics = _unique_persisted_samples(
        episode_dir, persisted_rows
    )
    unique_episode_sample_keys, episode_asset_diagnostics = _unique_persisted_samples(
        episode_dir, persisted_episode
    )
    unique_frames = len(unique_sample_keys)
    window_duration = max(end_s - start_s, 1e-9)
    wall_rate = unique_frames / window_duration
    sim_rate = unique_frames / window_duration
    interval_p99 = _percentile(intervals, 0.99)

    timing_by_sample = {
        str(event.get("payload", {}).get("sample_id")): event.get("payload", {})
        for event in events
        if event.get("event_type") == "rgbd_capture_timing"
    }
    persisted_keys = {
        (str(row.get("stream_id")), str(row.get("sample_id"))) for row in persisted_rows
    }
    timing_rows = [
        timing_by_sample.get(sample_id, {})
        for _stream, sample_id in persisted_keys
    ]
    timing_rows = [row for row in timing_rows if row]
    latency = {
        "lock_wait": _latency(row.get("lock_wait_ms") for row in timing_rows),
        "capture_to_render_complete": _latency(
            row.get("capture_to_render_complete_ms") for row in timing_rows
        ),
        "render": _latency(row.get("render_ms") for row in timing_rows),
        "encode": _latency(row.get("encode_ms") for row in timing_rows),
        "render_complete_to_publish": _latency(
            row.get("render_complete_to_publish_ms") for row in timing_rows
        ),
        "publish": _latency(row.get("publish_ms") for row in timing_rows),
        "arrival_to_persist": _latency(
            row.get("arrival_to_persist_ms") for row in persisted_rows
        ),
        "capture_to_persist": _latency(
            row.get("capture_to_persist_ms") for row in persisted_rows
        ),
    }

    stream_rates: dict[str, float] = {}
    stream_counts: dict[str, int] = {}
    by_stream: dict[str, list[float]] = defaultdict(list)
    for observation in observations:
        kind = str(observation.get("sensor_kind"))
        if kind in {"rgb", "depth"}:
            continue
        stream = str(observation.get("stream_id"))
        measurement = observation.get("measurement_time_s")
        if isinstance(measurement, (int, float)) and math.isfinite(float(measurement)):
            by_stream[stream].append(float(measurement))
    for stream, times in by_stream.items():
        stream_counts[stream] = len(times)
        times.sort()
        if len(times) > 1:
            gaps = [b - a for a, b in zip(times, times[1:]) if b > a]
            stream_rates[stream] = 1.0 / (sum(gaps) / len(gaps)) if gaps else 0.0

    recording_start = next(
        (event for event in events if event.get("event_type") == "recording_start"),
        None,
    )
    recording_stop = next(
        (event for event in reversed(events) if event.get("event_type") == "recording_stop"),
        None,
    )
    real_time_factor = None
    if recording_start and recording_stop:
        wall_span = float(recording_stop.get("backend_monotonic_s", 0.0)) - float(
            recording_start.get("backend_monotonic_s", 0.0)
        )
        sim_span = float(recording_stop.get("sim_time_s", 0.0)) - float(
            recording_start.get("sim_time_s", 0.0)
        )
        if wall_span > 0:
            real_time_factor = sim_span / wall_span

    start_ns = (
        float(recording_stop.get("sim_time_s", 0.0)) - float(recording_start.get("sim_time_s", 0.0))
        if recording_start and recording_stop
        else 0.0
    )
    generated = len(generated_samples)
    delivered = len(delivered_samples)
    persisted = len(persisted_rows)
    skip_reasons = {
        "not_generated": max(0, planned - generated),
        "missing_pair": max(0, generated - delivered),
        "not_persisted": max(0, delivered - persisted),
        "missing_or_invalid_asset": max(0, persisted - unique_frames),
    }
    stop_snapshot = ((recording_stop or {}).get("payload") or {}).get("snapshot") or {}
    schedule = ((stop_snapshot.get("diagnostics") or {}).get("rgbd_schedule") or {})
    if isinstance(schedule.get("skip_reasons"), dict):
        for reason, count in schedule["skip_reasons"].items():
            skip_reasons[f"backend:{reason}"] = int(count)

    profile_match = bool(persisted_rows) and all(
        row.get("profile_id") == profile.profile_id
        and row.get("profile_version") == profile.profile_version
        and row.get("profile_sha256") == profile_sha
        for row in persisted_rows
    )
    def causal(row: Mapping[str, Any]) -> bool:
        values = [
            row.get("capture_backend_monotonic_s"),
            row.get("arrival_backend_monotonic_s"),
            row.get("persist_backend_monotonic_s"),
        ]
        if not all(isinstance(value, (int, float)) for value in values):
            return False
        if not float(values[0]) <= float(values[1]) + 1e-6 <= float(values[2]) + 1e-6:
            return False
        timing = timing_by_sample.get(str(row.get("sample_id")), {})
        ordered = [
            timing.get("capture_backend_monotonic_s"),
            timing.get("render_start_backend_monotonic_s"),
            timing.get("render_complete_backend_monotonic_s"),
            timing.get("publish_backend_monotonic_s"),
            timing.get("arrival_backend_monotonic_s"),
        ]
        if not all(isinstance(value, (int, float)) for value in ordered):
            return False
        return all(
            float(a) <= float(b) + 1e-6 for a, b in zip(ordered, ordered[1:])
        )

    timing_causality = bool(persisted_rows) and all(causal(row) for row in persisted_rows)
    capture_binding = bool(persisted_rows) and all(
        row.get("capture_control_tick_id") and row.get("feedback_sample_id")
        for row in persisted_rows
    )
    transform_version = bool(observations) and all(
        (observation.get("metadata") or {}).get("transform_version") == "g2a-transform-1"
        for observation in observations
        if observation.get("sensor_kind") in {"rgb", "depth"}
    )
    windows = {
        "init": _window_metrics(
            persisted_episode,
            unique_episode_sample_keys,
            start_s=init_s,
            end_s=init_end_s,
        ),
        "analysis": _window_metrics(
            persisted_episode,
            unique_episode_sample_keys,
            start_s=start_s,
            end_s=end_s,
        ),
        "final": _window_metrics(
            persisted_episode,
            unique_episode_sample_keys,
            start_s=final_start_s,
            end_s=final_end_s,
        ),
    }
    windows["asset_diagnostics"] = episode_asset_diagnostics
    gates = {
        "audit_complete": general_audit == AuditStatus.COMPLETE and general.passed,
        "profile_match": profile_match,
        "analyzed_window_complete": start_ns >= float(raw_profile["acceptance"]["episode_duration_s"]) - 0.25,
        "rgb_depth_pairs": generated > 0 and generated == delivered,
        "persisted_unique": unique_frames >= min_unique,
        "asset_integrity": bool(unique_frames) and not asset_diagnostics["missing_or_invalid_assets"],
        "average_wall_rate": wall_rate >= target_rate,
        "p99_interval": interval_p99 is not None and interval_p99 <= target_p99,
        "timing_causality": timing_causality,
        "capture_binding": capture_binding,
        "transform_version": transform_version,
    }
    gate_reasons = [name for name, passed in gates.items() if not passed]
    summary = ObservationTimingSummary(
        profile_id=profile.profile_id,
        profile_version=profile.profile_version,
        profile_sha256=profile_sha,
        declared_window_s=[start_s, end_s],
        planned_samples=planned,
        generated_samples=generated,
        delivered_samples=delivered,
        persisted_samples=persisted,
        unique_frames=unique_frames,
        skipped_samples=max(0, planned - unique_frames),
        skip_reasons={key: value for key, value in skip_reasons.items() if value},
        wall_rate_hz=wall_rate,
        sim_rate_hz=sim_rate,
        real_time_factor=real_time_factor,
        interval_p50_s=_percentile(intervals, 0.50),
        interval_p95_s=_percentile(intervals, 0.95),
        interval_p99_s=interval_p99,
        interval_max_s=max(intervals) if intervals else None,
        window_metrics=windows,
        latency=latency,
        stream_rates_hz=stream_rates,
        stream_counts=stream_counts,
        gates=gates,
        gate_reasons=gate_reasons,
    )
    report = G2AQualityReport(
        episode_id=episode_id,
        run_id=run_dir.name,
        input_sha256=_input_hash(episode_dir),
        input_hash_scope=[
            "manifest.json",
            "episode.json",
            "episode_complete.json",
            "events.jsonl",
            "commands.jsonl",
            "control.jsonl",
            "observations/**/*.jsonl",
            "transforms.jsonl",
            "initial_snapshot.json",
            "assets/**",
            "assessments/**",
            "quality_report.json",
        ],
        profile_id=profile.profile_id,
        profile_version=profile.profile_version,
        profile_sha256=profile_sha,
        action_profile_id=action_profile.profile_id,
        action_profile_version=action_profile.profile_version,
        action_profile_sha256=action_profile_sha,
        audit_status=general_audit.value,
        passed=all(gates.values()),
        timing=summary,
        gates=gates,
        gate_reasons=gate_reasons,
        generated_wall_utc=_utc_now(),
        known_limitations=[
            "This report is valid only for the declared 640x480, 5 Hz g2a-rgbd-5hz-original profile.",
            "G1 historical D2 eligibility is not upgraded by this report.",
            "Navigation-frame correction for world-aligned lidar remains a G3 item.",
        ],
    )
    if write:
        output = (
            Path(output_root)
            if output_root is not None
            else root_path / "_g2a_reports"
        )
        output = output.resolve()
        output.mkdir(parents=True, exist_ok=True)
        _atomic_json(
            output / f"{episode_id}.g2a_quality.json",
            report.model_dump(mode="json"),
        )
    return report


def policy_observation_from_records(
    *,
    episode: Mapping[str, Any],
    rgb: Mapping[str, Any],
    depth: Mapping[str, Any] | None,
    joint_feedback: Mapping[str, Any],
    encoders: Mapping[str, Any],
    allowed_keys: Iterable[str],
) -> PolicyObservationV1:
    allowed = set(allowed_keys)
    forbidden = {
        "world_sim",
        "object_pose_world_sim",
        "cube_pose_world_sim",
        "diagnostics",
        "oracle",
        "truth",
    }

    def reject_truth(value: Any) -> None:
        if isinstance(value, Mapping):
            for key, child in value.items():
                lowered = str(key).lower()
                if lowered in forbidden or any(
                    token in lowered for token in ("truth", "oracle", "world_sim")
                ):
                    raise ValueError(f"policy input contains forbidden field: {key}")
                reject_truth(child)
        elif isinstance(value, list):
            for child in value:
                reject_truth(child)

    for record in (rgb, depth or {}, joint_feedback, encoders):
        reject_truth(record)
    feedback_value = joint_feedback.get("value") or {}
    encoder_value = encoders.get("value") or {}
    payload = {
        "run_id": rgb.get("run_id"),
        "episode_id": rgb.get("episode_id"),
        "stream_id": rgb.get("stream_id"),
        "sample_id": rgb.get("sample_id"),
        "measurement_time_s": rgb.get("measurement_time_s"),
        "clock_id": rgb.get("clock_id"),
        "frame_id": rgb.get("frame_id"),
        "rgb_uri": rgb.get("asset_uri"),
        "depth_uri": (depth or {}).get("asset_uri"),
        "valid_mask_uri": (depth or {}).get("valid_mask_uri"),
        "base_twist_body_m_s": encoder_value.get("base_twist_body_m_s"),
        "arm_user_joint_positions_rad": feedback_value.get("user_joint_positions_rad"),
        "gripper_width_m": feedback_value.get("gripper_width_m"),
        "available_sensors": sorted(
            {
                str(item.get("sensor_kind"))
                for item in (rgb, depth, joint_feedback, encoders)
                if item
            }
        ),
        "source": rgb.get("source"),
        "view_source": episode.get("view_source"),
        "quality": rgb.get("quality"),
    }
    filtered = {key: value for key, value in payload.items() if key in allowed}
    return PolicyObservationV1.model_validate(filtered)


def g2a_eligibility(
    report: G2AQualityReport,
    episode: Mapping[str, Any],
    events: Iterable[Mapping[str, Any]],
    assessment: Mapping[str, Any],
    observations: Iterable[Mapping[str, Any]],
) -> DataEligibility:
    event_rows = list(events)
    observation_rows = list(observations)
    kinds = {str(item.get("sensor_kind")) for item in observation_rows}
    phases = {
        str(event.get("task_phase"))
        for event in event_rows
        if event.get("task_phase")
    }
    requested_types: dict[str, str] = {}
    requested_payloads: dict[str, Mapping[str, Any]] = {}
    for event in event_rows:
        if event.get("event_type") != "requested" or not event.get("command_id"):
            continue
        request = (event.get("payload") or {}).get("request") or {}
        if isinstance(request, Mapping):
            requested_types[str(event["command_id"])] = str(
                request.get("command_type", "")
            )
            requested_payloads[str(event["command_id"])] = request.get("payload") or {}
    base_commands = {"set_base_twist"}
    arm_commands = {
        "set_arm_joint_goal",
        "set_gripper_width_goal",
        "execute_tcp_target",
    }
    action_rows = [
        event
        for event in event_rows
        if event.get("event_type") in {"approved", "dispatch", "dispatched"}
        and requested_types.get(str(event.get("command_id"))) in base_commands | arm_commands
    ]
    reference_rows = [
        event for event in event_rows if event.get("event_type") == "actuator_reference"
    ]
    audit_complete = report.audit_status == AuditStatus.COMPLETE.value
    try:
        task_result = TaskResult(assessment.get("task_result", TaskResult.UNKNOWN.value))
    except ValueError:
        task_result = TaskResult.UNKNOWN
    def nonzero_twist(command_id: str) -> bool:
        payload = requested_payloads.get(command_id) or {}
        return any(
            isinstance(payload.get(key), (int, float))
            and not isinstance(payload.get(key), bool)
            and abs(float(payload[key])) > 1e-9
            for key in ("vx_m_s", "vy_m_s", "wz_rad_s")
        )

    base_actions = any(
        requested_types.get(str(event.get("command_id"))) == "set_base_twist"
        and nonzero_twist(str(event.get("command_id")))
        for event in action_rows
    )
    arm_actions = any(
        requested_types.get(str(event.get("command_id"))) in arm_commands
        for event in action_rows
    )
    base_lifecycle = base_actions and bool(reference_rows)
    arm_lifecycle = arm_actions and bool(reference_rows)
    reason_rows: list[str] = list(report.gate_reasons)
    d0 = bool(
        audit_complete
        and report.passed
        and base_lifecycle
    )
    d2 = bool(
        audit_complete
        and report.passed
        and arm_lifecycle
        and "rgb" in kinds
        and "joint_feedback" in kinds
    )
    d3 = bool(
        d2
        and len(phases) >= 2
        and task_result not in {TaskResult.UNKNOWN, TaskResult.NOT_APPLICABLE}
    )
    actionable = bool(audit_complete and report.passed and arm_lifecycle)
    wrist_only = bool(
        actionable
        and episode.get("view_source") == "wrist_rgb_only"
        and "rgb" in kinds
        and "joint_feedback" in kinds
    )
    if not d0:
        reason_rows.append("D0 profile/audit/base-action gate failed")
    if not d2:
        reason_rows.append("D2 profile/audit/action/proprioception gate failed")
    if episode.get("operator_kind") == "agent_ui":
        reason_rows.append("agent_ui is development evidence; not a human_teleop source")
    return DataEligibility(
        episode_id=report.episode_id,
        audit_status=AuditStatus(report.audit_status),
        task_result=task_result,
        d0_navigation=d0,
        d1_language_navigation=False,
        d2_near_field_manipulation=d2,
        d3_combined_task=d3,
        reasons=sorted(set(reason_rows)),
        source_operator_kind=episode.get("operator_kind"),
        view_source=episode.get("view_source"),
        wrist_rgb_only_eligible=wrist_only,
        profile_id=report.profile_id,
        profile_version=report.profile_version,
        profile_sha256=report.profile_sha256,
        action_profile_id=report.action_profile_id,
        action_profile_version=report.action_profile_version,
        action_profile_sha256=report.action_profile_sha256,
        gates=report.gates,
    )


def write_g2a_eligibility(
    output_root: str | Path,
    eligibility: DataEligibility,
) -> Path:
    output = Path(output_root).resolve()
    output.mkdir(parents=True, exist_ok=True)
    path = output / f"{eligibility.episode_id}.g2a_eligibility.json"
    _atomic_json(path, eligibility.model_dump(mode="json"))
    return path


def assess_g2a_eligibility(
    root: str | Path,
    episode_id: str,
    *,
    profile_path: str | Path = "configs/profiles/g2a_rgbd_5hz.yaml",
    action_profile_path: str | Path = "configs/profiles/g2a_action_teleop_v1.yaml",
    run_id: str | None = None,
    output_root: str | Path | None = None,
    write: bool = True,
    validation: ValidationReport | None = None,
) -> tuple[G2AQualityReport, DataEligibility]:
    root_path = Path(root).resolve()
    run_dir, episode_dir = _find_episode(root_path, episode_id, run_id)
    output = (
        Path(output_root).resolve()
        if output_root is not None
        else root_path / "_g2a_reports"
    )
    quality = assess_g2a_quality(
        root_path,
        episode_id,
        profile_path=profile_path,
        action_profile_path=action_profile_path,
        run_id=run_dir.name,
        output_root=output,
        write=write,
        validation=validation,
    )
    episode = _read_json(episode_dir / "episode.json")
    events = _read_jsonl(episode_dir / "events.jsonl")
    observations = _read_jsonl(episode_dir / "observations/index.jsonl")
    if not observations:
        for path in sorted((episode_dir / "observations").glob("*.jsonl")):
            if path.name not in {"index.jsonl", "persist.jsonl"}:
                observations.extend(_read_jsonl(path))
    assessment = _read_json(episode_dir / "assessments/task_assessment.json")
    eligibility = g2a_eligibility(
        quality,
        episode,
        events,
        assessment or {"task_result": episode.get("result", TaskResult.UNKNOWN.value)},
        observations,
    )
    if write:
        write_g2a_eligibility(output, eligibility)
    return quality, eligibility
