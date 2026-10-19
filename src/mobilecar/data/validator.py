"""Read-only Episode audit producing a versioned quality report."""

from __future__ import annotations

import hashlib
import json
import math
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Mapping

from ..contracts import AuditStatus, OperatorKind, ValidationReport
from .recorder import COMPLETE_FILE, CONTRACT_VERSION, TERMINAL_EVENT, _atomic_json


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _read_jsonl(path: Path, errors: list[str]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path.exists():
        return rows
    with path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            text = line.strip()
            if not text:
                continue
            try:
                value = json.loads(text)
            except json.JSONDecodeError as exc:
                errors.append(f"{path.name}:{line_number}: invalid JSON: {exc.msg}")
                continue
            if not isinstance(value, dict):
                errors.append(f"{path.name}:{line_number}: expected JSON object")
                continue
            rows.append(value)
    return rows


def _find_episode(root: Path, episode_id: str, run_id: str | None) -> tuple[Path, Path] | None:
    if run_id:
        candidates = [root / run_id / "episodes" / episode_id]
    else:
        candidates = list(root.glob(f"*/episodes/{episode_id}"))
    matches = [path for path in candidates if path.is_dir()]
    if len(matches) != 1:
        return None
    episode_dir = matches[0]
    return episode_dir.parent.parent, episode_dir


def _asset_references(
    episode: Mapping[str, Any],
    events: Iterable[Mapping[str, Any]],
    observations: Iterable[Mapping[str, Any]],
) -> dict[str, list[str]]:
    references: dict[str, list[str]] = defaultdict(list)
    for item in observations:
        for key in ("asset_uri", "valid_mask_uri"):
            value = item.get(key)
            if isinstance(value, str):
                references[value].append(f"{item.get('sensor_kind', 'unknown')}:{item.get('sample_id', '?')}:{key}")
    for event in events:
        _collect_asset_refs(event, references, f"event:{event.get('event_type', '?')}")
    _collect_asset_refs(episode, references, "episode")
    return references


def _collect_asset_refs(value: Any, references: dict[str, list[str]], context: str) -> None:
    if isinstance(value, Mapping):
        for key, child in value.items():
            if key in {"asset_uri", "valid_mask_uri", "rgb_uri", "depth_uri"} and isinstance(child, str):
                references[child].append(f"{context}:{key}")
            elif key not in {"checksums"}:
                _collect_asset_refs(child, references, context)
    elif isinstance(value, list):
        for child in value:
            _collect_asset_refs(child, references, context)


def _is_finite_number(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _contains_world_truth(value: Any) -> bool:
    if isinstance(value, Mapping):
        for key, child in value.items():
            lowered = str(key).lower()
            if "truth" in lowered or key in {"object_pose_world_sim", "cube_pose_world_sim"}:
                return True
            if _contains_world_truth(child):
                return True
    elif isinstance(value, list):
        return any(_contains_world_truth(child) for child in value)
    return False


def validate_episode(
    root: str | Path,
    episode_id: str,
    *,
    run_id: str | None = None,
    write_report: bool = True,
) -> ValidationReport:
    """Validate identities, associations, assets, causality and closure."""

    root_path = Path(root).resolve()
    errors: list[str] = []
    warnings: list[str] = []
    found = _find_episode(root_path, episode_id, run_id)
    if found is None:
        report = ValidationReport(
            episode_id=episode_id,
            audit_status=AuditStatus.INCOMPLETE,
            passed=False,
            errors=["Episode directory is missing or ambiguous"],
            generated_wall_utc=_utc_now(),
        )
        return report
    source_run_dir, episode_dir = found
    effective_run_id = source_run_dir.name

    manifest_path = source_run_dir / "manifest.json"
    episode_path = episode_dir / "episode.json"
    complete_path = episode_dir / COMPLETE_FILE
    if not manifest_path.is_file():
        errors.append("manifest.json is missing")
    if not episode_path.is_file():
        errors.append("episode.json is missing")
    manifest = _read_json(manifest_path) if manifest_path.is_file() else {}
    episode = _read_json(episode_path) if episode_path.is_file() else {}
    complete = _read_json(complete_path) if complete_path.is_file() else None

    if manifest and manifest.get("contract_version") != CONTRACT_VERSION:
        errors.append("manifest contract_version mismatch")
    if episode and episode.get("contract_version") != CONTRACT_VERSION:
        errors.append("episode contract_version mismatch")
    if episode.get("episode_id") != episode_id:
        errors.append("episode.json episode_id mismatch")
    if episode.get("run_id") != effective_run_id:
        errors.append("episode.json run_id mismatch")

    operator_kind = episode.get("operator_kind")
    if operator_kind not in {item.value for item in OperatorKind}:
        errors.append(f"invalid operator_kind: {operator_kind!r}")
    if not episode.get("task_language"):
        warnings.append("task_language is empty")
    if not episode.get("view_source"):
        errors.append("view_source is missing")

    events = _read_jsonl(episode_dir / "events.jsonl", errors)
    commands = _read_jsonl(episode_dir / "commands.jsonl", errors)
    control = _read_jsonl(episode_dir / "control.jsonl", errors)
    transforms = _read_jsonl(episode_dir / "transforms.jsonl", errors)
    observations: list[dict[str, Any]] = []
    for path in sorted((episode_dir / "observations").glob("*.jsonl")):
        if path.name in {"index.jsonl", "persist.jsonl"}:
            continue
        observations.extend(_read_jsonl(path, errors))

    if complete is None:
        errors.append("episode_complete.json is missing (incomplete tail)")
    else:
        if complete.get("audit_status") != AuditStatus.COMPLETE.value:
            errors.append(
                f"completion tail records audit_status={complete.get('audit_status')!r}"
            )
        if complete.get("episode_id") != episode_id:
            errors.append("episode_complete.json episode_id mismatch")
        if complete.get("recording_segment_id") not in {
            segment.get("recording_segment_id") for segment in episode.get("segments", [])
        }:
            errors.append("episode_complete.json segment is not registered in episode.json")
        terminal_rows = [event for event in events if event.get("event_type") == TERMINAL_EVENT]
        if not terminal_rows:
            errors.append("terminal event is missing")
        elif terminal_rows[-1].get("payload", {}).get("recording_segment_id") != complete.get(
            "recording_segment_id"
        ):
            errors.append("terminal event segment does not match completion record")
        elif terminal_rows[-1].get("payload", {}).get("audit_status") != AuditStatus.COMPLETE.value:
            errors.append("terminal event marks the recording as incomplete/failed")

        expected_checksums = complete.get("checksums") or {}
        mutable = {"episode.json", "quality_report.json"}
        for relative, expected in expected_checksums.items():
            if relative in mutable:
                continue
            path = (episode_dir / relative).resolve()
            if not path.is_relative_to(episode_dir.resolve()):
                errors.append(f"checksum path escapes Episode: {relative}")
            elif not path.is_file():
                errors.append(f"checksummed file is missing: {relative}")
            elif _sha256(path) != expected:
                errors.append(f"checksum mismatch: {relative}")

    for index, event in enumerate(events, start=1):
        if event.get("contract_version") != CONTRACT_VERSION:
            errors.append(f"event {index}: contract_version mismatch")
        if event.get("run_id") != effective_run_id or event.get("episode_id") != episode_id:
            errors.append(f"event {index}: cross-Episode identity")
        if event.get("event_seq") != index:
            errors.append(f"event {index}: event_seq is not append-order contiguous")
        if event.get("event_type") in {"dispatch", "dispatched", "actuator_reference", "feedback"} and not event.get(
            "control_tick_id"
        ):
            errors.append(f"event {index}: {event.get('event_type')} lacks control_tick_id")
        if event.get("event_type") == "requested":
            request = (event.get("payload") or {}).get("request")
            if not isinstance(request, dict):
                warnings.append(
                    f"event {index}: requested payload lacks the complete raw CommandRequest"
                )

    for index, observation in enumerate(observations, start=1):
        if observation.get("contract_version") != CONTRACT_VERSION:
            errors.append(f"observation {index}: contract_version mismatch")
        if observation.get("run_id") != effective_run_id or observation.get("episode_id") != episode_id:
            errors.append(f"observation {index}: cross-Episode identity")
        if observation.get("quality") not in {
            "valid",
            "simulated_nominal",
            "simulated_raycast",
        }:
            warnings.append(f"observation {index}: quality={observation.get('quality')}")

    # Commands are correlated by command_id, never by feedback as an action label.
    requested_ids = {
        event.get("command_id")
        for event in events
        if event.get("event_type") == "requested" and event.get("command_id")
    }
    terminal_command_events = [
        event
        for event in events
        if event.get("command_id")
        and event.get("event_type")
        in {"dispatch", "dispatched", "approved", "rejected", "expired", "command_response"}
    ]
    for event in terminal_command_events:
        command_id = event.get("command_id")
        if requested_ids and command_id not in requested_ids:
            warnings.append(f"command {command_id} has lifecycle output but no requested event")

    for event in events:
        if _contains_world_truth(event.get("payload") or {}):
            errors.append(
                f"default event {event.get('event_type')} contains world_sim truth"
            )
    for observation in observations:
        if _contains_world_truth(observation):
            errors.append(
                f"default observation {observation.get('stream_id')}:{observation.get('sample_id')} "
                "contains world_sim truth"
            )

    # Per-stream measurement causality; event_seq itself need not track time.
    by_stream: dict[tuple[str, str], list[float]] = defaultdict(list)
    by_sensor_stream: dict[tuple[str, str, str], list[float]] = defaultdict(list)
    for observation in observations:
        measurement = observation.get("measurement_time_s")
        if not _is_finite_number(measurement):
            errors.append(
                f"observation {observation.get('stream_id')}:{observation.get('sample_id')} "
                "has non-finite measurement_time_s"
            )
            continue
        key = (str(observation.get("stream_id")), str(observation.get("clock_id")))
        by_stream[key].append(float(measurement))
        by_sensor_stream[
            (
                str(observation.get("stream_id")),
                str(observation.get("sensor_kind")),
                str(observation.get("clock_id")),
            )
        ].append(float(measurement))
        arrival = observation.get("arrival_time_s")
        if _is_finite_number(arrival) and float(arrival) + 1e-6 < float(measurement):
            errors.append(f"observation stream {key[0]} arrives before its measurement time")
        metadata = observation.get("metadata") or {}
        for key_name in ("T_base_link4", "T_link4_camera", "T_base_camera"):
            if key_name in metadata and not metadata.get(key_name):
                errors.append(f"observation stream {key[0]} has incomplete {key_name}")
        if observation.get("sensor_kind") in {"rgb", "depth"}:
            required_transform = ("T_base_link4", "T_link4_camera")
            if not all(metadata.get(key) for key in required_transform):
                errors.append(f"observation stream {key[0]} lacks image-time FK transforms")
            if not any(
                metadata.get(name)
                for name in ("T_base_camera", "T_base_camera_optical", "T_base_camera_render")
            ):
                errors.append(f"observation stream {key[0]} lacks image-time camera transform")
        if observation.get("sensor_kind") == "depth":
            depth_unit = metadata.get("depth_unit")
            depth_convention = str(metadata.get("depth_convention", "")).lower()
            metric = depth_unit in {"m", "meter", "metre"} or "metre" in depth_convention
            if not metric:
                errors.append(f"observation stream {key[0]} depth is not explicitly metric")
            if not observation.get("valid_mask_uri"):
                errors.append(f"observation stream {key[0]} depth lacks an explicit valid mask")

    rgbd_samples: dict[tuple[str, str], set[str]] = defaultdict(set)
    for observation in observations:
        kind = observation.get("sensor_kind")
        if kind in {"rgb", "depth"}:
            rgbd_samples[
                (str(observation.get("stream_id")), str(observation.get("sample_id")))
            ].add(str(kind))
    for (stream, sample_id), kinds in rgbd_samples.items():
        if kinds != {"rgb", "depth"}:
            missing = sorted({"rgb", "depth"} - kinds)
            errors.append(
                f"RGB-D sample {stream}:{sample_id} missing {','.join(missing)} pair"
            )

    actual_rates: dict[str, float] = {}
    max_gap: dict[str, float] = {}
    coverage: dict[str, float] = {}
    for (stream, kind, _clock), times in by_sensor_stream.items():
        if len(times) != len(set(times)):
            errors.append(
                f"observation sensor stream {stream}:{kind} has duplicate measurement timestamps"
            )
    for key, times in by_stream.items():
        times.sort()
        stream = key[0]
        if len(times) < 2:
            actual_rates[stream] = 0.0 if not times else 1.0
            coverage[stream] = 1.0
            continue
        gaps = [b - a for a, b in zip(times, times[1:])]
        positive = [gap for gap in gaps if gap > 0]
        if not positive:
            actual_rates[stream] = 0.0
        else:
            actual_rates[stream] = round(1.0 / (sum(positive) / len(positive)), 6)
        max_gap[stream] = round(max(gaps), 6)
        expected = actual_rates.get(stream, 0.0) * (times[-1] - times[0])
        coverage[stream] = round(min(1.0, len(times) / expected), 6) if expected > 0 else 1.0

    image_hashes: dict[str, tuple[str, str]] = {}
    for observation in observations:
        metadata = observation.get("metadata") or {}
        asset_uri = observation.get("asset_uri")
        declared_sha = metadata.get("asset_sha256")
        if isinstance(asset_uri, str) and isinstance(declared_sha, str):
            asset_path = (episode_dir / asset_uri).resolve()
            if (
                asset_path.is_relative_to(episode_dir.resolve())
                and asset_path.is_file()
                and _sha256(asset_path) != declared_sha
            ):
                errors.append(f"asset hash mismatch: {asset_uri}")
        if observation.get("sensor_kind") != "rgb":
            continue
        uri = asset_uri
        if not isinstance(uri, str):
            continue
        path = episode_dir / uri
        if path.is_file():
            digest = _sha256(path)
            previous = image_hashes.get(digest)
            if previous is not None:
                previous_sample, previous_uri = previous
                if previous_uri == uri:
                    errors.append(
                        f"RGB asset URI {uri} is reused by samples "
                        f"{previous_sample} and {observation.get('sample_id')}"
                    )
                # Pixel-identical files may legitimately be captured while the
                # scene is stationary; distinct URIs prove separate captures.
                # Measurement order is validated above.
            image_hashes[digest] = (
                str(observation.get("sample_id")),
                uri,
            )

    references = _asset_references(episode, events, observations)
    for relative, sources in references.items():
        if relative.startswith(("http://", "https://", "/api/")):
            warnings.append(f"external asset reference requires resolver: {relative}")
            continue
        path = (episode_dir / relative).resolve()
        if not path.is_relative_to(episode_dir.resolve()):
            errors.append(f"asset path escapes Episode: {relative}")
        elif not path.is_file():
            errors.append(f"asset missing: {relative} referenced by {', '.join(sources[:3])}")
        elif path.stat().st_size == 0:
            errors.append(f"asset is empty: {relative}")

    if not observations:
        warnings.append("Episode contains no sensor observations")
    if not steps_have_base_or_arm_reference(control):
        warnings.append("control.jsonl has no actuator_reference records")

    counts = {
        "events": len(events),
        "commands": len(commands),
        "control": len(control),
        "observations": len(observations),
        "transforms": len(transforms),
        "assets_referenced": len(references),
        "segments": len(episode.get("segments", [])),
    }
    audit = AuditStatus.INCOMPLETE if errors else AuditStatus.COMPLETE
    report = ValidationReport(
        episode_id=episode_id,
        audit_status=audit,
        passed=not errors,
        errors=errors,
        warnings=warnings,
        counts=counts,
        actual_rates_hz=actual_rates,
        max_gap_s=max_gap,
        coverage=coverage,
        generated_wall_utc=_utc_now(),
    )
    if write_report:
        _atomic_json(
            episode_dir / "quality_report.json",
            {
                **report.model_dump(mode="json"),
                "recording_status": episode.get("recording_status", "UNKNOWN"),
                "termination_reason": episode.get("termination_reason"),
            },
        )
    return report


def steps_have_base_or_arm_reference(rows: Iterable[Mapping[str, Any]]) -> bool:
    return any(
        row.get("event_type") == "actuator_reference"
        or (row.get("payload") or {}).get("actuator_reference")
        for row in rows
    )
