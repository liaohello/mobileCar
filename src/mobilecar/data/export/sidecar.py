"""Per-row provenance and retained raw-depth references for G2B datasets."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
from typing import Any, Iterable
from uuid import uuid4

from .models import AlignmentResult, AlignedRow, ExportProfile, sha256_file
from .reader import EpisodeReader


def sha256_path(path: str | Path) -> str:
    return sha256_file(path)


def write_json(path: str | Path, value: Any) -> Path:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(f".{output.name}.{uuid4().hex}.tmp")
    temporary.write_text(
        json.dumps(value, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, output)
    return output


def write_jsonl(path: str | Path, rows: Iterable[dict[str, Any]]) -> Path:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(f".{output.name}.{uuid4().hex}.tmp")
    with temporary.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(
                json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n"
            )
    os.replace(temporary, output)
    return output


def _row_sidecar(
    *,
    profile_name: str,
    export_episode_index: int,
    frame_index: int,
    row: AlignedRow,
    reader: EpisodeReader,
    profile: ExportProfile,
) -> dict[str, Any]:
    issued = row.profiles[profile_name]
    feedback = row.joint_feedback
    encoder = row.encoder
    imu = row.imu
    reference = row.actuator_reference
    return {
        "schema_version": 1,
        "profile_id": profile.profile_id,
        "profile_sha256": profile.sha256,
        "rule_sha256": row_safe_rule_hash(profile),
        "export_profile": profile_name.upper(),
        "split": "dev",
        "run_id": reader.run_id,
        "parent_episode_id": reader.episode_id,
        "recording_segment_id": row.image.recording_segment_id,
        "source": reader.source,
        "operator_kind": reader.episode.get("operator_kind"),
        "view_source": reader.episode.get("view_source"),
        "source_task_language": reader.task_language,
        "source_task_result": reader.task_result,
        "source_audit_status": reader.audit_status,
        "export_episode_index": export_episode_index,
        "frame_index": frame_index,
        "dataset_timestamp_s": frame_index / profile.grid_rate_hz,
        "grid_index": row.grid_index,
        "grid_sim_time_s": row.decision_sim_time_s,
        "decision_sim_time_s": row.decision_sim_time_s,
        "decision_backend_monotonic_s": row.decision_backend_monotonic_s,
        "clock_id": "backend_monotonic",
        "image": {
            "sample_id": row.image.sample_id,
            "stream_id": row.image.stream_id,
            "control_tick_id": row.image.control_tick_id,
            "feedback_sample_id": row.image.feedback_sample_id,
            "capture_sim_time_s": row.image.capture_sim_time_s,
            "capture_backend_monotonic_s": row.image.capture_backend_monotonic_s,
            "render_publish_backend_monotonic_s": row.image.publish_backend_monotonic_s,
            "arrival_backend_monotonic_s": row.image.arrival_backend_monotonic_s,
            "persist_backend_monotonic_s": row.image.persist_backend_monotonic_s,
            "capture_age_s": row.image_age_s,
            "arrival_age_s": row.image_arrival_age_s,
            "rgb_uri": row.image.rgb_uri,
            "depth_uri": row.image.depth_uri,
            "valid_mask_uri": row.image.valid_mask_uri,
            "width": row.image.width,
            "height": row.image.height,
        },
        "feedback": {
            "sample_id": feedback.sample_id,
            "control_tick_id": feedback.control_tick_id,
            "capture_sim_time_s": feedback.capture_sim_time_s,
            "capture_backend_monotonic_s": feedback.capture_backend_monotonic_s,
            "arrival_backend_monotonic_s": feedback.arrival_backend_monotonic_s,
            "capture_age_s": row.feedback_age_s,
            "image_capture_skew_s": row.image_feedback_capture_skew_s,
        },
        "encoder": (
            {
                "sample_id": encoder.sample_id,
                "control_tick_id": encoder.control_tick_id,
                "capture_sim_time_s": encoder.capture_sim_time_s,
                "capture_backend_monotonic_s": encoder.capture_backend_monotonic_s,
                "arrival_backend_monotonic_s": encoder.arrival_backend_monotonic_s,
            }
            if encoder is not None
            else None
        ),
        "imu": (
            {
                "sample_id": imu.sample_id,
                "control_tick_id": imu.control_tick_id,
                "capture_sim_time_s": imu.capture_sim_time_s,
                "capture_backend_monotonic_s": imu.capture_backend_monotonic_s,
                "arrival_backend_monotonic_s": imu.arrival_backend_monotonic_s,
            }
            if imu is not None
            else None
        ),
        "action_reference": {
            "layer": "actuator_reference",
            "control_tick_id": reference.control_tick_id,
            "backend_monotonic_s": reference.backend_monotonic_s,
            "sim_time_s": reference.sim_time_s,
            "payload_sha256": hashlib.sha256(
                json.dumps(
                    reference.payload,
                    sort_keys=True,
                    separators=(",", ":"),
                ).encode("utf-8")
            ).hexdigest(),
        },
        "dispatch_provenance": issued.dispatch_provenance,
        "effective_intervals_sim_time_s": issued.effective_intervals,
        "action_profile": {
            "layer": profile.action_spec(profile_name)["layer"],
            "frame_id": issued.action_frame_id,
            "units": issued.action_units,
            "joint_order": (
                profile.action_spec("d2").get("joint_order")
                if profile_name == "d2"
                else None
            ),
            "zero_model_rad": (
                profile.action_spec("d2").get("user_zero_model_rad")
                if profile_name == "d2"
                else None
            ),
            "direction": (
                profile.action_spec("d2").get("user_direction")
                if profile_name == "d2"
                else None
            ),
            "model_sha256": profile.model_sha256,
            "robot_config_sha256": profile.robot_config_sha256,
            "arm_spec_sha256": profile.arm_spec_sha256,
        },
        "observation_state": issued.state,
        "action": issued.action,
        "selection": {
            "persisted_is_durability_not_online_arrival": True,
            "arrival_boundary": "arrival_backend_monotonic_s <= decision",
            "future_interpolation": False,
            "old_image_reuse": False,
            "reasons": row.reasons,
        },
    }


def row_safe_rule_hash(profile: ExportProfile) -> str:
    import hashlib

    payload = json.dumps(
        {
            "profile_sha256": profile.sha256,
            "causality": profile.raw["causality"],
            "actions": profile.raw["actions"],
            "features": profile.raw["features"],
        },
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _depth_artifacts(
    reader: EpisodeReader,
    rows: Iterable[AlignedRow],
) -> list[dict[str, Any]]:
    seen: set[tuple[str, str]] = set()
    result: list[dict[str, Any]] = []
    for row in rows:
        for kind, uri in (
            ("depth", row.image.depth_uri),
            ("valid_mask", row.image.valid_mask_uri),
        ):
            key = (kind, uri)
            if key in seen:
                continue
            seen.add(key)
            result.append(
                {
                    "sample_id": row.image.sample_id,
                    "kind": kind,
                    "source_uri_relative_to_episode": uri,
                    "sha256": reader.asset_hash(uri),
                }
            )
    return result


def write_dataset_sidecar(
    *,
    dataset_root: str | Path,
    reader: EpisodeReader,
    alignment: AlignmentResult,
    profile: ExportProfile,
    profile_name: str,
    segment_row_counts: list[int],
    source_file_hashes_before: dict[str, str],
    source_file_hashes_after: dict[str, str] | None = None,
    environment: dict[str, Any],
    loader_result: dict[str, Any],
) -> dict[str, Any]:
    root = Path(dataset_root)
    sidecar_root = root / "g2b"
    rows = alignment.rows_for(profile_name)
    segments = alignment.contiguous_segments(profile_name)
    sidecar_rows: list[dict[str, Any]] = []
    for episode_index, segment in enumerate(segments):
        for frame_index, row in enumerate(segment):
            sidecar_rows.append(
                _row_sidecar(
                    profile_name=profile_name,
                    export_episode_index=episode_index,
                    frame_index=frame_index,
                    row=row,
                    reader=reader,
                    profile=profile,
                )
            )
    if [len(item) for item in segments] != segment_row_counts:
        raise ValueError("written segment lengths do not match aligned segments")
    write_jsonl(sidecar_root / "sidecar.jsonl", sidecar_rows)
    write_jsonl(sidecar_root / "depth_artifacts.jsonl", _depth_artifacts(reader, rows))
    write_jsonl(sidecar_root / "rejections.jsonl", alignment.rejected)
    write_json(
        sidecar_root / "profile.json",
        {
            "profile_id": profile.profile_id,
            "profile_sha256": profile.sha256,
            "feature_profile": profile_name.upper(),
            "raw": profile.raw,
        },
    )
    write_json(
        sidecar_root / "source_episode.json",
        {
            **reader.identity(),
            "source_file_hashes_before": source_file_hashes_before,
            "source_file_hashes_after": source_file_hashes_after,
            "source_files_unchanged": (
                None
                if source_file_hashes_after is None
                else source_file_hashes_before == source_file_hashes_after
            ),
            "source_tree_sha256": hashlib.sha256(
                "".join(
                    f"{path} {digest}\n"
                    for path, digest in sorted(source_file_hashes_before.items())
                ).encode("utf-8")
            ).hexdigest(),
        },
    )
    write_json(
        sidecar_root / "alignment.json",
        {
            "summary": alignment.summary,
            "segments": [
                {
                    "episode_index": index,
                    "length": len(segment),
                    "first_grid_index": segment[0].grid_index,
                    "last_grid_index": segment[-1].grid_index,
                }
                for index, segment in enumerate(segments)
            ],
            "rejection_count": len(alignment.rejected),
        },
    )
    write_json(
        sidecar_root / "split.json",
        {
            "split_schema_version": 1,
            "parent_episode_id": reader.episode_id,
            "dev": list(range(len(segments))),
            "train": [],
            "validation": [],
            "reason": (
                "One real source Episode; all subsegments remain in dev. "
                "Empty train/validation is intentional and prevents parent leakage."
            ),
        },
    )
    write_json(
        sidecar_root / "loader_verification.json",
        loader_result,
    )
    write_json(
        sidecar_root / "environment.json",
        environment,
    )
    return {
        "sidecar_root": str(sidecar_root),
        "rows": len(sidecar_rows),
        "segments": len(segments),
        "sidecar_sha256": sha256_file(sidecar_root / "sidecar.jsonl"),
    }


def write_output_manifest(dataset_root: str | Path) -> dict[str, Any]:
    root = Path(dataset_root)
    files: list[dict[str, Any]] = []
    for path in sorted(root.rglob("*")):
        if not path.is_file() or path.name == "manifest.json":
            continue
        files.append(
            {
                "path": path.relative_to(root).as_posix(),
                "bytes": path.stat().st_size,
                "sha256": sha256_file(path),
            }
        )
    manifest = {
        "schema_version": 1,
        "dataset_root_name": root.name,
        "file_count": len(files),
        "total_bytes": sum(item["bytes"] for item in files),
        "files": files,
    }
    write_json(root / "manifest.json", manifest)
    return manifest
