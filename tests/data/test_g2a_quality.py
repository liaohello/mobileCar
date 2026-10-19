from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np

from mobilecar.contracts import AuditStatus, ValidationReport
from mobilecar.data import (
    assess_g2a_quality,
    assess_g2a_eligibility,
    g2a_eligibility,
    policy_observation_from_records,
)


ROOT = Path(__file__).resolve().parents[2]
PROFILE = ROOT / "configs/profiles/g2a_rgbd_5hz.yaml"
ACTION_PROFILE = ROOT / "configs/profiles/g2a_action_teleop_v1.yaml"


def _append(path: Path, rows: list[dict]):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row) + "\n")


def _validation(episode_id: str) -> ValidationReport:
    return ValidationReport(
        episode_id=episode_id,
        audit_status=AuditStatus.COMPLETE,
        passed=True,
        errors=[],
        warnings=[],
        counts={"observations": 1},
        generated_wall_utc="2026-10-17T00:00:00Z",
    )


def _episode(
    tmp_path: Path,
    *,
    times: list[float],
    missing_depth_index: int | None = None,
    identical_rgb: bool = False,
) -> tuple[Path, str, str]:
    run_id = "run_g2a_fixture"
    episode_id = "episode_g2a_fixture"
    episode_dir = tmp_path / run_id / "episodes" / episode_id
    for child in ("observations", "assets/rgb", "assets/depth", "assets/masks", "assessments"):
        (episode_dir / child).mkdir(parents=True, exist_ok=True)
    (tmp_path / run_id / "manifest.json").write_text(
        json.dumps({"contract_version": "mobilecar-1.0", "run_id": run_id}),
        encoding="utf-8",
    )
    (episode_dir / "episode.json").write_text(
        json.dumps(
            {
                "contract_version": "mobilecar-1.0",
                "run_id": run_id,
                "episode_id": episode_id,
                "task_language": "quality fixture",
                "operator_kind": "agent_ui",
                "view_source": "wrist_rgb_only",
                "segments": [{"recording_segment_id": "seg_1"}],
                "result": "success",
            }
        ),
        encoding="utf-8",
    )
    (episode_dir / "episode_complete.json").write_text(
        json.dumps({"audit_status": "COMPLETE"}), encoding="utf-8"
    )
    transforms = [
        [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
    ]
    index: list[dict] = []
    rgb_rows: list[dict] = []
    depth_rows: list[dict] = []
    persist_rows: list[dict] = []
    timing_events: list[dict] = []
    for index_value, capture_sim in enumerate(times):
        sample_id = f"sample_{index_value:04d}"
        capture_mono = 100.0 + capture_sim
        arrival = capture_mono + 0.02
        persist = capture_mono + 0.05
        rgb_uri = f"assets/rgb/{sample_id}.png"
        depth_uri = f"assets/depth/{sample_id}.npy"
        mask_uri = f"assets/masks/{sample_id}.png"
        rgb_payload = b"same-rgb" if identical_rgb else f"rgb-{index_value}".encode()
        (episode_dir / rgb_uri).write_bytes(rgb_payload)
        np.save(episode_dir / depth_uri, np.full((2, 2), 1.0 + index_value, dtype=np.float32))
        (episode_dir / mask_uri).write_bytes(f"mask-{index_value}".encode())
        metadata = {
            "transform_version": "g2a-transform-1",
            "T_base_link4": transforms[0],
            "T_link4_camera": transforms[0],
            "T_base_camera": transforms[0],
            "depth_unit": "m",
            "profile_id": "g2a-rgbd-5hz-original",
            "profile_version": "1.0",
            "profile_sha256": hashlib.sha256(PROFILE.read_bytes()).hexdigest(),
        }
        timing = {
            "sample_id": sample_id,
            "capture_backend_monotonic_s": capture_mono,
            "render_start_backend_monotonic_s": capture_mono + 0.002,
            "render_complete_backend_monotonic_s": capture_mono + 0.01,
            "publish_backend_monotonic_s": capture_mono + 0.015,
            "arrival_backend_monotonic_s": arrival,
            "lock_wait_ms": 1.0,
            "render_ms": 8.0,
            "encode_ms": 2.0,
        }
        common = {
            "contract_version": "mobilecar-1.0",
            "run_id": run_id,
            "episode_id": episode_id,
            "stream_id": "rgbd",
            "sample_id": sample_id,
            "measurement_time_s": capture_mono,
            "arrival_time_s": arrival,
            "clock_id": "backend_monotonic",
            "frame_id": "rgbd_optical",
            "source": "fixture",
            "quality": "valid",
            "metadata": metadata,
            "capture_sim_time_s": capture_sim,
            "capture_backend_monotonic_s": capture_mono,
            "capture_control_tick_id": f"tick_{index_value:09d}",
            "feedback_sample_id": f"joint_{index_value:09d}",
            "timing": timing,
        }
        rgb = {**common, "sensor_kind": "rgb", "asset_uri": rgb_uri}
        index.append(rgb)
        rgb_rows.append(rgb)
        if missing_depth_index != index_value:
            depth = {
                **common,
                "sensor_kind": "depth",
                "asset_uri": depth_uri,
                "valid_mask_uri": mask_uri,
            }
            index.append(depth)
            depth_rows.append(depth)
            persist_rows.append(
                {
                    "run_id": run_id,
                    "episode_id": episode_id,
                    "stream_id": "rgbd",
                    "sample_id": sample_id,
                    "capture_sim_time_s": capture_sim,
                    "capture_backend_monotonic_s": capture_mono,
                    "arrival_backend_monotonic_s": arrival,
                    "persist_backend_monotonic_s": persist,
                    "capture_control_tick_id": common["capture_control_tick_id"],
                    "feedback_sample_id": common["feedback_sample_id"],
                    "rgb_uri": rgb_uri,
                    "depth_uri": depth_uri,
                    "valid_mask_uri": mask_uri,
                    "profile_id": metadata["profile_id"],
                    "profile_version": metadata["profile_version"],
                    "profile_sha256": metadata["profile_sha256"],
                    "arrival_to_persist_ms": 30.0,
                    "capture_to_persist_ms": 50.0,
                }
            )
        timing_events.append(
            {
                "event_type": "rgbd_capture_timing",
                "payload": timing,
            }
        )
    index.append(
        {
            "contract_version": "mobilecar-1.0",
            "run_id": run_id,
            "episode_id": episode_id,
            "stream_id": "joint_feedback",
            "sample_id": "joint_fixture",
            "sensor_kind": "joint_feedback",
            "measurement_time_s": 100.0,
            "arrival_time_s": 100.01,
            "clock_id": "backend_monotonic",
            "frame_id": "arm_base",
            "source": "fixture",
            "quality": "valid",
            "value": {
                "user_joint_positions_rad": [0.0] * 5,
                "gripper_width_m": 0.03,
            },
        }
    )
    _append(episode_dir / "observations/index.jsonl", index)
    _append(episode_dir / "observations/rgb.jsonl", rgb_rows)
    _append(episode_dir / "observations/depth.jsonl", depth_rows)
    _append(episode_dir / "observations/persist.jsonl", persist_rows)
    _append(
        episode_dir / "events.jsonl",
        [
            {
                "event_type": "recording_start",
                "backend_monotonic_s": 100.0,
                "sim_time_s": 0.0,
            },
            {
                "event_type": "requested",
                "command_id": "base_command",
                "payload": {
                    "request": {
                        "command_type": "set_base_twist",
                        "payload": {
                            "vx_m_s": 0.01,
                            "vy_m_s": 0.0,
                            "wz_rad_s": 0.0,
                        },
                    }
                },
            },
            {
                "event_type": "approved",
                "command_id": "base_command",
                "payload": {"base": "twist"},
            },
            {
                "event_type": "requested",
                "command_id": "arm_command",
                "payload": {
                    "request": {
                        "command_type": "set_arm_joint_goal",
                        "payload": {"joint_positions_rad": [0.0] * 5},
                    }
                },
            },
            {
                "event_type": "approved",
                "command_id": "arm_command",
                "payload": {"arm": "joint_goal"},
            },
            {
                "event_type": "dispatch",
                "command_id": "arm_command",
                "payload": {"kind": "arm_joint_goal"},
            },
            {
                "event_type": "actuator_reference",
                "payload": {
                    "arm_model_reference_rad": [0.0] * 5,
                    "base_twist": [0.0, 0.0, 0.0],
                },
            },
            *timing_events,
            {
                "event_type": "recording_stop",
                "backend_monotonic_s": 160.0,
                "sim_time_s": 60.0,
                "payload": {"snapshot": {"diagnostics": {"rgbd_schedule": {"skip_reasons": {}}}}},
            },
        ],
    )
    _append(
        episode_dir / "transforms.jsonl",
        [{"transform_version": "g2a-transform-1"}],
    )
    (episode_dir / "quality_report.json").write_text(
        json.dumps({"audit_status": "COMPLETE"}), encoding="utf-8"
    )
    (episode_dir / "assessments/task_assessment.json").write_text(
        json.dumps({"task_result": "success"}), encoding="utf-8"
    )
    return tmp_path, episode_id, run_id


def _times() -> list[float]:
    return [
        *(0.2 * value for value in range(1, 25)),
        *(5.0 + 0.2 * value for value in range(250)),
        *(55.0 + 0.2 * value for value in range(25)),
    ]


def test_g2a_quality_uses_sample_ids_and_reports_all_windows(tmp_path):
    root, episode_id, run_id = _episode(tmp_path, times=_times())
    report = assess_g2a_quality(
        root,
        episode_id,
        run_id=run_id,
        profile_path=PROFILE,
        action_profile_path=ACTION_PROFILE,
        output_root=tmp_path / "reports",
        validation=_validation(episode_id),
    )
    assert report.passed, report.gate_reasons
    assert report.timing.persisted_samples == 250
    assert report.timing.unique_frames == 250
    assert report.timing.wall_rate_hz >= 4.5
    assert report.timing.window_metrics["init"]["unique"] == 24
    assert report.timing.window_metrics["analysis"]["unique"] == 250
    assert report.timing.window_metrics["final"]["unique"] == 25
    assert report.action_profile_id == "g2a-teleop-action-1.0"
    assert len(report.input_sha256) == 64
    assert "assets/**" in report.input_hash_scope


def test_pixel_identical_stationary_frames_remain_distinct_samples(tmp_path):
    root, episode_id, run_id = _episode(
        tmp_path,
        times=[5.0, 5.2],
        identical_rgb=True,
    )
    report = assess_g2a_quality(
        root,
        episode_id,
        run_id=run_id,
        profile_path=PROFILE,
        action_profile_path=ACTION_PROFILE,
        output_root=tmp_path / "reports",
        validation=_validation(episode_id),
    )
    assert report.timing.unique_frames == 2
    assert report.timing.window_metrics["asset_diagnostics"]["pixel_identical_groups"] == 1
    assert "duplicate_frame" not in report.timing.skip_reasons


def test_missing_depth_pair_fails_profile_quality(tmp_path):
    root, episode_id, run_id = _episode(
        tmp_path,
        times=_times(),
        missing_depth_index=30,
    )
    report = assess_g2a_quality(
        root,
        episode_id,
        run_id=run_id,
        profile_path=PROFILE,
        action_profile_path=ACTION_PROFILE,
        output_root=tmp_path / "reports",
        validation=_validation(episode_id),
    )
    assert not report.gates["rgb_depth_pairs"]
    assert not report.passed
    assert report.timing.skip_reasons["missing_pair"] == 1


def test_profile_bound_eligibility_allows_rgb_only_observations(tmp_path):
    root, episode_id, run_id = _episode(tmp_path, times=_times())
    quality, eligibility = assess_g2a_eligibility(
        root,
        episode_id,
        run_id=run_id,
        profile_path=PROFILE,
        action_profile_path=ACTION_PROFILE,
        output_root=tmp_path / "reports",
        validation=_validation(episode_id),
    )
    assert quality.passed
    assert eligibility.wrist_rgb_only_eligible
    assert eligibility.d2_near_field_manipulation
    assert eligibility.action_profile_sha256 == quality.action_profile_sha256
    assert eligibility.profile_sha256 == quality.profile_sha256


def test_stop_only_episode_does_not_qualify_as_demonstration(tmp_path):
    root, episode_id, run_id = _episode(tmp_path, times=_times())
    quality = assess_g2a_quality(
        root,
        episode_id,
        run_id=run_id,
        profile_path=PROFILE,
        action_profile_path=ACTION_PROFILE,
        output_root=tmp_path / "reports",
        validation=_validation(episode_id),
    )
    events = [
        {
            "event_type": "requested",
            "command_id": "stop_command",
            "payload": {
                "request": {
                    "command_type": "stop",
                    "payload": {"latch": True},
                }
            },
        },
        {
            "event_type": "approved",
            "command_id": "stop_command",
            "payload": {"base": "zero_and_brake", "arm": "cancel_and_hold"},
        },
        {"event_type": "actuator_reference", "payload": {"base_twist": [0, 0, 0]}},
    ]
    episode = json.loads(
        (root / run_id / "episodes" / episode_id / "episode.json").read_text()
    )
    observations = [
        {"sensor_kind": "rgb", "sample_id": "frame"},
        {"sensor_kind": "joint_feedback", "sample_id": "joint"},
    ]
    eligibility = g2a_eligibility(
        quality,
        episode,
        events,
        {"task_result": "failure"},
        observations,
    )
    assert not eligibility.d0_navigation
    assert not eligibility.d2_near_field_manipulation
    assert not eligibility.wrist_rgb_only_eligible


def test_policy_observation_rejects_world_truth_and_uses_allowlist():
    episode = {"view_source": "third_party_3d"}
    rgb = {
        "run_id": "run",
        "episode_id": "episode",
        "stream_id": "rgbd",
        "sample_id": "frame",
        "measurement_time_s": 1.0,
        "clock_id": "backend_monotonic",
        "frame_id": "rgbd_optical",
        "asset_uri": "assets/rgb/frame.png",
        "source": "mujoco_renderer",
        "quality": "valid",
        "capture_control_tick_id": "tick_1",
        "feedback_sample_id": "joint_1",
    }
    joint = {
        "sensor_kind": "joint_feedback",
        "value": {
            "user_joint_positions_rad": [0.0] * 5,
            "gripper_width_m": 0.03,
        },
    }
    encoders = {
        "sensor_kind": "encoders",
        "value": {"base_twist_body_m_s": [0.0, 0.0, 0.0]},
    }
    allowed = [
        "run_id",
        "episode_id",
        "stream_id",
        "sample_id",
        "measurement_time_s",
        "clock_id",
        "frame_id",
        "rgb_uri",
        "depth_uri",
        "valid_mask_uri",
        "base_twist_body_m_s",
        "arm_user_joint_positions_rad",
        "gripper_width_m",
        "available_sensors",
        "source",
        "view_source",
        "quality",
    ]
    policy = policy_observation_from_records(
        episode=episode,
        rgb=rgb,
        depth=None,
        joint_feedback=joint,
        encoders=encoders,
        allowed_keys=allowed,
    )
    payload = policy.model_dump(mode="json")
    assert payload["arm_user_joint_positions_rad"] == [0.0] * 5
    assert "capture_control_tick_id" not in payload
    blocked = dict(joint)
    blocked["value"] = {
        **joint["value"],
        "object_pose_world_sim": {"position_m": [1.0, 2.0, 3.0]},
    }
    try:
        policy_observation_from_records(
            episode=episode,
            rgb=rgb,
            depth=None,
            joint_feedback=blocked,
            encoders=encoders,
            allowed_keys=allowed,
        )
    except ValueError as exc:
        assert "world_sim" in str(exc)
    else:
        raise AssertionError("world_sim truth was accepted as policy input")
