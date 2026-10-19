from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path

from mobilecar.contracts import EpisodeStartRequest, EventRecord, RobotSnapshot
from mobilecar.data import Recorder, assess_grasp_oracle, evaluate_eligibility
from mobilecar.data.validator import validate_episode


ROOT = Path(__file__).resolve().parents[2]


def _samples(lifted: bool):
    rows = [
        {
            "sim_time_s": 0.0,
            "cube_position_world_sim_m": [0.4, 0.0, 0.112],
            "left_contact_force_n": 0.0,
            "right_contact_force_n": 0.0,
            "bilateral_contact": False,
        }
    ]
    for index in range(1, 12):
        time_s = index * 0.25
        z = 0.112 + (0.058 if lifted else 0.0)
        rows.append(
            {
                "sim_time_s": time_s,
                "cube_position_world_sim_m": [0.4, 0.0, z],
                "left_contact_force_n": 0.4,
                "right_contact_force_n": 0.4,
                "bilateral_contact": True,
            }
        )
    return rows


def test_oracle_requires_physical_lift_and_hold():
    success = assess_grasp_oracle(
        _samples(True),
        "fixture_episode",
        lift_threshold_m=0.05,
        hold_threshold_s=2.0,
        write=False,
    )
    assert success.task_result == "success"
    assert success.lift_height_m > 0.05
    assert success.hold_duration_s >= 2.0
    assert success.bilateral_contact_fraction > 0.9
    failed = assess_grasp_oracle(_samples(False), "fixture_episode", write=False)
    assert failed.task_result == "failure"


def test_eligibility_separates_agent_source_and_audit(tmp_path):
    # Reuse the normal recorder fixture shape through a compact hand-built Episode.
    episode_dir = tmp_path / "fixture_run" / "episodes" / "fixture_episode"
    (episode_dir / "observations").mkdir(parents=True)
    (episode_dir / "assessments").mkdir()
    (episode_dir / "assets/rgb").mkdir(parents=True)
    (episode_dir / "assets/rgb/frame.png").write_bytes(b"rgb")
    (tmp_path / "fixture_run/manifest.json").write_text(
        json.dumps({"contract_version": "mobilecar-1.0", "run_id": "fixture_run"}),
        encoding="utf-8",
    )
    episode = {
        "contract_version": "mobilecar-1.0",
        "run_id": "fixture_run",
        "episode_id": "fixture_episode",
        "task_language": "pick",
        "operator_kind": "agent_ui",
        "view_source": "third_person_3d",
        "segments": [{"recording_segment_id": "seg_1"}],
        "result": "success",
    }
    (episode_dir / "episode.json").write_text(json.dumps(episode), encoding="utf-8")
    (episode_dir / "quality_report.json").write_text(
        json.dumps(
            {
                "contract_version": "mobilecar-1.0",
                "episode_id": "fixture_episode",
                "audit_status": "COMPLETE",
                "passed": True,
                "errors": [],
                "warnings": [],
                "generated_wall_utc": "2026-10-17T00:00:00Z",
            }
        ),
        encoding="utf-8",
    )
    (episode_dir / "assessments/task_assessment.json").write_text(
        json.dumps(
            {
                "contract_version": "mobilecar-1.0",
                "episode_id": "fixture_episode",
                "task_result": "success",
                "oracle_source": "fixture",
            }
        ),
        encoding="utf-8",
    )
    action = {
        "event_type": "actuator_reference",
        "payload": {"base": {"vx_m_s": 0.02}},
        "task_phase": "APPROACH",
    }
    with (episode_dir / "events.jsonl").open("w", encoding="utf-8") as handle:
        handle.write(json.dumps(action) + "\n")
    with (episode_dir / "observations/index.jsonl").open("w", encoding="utf-8") as handle:
        handle.write(
            json.dumps(
                {
                    "sensor_kind": "rgb",
                    "asset_uri": "assets/rgb/frame.png",
                    "sample_id": "frame",
                }
            )
            + "\n"
        )
        handle.write(
            json.dumps(
                {
                    "sensor_kind": "joint_feedback",
                    "sample_id": "joints",
                }
            )
            + "\n"
        )
    result = evaluate_eligibility(tmp_path, "fixture_episode")
    assert result.d2_near_field_manipulation
    assert result.source_operator_kind == "agent_ui"
    assert any("agent_ui" in reason for reason in result.reasons)
    assert not result.wrist_rgb_only_eligible


def test_oracle_samples_follow_isolated_recording_path(tmp_path):
    snapshot = RobotSnapshot.model_validate_json(
        (ROOT / "configs/examples/snapshot_fixture.json").read_text(encoding="utf-8")
    )
    snapshot.diagnostics["object_pose_world_sim"] = {"position_m": [0.4, 0.0, 0.112]}
    recorder = Recorder(root=tmp_path, queue_capacity=64)
    assert recorder.start(
        EpisodeStartRequest(
            operator_id="oracle_fixture",
            operator_kind="agent_ui",
            view_source="third_person_3d",
        ),
        snapshot,
    ).accepted
    for time_s, z, contact in ((1.0, 0.17, True), (3.2, 0.17, True)):
        recorder.publish(
            EventRecord(
                run_id=snapshot.run_id,
                episode_id=snapshot.episode_id,
                event_type="oracle_sample",
                wall_utc=datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                sim_time_s=time_s,
                payload={
                    "cube_position_world_sim_m": [0.4, 0.0, z],
                    "left_contact_force_n": 0.5,
                    "right_contact_force_n": 0.5,
                    "bilateral_contact": contact,
                },
            )
        )
    assert recorder.stop(final_snapshot=snapshot).accepted
    samples = (
        tmp_path
        / snapshot.run_id
        / "episodes"
        / snapshot.episode_id
        / "assessments/oracle_samples.jsonl"
    )
    assert len(samples.read_text(encoding="utf-8").splitlines()) >= 4
    assessment = assess_grasp_oracle(tmp_path, snapshot.episode_id)
    assert assessment.task_result == "success"
    episode_metadata = json.loads(
        (
            tmp_path
            / snapshot.run_id
            / "episodes"
            / snapshot.episode_id
            / "episode.json"
        ).read_text(encoding="utf-8")
    )
    assert episode_metadata["result"] == "success"
    assert episode_metadata["task_result_source"] == "world_sim_isolated_oracle_v1"
    event_types = {
        json.loads(event)["event_type"]
        for event in (
            tmp_path
            / snapshot.run_id
            / "episodes"
            / snapshot.episode_id
            / "events.jsonl"
        ).read_text(encoding="utf-8").splitlines()
    }
    assert not ({"oracle_sample", "world_state_sample", "grasp_oracle_sample"} & event_types)
    initial = json.loads(
        (
            tmp_path
            / snapshot.run_id
            / "episodes"
            / snapshot.episode_id
            / "initial_snapshot.json"
        ).read_text(encoding="utf-8")
    )
    assert "object_pose_world_sim" not in initial["diagnostics"]
    recorder.close()
