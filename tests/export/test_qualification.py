"""Focused tests for the independent G2B qualification conclusions."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

from mobilecar.data.export.models import (
    ActuatorReference,
    AlignedRow,
    AlignmentResult,
    ImageSample,
    ObservationSample,
    ProfileRow,
    load_export_profile,
)
from mobilecar.data.export.qualification import (
    assess_export_valid,
    assess_train_qualified,
)


ROOT = Path(__file__).resolve().parents[2]
PROFILE_PATH = ROOT / "configs/export/g2b_export_v1.yaml"


def _row() -> AlignedRow:
    image = ImageSample(
        sample_id="rgbd_1",
        stream_id="rgbd",
        recording_segment_id="seg_1",
        capture_sim_time_s=1.0,
        capture_backend_monotonic_s=100.0,
        publish_backend_monotonic_s=100.05,
        arrival_backend_monotonic_s=100.2,
        persist_backend_monotonic_s=100.3,
        control_tick_id="tick_1",
        feedback_sample_id="joint_1",
        rgb_uri="assets/rgb.png",
        depth_uri="assets/depth.npy",
        valid_mask_uri="assets/mask.png",
        width=640,
        height=480,
        run_id="run_1",
        episode_id="episode_1",
    )
    feedback = ObservationSample(
        sample_id="joint_1",
        sensor_kind="joint_feedback",
        stream_id="joint_feedback",
        recording_segment_id="seg_1",
        control_tick_id="tick_1",
        capture_sim_time_s=1.0,
        capture_backend_monotonic_s=100.0,
        arrival_backend_monotonic_s=100.0,
        value={},
        run_id="run_1",
        episode_id="episode_1",
        feedback_sample_id="joint_1",
    )
    encoder = ObservationSample(
        sample_id="encoder_1",
        sensor_kind="encoders",
        stream_id="encoders",
        recording_segment_id="seg_1",
        control_tick_id="tick_1",
        capture_sim_time_s=1.0,
        capture_backend_monotonic_s=100.0,
        arrival_backend_monotonic_s=100.0,
        value={},
        run_id="run_1",
        episode_id="episode_1",
    )
    imu = ObservationSample(
        sample_id="imu_1",
        sensor_kind="imu",
        stream_id="imu",
        recording_segment_id="seg_1",
        control_tick_id="tick_1",
        capture_sim_time_s=1.0,
        capture_backend_monotonic_s=100.0,
        arrival_backend_monotonic_s=100.0,
        value={},
        run_id="run_1",
        episode_id="episode_1",
    )
    reference = ActuatorReference(
        sequence=1,
        backend_monotonic_s=100.0,
        sim_time_s=1.0,
        control_tick_id="tick_1",
        payload={},
    )
    d0 = ProfileRow(
        profile="d0",
        state=tuple([0.0] * 10),
        action=(0.01, 0.0, 0.0),
        action_units=("m/s", "m/s", "rad/s"),
        action_frame_id="base_link",
        dispatch_provenance={"base": {}},
        effective_intervals={"base": [1.0, 2.0]},
    )
    d2 = ProfileRow(
        profile="d2",
        state=tuple([0.0] * 6),
        action=tuple([0.0] * 6),
        action_units=("rad", "rad", "rad", "rad", "rad", "m"),
        action_frame_id="arm_base",
        dispatch_provenance={"arm": {}, "gripper": {}},
        effective_intervals={"arm": [1.0, 2.0], "gripper": [1.0, 2.0]},
    )
    return AlignedRow(
        grid_index=5,
        decision_sim_time_s=1.0,
        decision_backend_monotonic_s=100.2,
        image=image,
        joint_feedback=feedback,
        encoder=encoder,
        imu=imu,
        actuator_reference=reference,
        image_age_s=0.2,
        image_arrival_age_s=0.0,
        feedback_age_s=0.2,
        image_feedback_capture_skew_s=0.0,
        profiles={"d0": d0, "d2": d2},
    )


def _alignment(profile, rows) -> AlignmentResult:
    return AlignmentResult(
        rows=rows,
        rejected=[],
        summary={"profile_sha256": profile.sha256},
        source_identity={"run_id": "run_1", "episode_id": "episode_1"},
    )


def test_both_profiles_can_be_export_valid_without_training_qualification() -> None:
    profile = load_export_profile(PROFILE_PATH)
    alignment = _alignment(profile, [_row()])
    for name in ("d0", "d2"):
        result = assess_export_valid(
            profile_name=name,
            profile=profile,
            alignment=alignment,
        )
        assert result["passed"] is True
        assert result["rows"] == 1

    training = assess_train_qualified(
        profile_name="d2",
        profile=profile,
        operator_kind="agent_ui",
        task_result="failure",
        view_source="third_person_3d",
        parent_episode_count=1,
        export_result={"passed": True},
    )
    assert training["qualified"] is False
    assert any("operator_kind" in reason for reason in training["reasons"])
    assert any("task_result" in reason for reason in training["reasons"])
    assert any("parent_episode_count" in reason for reason in training["reasons"])


def test_future_observation_and_missing_provenance_are_rejected() -> None:
    profile = load_export_profile(PROFILE_PATH)
    normal = _row()
    future = replace(
        normal,
        image=replace(
            normal.image,
            arrival_backend_monotonic_s=normal.decision_backend_monotonic_s + 0.01,
        ),
    )
    result = assess_export_valid(
        profile_name="d0",
        profile=profile,
        alignment=_alignment(profile, [future]),
    )
    assert result["passed"] is False
    assert "future_image_arrival" in result["row_failure_counts"]

    missing_dispatch = replace(
        normal,
        profiles={
            "d0": replace(
                normal.profiles["d0"],
                dispatch_provenance={},
                effective_intervals={},
            )
        },
    )
    result = assess_export_valid(
        profile_name="d0",
        profile=profile,
        alignment=_alignment(profile, [missing_dispatch]),
    )
    assert result["passed"] is False
    assert "d0_dispatch_provenance_missing" in result["row_failure_counts"]
