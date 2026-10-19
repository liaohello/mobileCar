"""Unit checks for G2B source feedback, causal selection and split safety."""

from __future__ import annotations

from dataclasses import dataclass

import pytest

from mobilecar.data.export.aligner import (
    _d2_state,
    _model_to_user,
    assert_policy_feature_allowlist,
    latest_arrived,
    parent_split_leak,
)
from mobilecar.data.export.models import AlignmentResult, ObservationSample
from mobilecar.data.export.reader import (
    EpisodeReader,
    SourceIntegrityError,
    resolve_episode_media,
)


def _sample(sample_id: str, arrival: float, capture: float = 0.0) -> ObservationSample:
    return ObservationSample(
        sample_id=sample_id,
        sensor_kind="encoders",
        stream_id="encoders",
        recording_segment_id="seg",
        control_tick_id="tick",
        capture_sim_time_s=capture,
        capture_backend_monotonic_s=capture,
        arrival_backend_monotonic_s=arrival,
        value={},
        run_id="run",
        episode_id="episode",
    )


def test_latest_arrived_never_selects_future_sample() -> None:
    samples = [_sample("a", 1.0), _sample("b", 1.2), _sample("c", 1.4)]
    assert latest_arrived(
        samples, 1.19, arrival="arrival_backend_monotonic_s"
    ).sample_id == "a"
    assert latest_arrived(
        samples, 1.2, arrival="arrival_backend_monotonic_s"
    ).sample_id == "b"


def test_event_feedback_uses_exact_control_tick_and_monotonic_pair() -> None:
    reader = EpisodeReader.__new__(EpisodeReader)
    reader.run_id = "run"
    reader.episode_id = "episode"
    reader.episode = {
        "segments": [{"recording_segment_id": "seg"}],
    }
    reader.events = [
        {
            "run_id": "run",
            "episode_id": "episode",
            "event_type": "feedback",
            "backend_monotonic_s": 100.04,
            "control_tick_id": "tick_000000042",
            "payload": {
                "feedback_time_monotonic": 100.03,
                "sim_time_s": 1.2,
                "arm_model_qpos_rad": [0.1, 0.2, 0.3, -1.5707963267948966, 0.4],
                "gripper_width_m": 0.025,
            },
        }
    ]
    feedback = reader._build_event_feedback()
    assert len(feedback) == 1
    assert feedback[0].sample_id == "joint_feedback_000000042"
    assert feedback[0].control_tick_id == "tick_000000042"
    assert feedback[0].capture_backend_monotonic_s == 100.03
    assert feedback[0].arrival_backend_monotonic_s == 100.04
    assert feedback[0].value["model_joint_positions_rad"][3] == pytest.approx(
        -1.5707963267948966
    )


def test_d2_state_maps_model_angles_to_user_coordinates() -> None:
    feedback = ObservationSample(
        sample_id="joint",
        sensor_kind="joint_feedback",
        stream_id="joint_feedback",
        recording_segment_id="seg",
        control_tick_id="tick",
        capture_sim_time_s=1.0,
        capture_backend_monotonic_s=10.0,
        arrival_backend_monotonic_s=10.01,
        value={
            "model_joint_positions_rad": [
                0.1,
                0.2,
                0.3,
                -1.47,
                0.5,
            ],
            "gripper_width_m": 0.031,
        },
        run_id="run",
        episode_id="episode",
    )
    state = _d2_state(
        feedback,
        zero_model_rad=(0.0, 0.0, 0.0, -1.5707963267948966, 0.0),
        direction=(1.0, 1.0, 1.0, 1.0, 1.0),
    )
    assert state == pytest.approx([0.1, 0.2, 0.3, 0.1007963267948966, 0.5, 0.031])
    assert _model_to_user(
        [0.0, 0.0, 0.0, -1.5707963267948966, 0.0],
        zero_model_rad=[0.0, 0.0, 0.0, -1.5707963267948966, 0.0],
        direction=[1.0, 1.0, 1.0, 1.0, 1.0],
    ) == pytest.approx([0.0] * 5)


def test_parent_episode_subsegments_cannot_cross_splits() -> None:
    assert not parent_split_leak(
        [
            {"parent_episode_id": "a", "split": "dev"},
            {"parent_episode_id": "a", "split": "dev"},
        ]
    )
    assert parent_split_leak(
        [
            {"parent_episode_id": "a", "split": "train"},
            {"parent_episode_id": "a", "split": "validation"},
        ]
    )


@dataclass
class _IdentityRow:
    run_id: str
    episode_id: str
    clock_id: str


def test_sample_identity_rejects_clock_or_episode_mix() -> None:
    from mobilecar.data.export.aligner import assert_sample_identity

    good = {
        "run_id": "run",
        "episode_id": "episode",
        "clock_id": "backend_monotonic",
    }
    assert_sample_identity(
        good,
        run_id="run",
        episode_id="episode",
        clock_id="backend_monotonic",
    )
    with pytest.raises(SourceIntegrityError, match="clock_mismatch"):
        assert_sample_identity(
            {**good, "clock_id": "wall_utc"},
            run_id="run",
            episode_id="episode",
            clock_id="backend_monotonic",
        )
    with pytest.raises(SourceIntegrityError, match="cross_episode"):
        assert_sample_identity(
            {**good, "episode_id": "other"},
            run_id="run",
            episode_id="episode",
            clock_id="backend_monotonic",
        )


def test_missing_media_truth_features_and_long_gap_are_rejected(
    tmp_path,
) -> None:
    episode = tmp_path / "episode"
    episode.mkdir()
    with pytest.raises(SourceIntegrityError, match="source asset missing"):
        resolve_episode_media(episode, "assets/rgb/missing.png")
    with pytest.raises(ValueError, match="forbidden policy feature"):
        assert_policy_feature_allowlist(
            ["observation.images.link4_rgb", "world_sim.object_pose"]
        )

    @dataclass
    class Row:
        grid_index: int
        profiles: dict
        reasons: tuple = ()

    result = AlignmentResult(
        rows=[
            Row(0, {"d0": object()}),
            Row(1, {"d0": object()}),
            Row(4, {"d0": object()}),
        ],
        rejected=[],
        summary={},
        source_identity={},
        profile_max_gap_slots=1,
    )
    assert [len(segment) for segment in result.contiguous_segments("d0")] == [
        2,
        1,
    ]
