"""Writer-side contract checks for the locked G2B export boundary."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image

from mobilecar.data.export import lerobot_adapter
from mobilecar.data.export.models import (
    ActuatorReference,
    AlignedRow,
    AlignmentResult,
    ImageSample,
    ObservationSample,
    ProfileRow,
    load_export_profile,
)
from mobilecar.data.export.sidecar import write_dataset_sidecar


ROOT = Path(__file__).resolve().parents[2]
PROFILE = ROOT / "configs/export/g2b_export_v1.yaml"


def test_feature_spec_preserves_profile_names_and_link4_video() -> None:
    profile = load_export_profile(PROFILE)
    features = lerobot_adapter._feature_spec(profile, "d0", 480, 640)

    assert features["observation.images.link4_rgb"] == {
        "dtype": "video",
        "shape": (3, 480, 640),
        "names": ["channels", "height", "width"],
    }
    assert features["observation.state"]["dtype"] == "float32"
    assert features["observation.state"]["names"] == profile.feature_spec("d0")[
        "state_names"
    ]
    assert features["action"]["shape"] == (3,)
    assert features["action"]["names"] == [
        "vx_m_s",
        "vy_m_s",
        "wz_rad_s",
    ]


def test_load_image_rejects_a_shape_that_does_not_match_the_profile(tmp_path: Path) -> None:
    image_path = tmp_path / "rgb.png"
    Image.new("RGB", (640, 480), (1, 2, 3)).save(image_path)
    assert lerobot_adapter._load_image(image_path, 640, 480).shape == (480, 640, 3)

    wrong_path = tmp_path / "wrong.png"
    Image.new("RGB", (32, 24), (1, 2, 3)).save(wrong_path)
    try:
        lerobot_adapter._load_image(wrong_path, 640, 480)
    except ValueError as exc:
        assert "RGB shape mismatch" in str(exc)
    else:
        raise AssertionError("wrong RGB shape was accepted")


def test_official_loader_boundary_checks_every_sample_and_small_batch(
    monkeypatch,
) -> None:
    camera_key = "observation.images.link4_rgb"

    class FakeMeta:
        info = {
            "codebase_version": "v3.0",
            "splits": {"dev": "0:1"},
            "features": {
                "observation.state": {
                    "dtype": "float32",
                    "shape": [3],
                },
                "action": {
                    "dtype": "float32",
                    "shape": [3],
                },
                camera_key: {
                    "dtype": "video",
                    "shape": [3, 4, 5],
                },
            },
        }
        camera_keys = [camera_key]

    class FakeDataset:
        fps = 5
        num_episodes = 1
        meta = FakeMeta()

        def __init__(self, *_args, **_kwargs) -> None:
            pass

        def __len__(self) -> int:
            return 2

        def __getitem__(self, index: int) -> dict:
            return {
                "observation.state": np.zeros((3,), dtype=np.float32),
                "action": np.ones((3,), dtype=np.float32),
                camera_key: np.zeros((3, 4, 5), dtype=np.uint8),
                "timestamp": np.float32(index / 5),
                "frame_index": np.int64(index),
                "episode_index": np.int64(0),
                "task": "fixture task",
            }

    class FakeDataLoader:
        def __init__(self, dataset, batch_size: int, **_kwargs) -> None:
            self.dataset = dataset
            self.batch_size = batch_size

        def __iter__(self):
            size = min(self.batch_size, len(self.dataset))
            yield {
                "observation.state": np.zeros((size, 3), dtype=np.float32),
                "action": np.ones((size, 3), dtype=np.float32),
                "timestamp": np.arange(size, dtype=np.float32) / 5,
                camera_key: np.zeros((size, 3, 4, 5), dtype=np.uint8),
            }

    monkeypatch.setattr(
        lerobot_adapter,
        "_official_api",
        lambda: (FakeDataset, FakeDataLoader, lambda *_: None),
    )
    monkeypatch.setattr(
        lerobot_adapter.importlib.metadata,
        "version",
        lambda _name: "0.4.4",
    )
    result = lerobot_adapter.verify_with_official_loader(
        "/fixture/dataset",
        "mobilecar/fixture",
        expected_rows=2,
        expected_state_shape=3,
        expected_action_shape=3,
    )

    assert result["passed"] is True
    assert result["all_samples_checked"] == 2
    assert result["all_video_frames_decoded"] is True
    assert result["batch_size"] == 2
    assert result["batch_shapes"][camera_key] == [2, 3, 4, 5]


def _digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_sidecar_retains_row_provenance_depth_and_parent_split(
    tmp_path: Path,
) -> None:
    profile = load_export_profile(PROFILE)
    episode_dir = tmp_path / "episode"
    (episode_dir / "assets/depth").mkdir(parents=True)
    (episode_dir / "assets/masks").mkdir(parents=True)
    depth_path = episode_dir / "assets/depth/fixture_depth_m.npy"
    mask_path = episode_dir / "assets/masks/fixture_valid.png"
    depth_path.write_bytes(b"metric-depth-fixture")
    mask_path.write_bytes(b"mask-fixture")

    image = ImageSample(
        sample_id="rgbd_fixture",
        stream_id="rgbd",
        recording_segment_id="seg_fixture",
        capture_sim_time_s=1.0,
        capture_backend_monotonic_s=100.0,
        publish_backend_monotonic_s=100.05,
        arrival_backend_monotonic_s=100.06,
        persist_backend_monotonic_s=100.1,
        control_tick_id="tick_000000100",
        feedback_sample_id="joint_feedback_000000100",
        rgb_uri="assets/rgb/fixture.png",
        depth_uri="assets/depth/fixture_depth_m.npy",
        valid_mask_uri="assets/masks/fixture_valid.png",
        width=640,
        height=480,
        run_id="run_fixture",
        episode_id="episode_fixture",
    )
    feedback = ObservationSample(
        sample_id="joint_feedback_000000100",
        sensor_kind="joint_feedback",
        stream_id="joint_feedback",
        recording_segment_id="seg_fixture",
        control_tick_id="tick_000000100",
        capture_sim_time_s=0.999,
        capture_backend_monotonic_s=99.999,
        arrival_backend_monotonic_s=99.999,
        value={
            "user_joint_positions_rad": [0.1, 0.2, 0.3, 0.4, 0.5],
            "gripper_width_m": 0.025,
        },
        run_id="run_fixture",
        episode_id="episode_fixture",
    )
    encoder = ObservationSample(
        sample_id="encoders_000000100",
        sensor_kind="encoders",
        stream_id="encoders",
        recording_segment_id="seg_fixture",
        control_tick_id="tick_000000100",
        capture_sim_time_s=1.0,
        capture_backend_monotonic_s=100.0,
        arrival_backend_monotonic_s=100.0,
        value={"wheel_velocity_rad_s": [0.1, 0.2, 0.3, 0.4]},
        run_id="run_fixture",
        episode_id="episode_fixture",
    )
    imu = ObservationSample(
        sample_id="imu_000000100",
        sensor_kind="imu",
        stream_id="imu",
        recording_segment_id="seg_fixture",
        control_tick_id="tick_000000100",
        capture_sim_time_s=1.0,
        capture_backend_monotonic_s=100.0,
        arrival_backend_monotonic_s=100.0,
        value={"acceleration_m_s2": [0.0, 0.0, 9.81], "gyro_rad_s": [0.0, 0.0, 0.0]},
        run_id="run_fixture",
        episode_id="episode_fixture",
    )
    reference = ActuatorReference(
        sequence=3,
        backend_monotonic_s=100.01,
        sim_time_s=1.0,
        control_tick_id="tick_000000100",
        payload={
            "base_twist": [0.02, 0.0, 0.03],
            "arm_model_reference_rad": [0.1, 0.2, 0.3, -1.5707963267948966, 0.5, -1.0],
            "gripper_width_target_m": 0.03,
        },
    )
    profile_row = ProfileRow(
        profile="d0",
        state=(0.1, 0.2, 0.3, 0.4, 0.0, 0.0, 9.81, 0.0, 0.0, 0.0),
        action=(0.02, 0.0, 0.03),
        action_units=("m/s", "m/s", "rad/s"),
        action_frame_id="base_link",
        dispatch_provenance={
            "base": {
                "command_id": "cmd_fixture",
                "requested": {"backend_monotonic_s": 99.9},
                "approved": {"backend_monotonic_s": 99.91},
                "dispatch": {
                    "backend_monotonic_s": 99.92,
                    "control_tick_id": "tick_000000099",
                },
            }
        },
        effective_intervals={"base": [1.0, 2.0]},
    )
    row = AlignedRow(
        grid_index=5,
        decision_sim_time_s=1.0,
        decision_backend_monotonic_s=100.2,
        image=image,
        joint_feedback=feedback,
        encoder=encoder,
        imu=imu,
        actuator_reference=reference,
        image_age_s=0.2,
        image_arrival_age_s=0.14,
        feedback_age_s=0.201,
        image_feedback_capture_skew_s=0.001,
        profiles={"d0": profile_row},
    )
    alignment = AlignmentResult(
        rows=[row],
        rejected=[
            {
                "grid_index": 6,
                "reasons": ["image_age_exceeded"],
            }
        ],
        summary={"profile_id": profile.profile_id, "d0_rows": 1},
        source_identity={
            "run_id": "run_fixture",
            "episode_id": "episode_fixture",
        },
    )
    before = {
        "assets/depth/fixture_depth_m.npy": _digest(depth_path),
        "assets/masks/fixture_valid.png": _digest(mask_path),
    }

    class FakeReader:
        run_id = "run_fixture"
        episode_id = "episode_fixture"
        source = "mujoco_renderer"
        task_language = "fixture task"
        task_result = "failure"
        audit_status = "COMPLETE"
        episode = {
            "operator_kind": "agent_ui",
            "view_source": "third_person_3d",
        }

        def identity(self):
            return {
                "run_id": self.run_id,
                "episode_id": self.episode_id,
                "task_result": self.task_result,
            }

        def asset_hash(self, relative_uri: str) -> str:
            return before[relative_uri]

    dataset_root = tmp_path / "dataset"
    dataset_root.mkdir()
    result = write_dataset_sidecar(
        dataset_root=dataset_root,
        reader=FakeReader(),
        alignment=alignment,
        profile=profile,
        profile_name="d0",
        segment_row_counts=[1],
        source_file_hashes_before=before,
        source_file_hashes_after=dict(before),
        environment={"lerobot": "0.4.4"},
        loader_result={"passed": True, "all_samples_checked": 1},
    )

    assert result["rows"] == 1
    sidecar_row = json.loads(
        (dataset_root / "g2b/sidecar.jsonl").read_text(encoding="utf-8")
    )
    assert sidecar_row["run_id"] == "run_fixture"
    assert sidecar_row["parent_episode_id"] == "episode_fixture"
    assert sidecar_row["recording_segment_id"] == "seg_fixture"
    assert sidecar_row["grid_index"] == 5
    assert sidecar_row["source_task_result"] == "failure"
    assert sidecar_row["operator_kind"] == "agent_ui"
    assert sidecar_row["view_source"] == "third_person_3d"
    assert sidecar_row["action_reference"]["control_tick_id"] == "tick_000000100"
    assert sidecar_row["dispatch_provenance"]["base"]["command_id"] == "cmd_fixture"
    assert sidecar_row["action"] == [0.02, 0.0, 0.03]

    split = json.loads(
        (dataset_root / "g2b/split.json").read_text(encoding="utf-8")
    )
    assert split["dev"] == [0]
    assert split["train"] == []
    assert split["validation"] == []

    source_episode = json.loads(
        (dataset_root / "g2b/source_episode.json").read_text(encoding="utf-8")
    )
    assert source_episode["source_files_unchanged"] is True
    depth_rows = [
        json.loads(line)
        for line in (dataset_root / "g2b/depth_artifacts.jsonl")
        .read_text(encoding="utf-8")
        .splitlines()
    ]
    assert {row["kind"] for row in depth_rows} == {"depth", "valid_mask"}
