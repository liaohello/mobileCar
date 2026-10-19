"""Locked official LeRobot writer and loader verification boundary."""

from __future__ import annotations

import importlib.metadata
import platform
import sys
from pathlib import Path
from typing import Any, Sequence

import numpy as np
from PIL import Image

from .models import AlignedRow, AlignmentResult, ExportProfile, sha256_file
from .reader import EpisodeReader
from .sidecar import write_dataset_sidecar, write_output_manifest


EXPECTED_LEROBOT_VERSION = "0.4.4"
EXPECTED_LEROBOT_COMMIT = "8fff0fde7c79f23a93d845d1a50e985de01f8b8a"
EXPECTED_LEROBOT_WHEEL_SHA256 = (
    "e53800ead8216861540ad3aebaf12e3cf87a399b3c1f234eeead33716c9c24fd"
)
EXPECTED_CODEBASE_VERSION = "v3.0"
EXPECTED_FPS = 5


def _official_api() -> tuple[Any, Any, Any]:
    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        from lerobot.datasets.utils import write_info
        import torch.utils.data as torch_data
    except ImportError as exc:
        raise RuntimeError(
            "official lerobot==0.4.4 is not installed in this export environment"
        ) from exc
    version = importlib.metadata.version("lerobot")
    if version != EXPECTED_LEROBOT_VERSION:
        raise RuntimeError(
            f"locked lerobot version mismatch: expected {EXPECTED_LEROBOT_VERSION}, got {version}"
        )
    return LeRobotDataset, torch_data.DataLoader, write_info


def environment_identity() -> dict[str, Any]:
    packages: dict[str, str | None] = {}
    for name in ("lerobot", "torch", "torchvision", "datasets", "av", "pillow", "numpy"):
        try:
            packages[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            packages[name] = None
    return {
        "python": sys.version,
        "python_executable": sys.executable,
        "platform": platform.platform(),
        "packages": packages,
        "lerobot_tag_commit": EXPECTED_LEROBOT_COMMIT,
        "lerobot_wheel_sha256": EXPECTED_LEROBOT_WHEEL_SHA256,
        "codebase_version": EXPECTED_CODEBASE_VERSION,
        "fps": EXPECTED_FPS,
        "video_codec": "h264",
        "video_backend": "pyav",
    }


def _feature_spec(
    profile: ExportProfile,
    profile_name: str,
    image_height: int,
    image_width: int,
) -> dict[str, Any]:
    spec = profile.feature_spec(profile_name)
    state_names = list(spec["state_names"])
    action_names = list(spec["action_names"])
    return {
        profile.image_key: {
            "dtype": "video",
            "shape": (3, image_height, image_width),
            "names": ["channels", "height", "width"],
        },
        "observation.state": {
            "dtype": "float32",
            "shape": (len(state_names),),
            "names": state_names,
        },
        "action": {
            "dtype": "float32",
            "shape": (len(action_names),),
            "names": action_names,
        },
    }


def _load_image(path: Path, expected_width: int, expected_height: int) -> np.ndarray:
    with Image.open(path) as handle:
        image = np.asarray(handle.convert("RGB"), dtype=np.uint8)
    if image.shape != (expected_height, expected_width, 3):
        raise ValueError(
            f"RGB shape mismatch for {path}: {image.shape} != "
            f"{(expected_height, expected_width, 3)}"
        )
    return image


def _tensor_shape(value: Any) -> list[int]:
    if hasattr(value, "shape"):
        return [int(item) for item in value.shape]
    array = np.asarray(value)
    return [int(item) for item in array.shape]


def _vector_values(value: Any) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    return np.asarray(value, dtype=np.float64).reshape(-1)


def _expected_row_layout(
    rows: Sequence[AlignedRow],
    segment_lengths: Sequence[int],
) -> list[tuple[int, int, float, AlignedRow]]:
    if sum(segment_lengths) != len(rows):
        raise AssertionError("segment lengths do not cover all aligned rows")
    layout: list[tuple[int, int, float, AlignedRow]] = []
    offset = 0
    for episode_index, length in enumerate(segment_lengths):
        if length <= 0:
            raise AssertionError("LeRobot episode segments must be non-empty")
        for frame_index in range(length):
            layout.append(
                (
                    episode_index,
                    frame_index,
                    frame_index / EXPECTED_FPS,
                    rows[offset + frame_index],
                )
            )
        offset += length
    return layout


def verify_with_official_loader(
    dataset_root: str | Path,
    repo_id: str,
    *,
    expected_rows: int,
    expected_state_shape: int,
    expected_action_shape: int,
    expected_image_shape: Sequence[int] | None = None,
    expected_row_data: Sequence[AlignedRow] | None = None,
    expected_segment_lengths: Sequence[int] | None = None,
) -> dict[str, Any]:
    LeRobotDataset, DataLoader, _ = _official_api()
    root = Path(dataset_root)
    dataset = LeRobotDataset(
        repo_id,
        root=root,
        video_backend="pyav",
        tolerance_s=1e-4,
    )
    if len(dataset) != expected_rows:
        raise AssertionError(
            f"official loader row count {len(dataset)} != {expected_rows}"
        )
    if dataset.fps != EXPECTED_FPS:
        raise AssertionError(f"official loader fps {dataset.fps} != {EXPECTED_FPS}")
    codebase_version = dataset.meta.info.get("codebase_version")
    if codebase_version != EXPECTED_CODEBASE_VERSION:
        raise AssertionError(
            "official loader codebase_version "
            f"{codebase_version!r} != {EXPECTED_CODEBASE_VERSION!r}"
        )
    if list(dataset.meta.camera_keys) != ["observation.images.link4_rgb"]:
        raise AssertionError(
            f"unexpected official camera keys: {list(dataset.meta.camera_keys)}"
        )
    expected_split = f"0:{dataset.num_episodes}"
    if dataset.meta.info.get("splits") != {"dev": expected_split}:
        raise AssertionError(
            f"official dataset split {dataset.meta.info.get('splits')} "
            f"!= {{'dev': {expected_split!r}}}"
        )
    loaded_features = dataset.meta.info.get("features") or {}
    expected_feature_shapes = {
        "observation.state": [expected_state_shape],
        "action": [expected_action_shape],
    }
    if expected_image_shape is not None:
        expected_feature_shapes["observation.images.link4_rgb"] = list(
            expected_image_shape
        )
    for key, shape in expected_feature_shapes.items():
        actual_shape = list(loaded_features[key]["shape"])
        if actual_shape != shape:
            raise AssertionError(
                f"official feature {key} shape {actual_shape} != {shape}"
            )
    if loaded_features["observation.images.link4_rgb"]["dtype"] != "video":
        raise AssertionError("official link4 RGB feature is not stored as video")

    layout = None
    if expected_row_data is not None:
        if expected_segment_lengths is None:
            raise ValueError(
                "expected_segment_lengths is required with expected_row_data"
            )
        layout = _expected_row_layout(
            expected_row_data,
            expected_segment_lengths,
        )
        if len(layout) != expected_rows:
            raise AssertionError("expected row layout count mismatch")
        episode_lengths = [
            int(dataset.meta.episodes[index]["length"])
            for index in range(dataset.num_episodes)
        ]
        if episode_lengths != list(expected_segment_lengths):
            raise AssertionError(
                f"official episode lengths {episode_lengths} != "
                f"{list(expected_segment_lengths)}"
            )

    selected_indices = {0, len(dataset) // 2, len(dataset) - 1}
    samples: list[dict[str, Any]] = []
    camera_key = dataset.meta.camera_keys[0]
    for index in range(len(dataset)):
        item = dataset[index]
        state_shape = _tensor_shape(item["observation.state"])
        action_shape = _tensor_shape(item["action"])
        image_shape = _tensor_shape(item[camera_key])
        if state_shape != [expected_state_shape]:
            raise AssertionError(f"state shape {state_shape} at row {index}")
        if action_shape != [expected_action_shape]:
            raise AssertionError(f"action shape {action_shape} at row {index}")
        if len(image_shape) != 3 or image_shape[0] != 3:
            raise AssertionError(f"image shape {image_shape} at row {index}")
        if (
            expected_image_shape is not None
            and image_shape != list(expected_image_shape)
        ):
            raise AssertionError(
                f"image shape {image_shape} at row {index} != "
                f"{list(expected_image_shape)}"
            )

        if layout is not None:
            expected_episode, expected_frame, expected_timestamp, row = layout[index]
            actual_timestamp = float(item["timestamp"])
            actual_frame = int(item["frame_index"])
            actual_episode = int(item["episode_index"])
            if actual_episode != expected_episode:
                raise AssertionError(
                    f"episode index {actual_episode} at row {index} "
                    f"!= {expected_episode}"
                )
            if actual_frame != expected_frame:
                raise AssertionError(
                    f"frame index {actual_frame} at row {index} != {expected_frame}"
                )
            if abs(actual_timestamp - expected_timestamp) > 1e-5:
                raise AssertionError(
                    f"timestamp {actual_timestamp} at row {index} "
                    f"!= {expected_timestamp}"
                )
            issued = row.profiles["d0" if expected_action_shape == 3 else "d2"]
            actual_state = _vector_values(item["observation.state"])
            actual_action = _vector_values(item["action"])
            if not np.allclose(
                actual_state,
                np.asarray(issued.state, dtype=np.float64),
                rtol=1e-6,
                atol=1e-6,
            ):
                raise AssertionError(
                    f"observation.state values do not match row {index}"
                )
            if not np.allclose(
                actual_action,
                np.asarray(issued.action, dtype=np.float64),
                rtol=1e-6,
                atol=1e-6,
            ):
                raise AssertionError(f"action values do not match row {index}")

            if index in selected_indices:
                samples.append(
                    {
                        "index": index,
                        "timestamp": actual_timestamp,
                        "frame_index": actual_frame,
                        "episode_index": actual_episode,
                        "observation_state_shape": state_shape,
                        "action_shape": action_shape,
                        "image_shape": image_shape,
                        "task": str(item["task"]),
                        "source_row": {
                            "parent_episode_id": row.image.episode_id,
                            "recording_segment_id": row.image.recording_segment_id,
                            "grid_index": row.grid_index,
                            "image_sample_id": row.image.sample_id,
                            "image_control_tick_id": row.image.control_tick_id,
                            "feedback_sample_id": (
                                row.joint_feedback.sample_id
                                if row.joint_feedback is not None
                                else None
                            ),
                            "action_reference_control_tick_id": (
                                row.actuator_reference.control_tick_id
                            ),
                        },
                    }
                )
        elif index in selected_indices:
            samples.append(
                {
                    "index": index,
                    "timestamp": float(item["timestamp"]),
                    "frame_index": int(item["frame_index"]),
                    "episode_index": int(item["episode_index"]),
                    "observation_state_shape": state_shape,
                    "action_shape": action_shape,
                    "image_shape": image_shape,
                    "task": str(item["task"]),
                }
            )

    batch_size = min(4, len(dataset))
    batch = next(
        iter(
            DataLoader(
                dataset,
                batch_size=batch_size,
                shuffle=False,
                num_workers=0,
            )
        )
    )
    batch_shapes = {
        key: _tensor_shape(batch[key])
        for key in ("observation.state", "action", "timestamp")
    }
    batch_shapes[camera_key] = _tensor_shape(batch[camera_key])
    if batch_shapes["observation.state"] != [batch_size, expected_state_shape]:
        raise AssertionError(
            f"batch state shape {batch_shapes['observation.state']} is invalid"
        )
    if batch_shapes["action"] != [batch_size, expected_action_shape]:
        raise AssertionError(
            f"batch action shape {batch_shapes['action']} is invalid"
        )
    if (
        expected_image_shape is not None
        and batch_shapes[camera_key] != [batch_size, *expected_image_shape]
    ):
        raise AssertionError(
            f"batch image shape {batch_shapes[camera_key]} is invalid"
        )
    return {
        "passed": True,
        "lerobot_version": importlib.metadata.version("lerobot"),
        "lerobot_tag_commit": EXPECTED_LEROBOT_COMMIT,
        "lerobot_wheel_sha256": EXPECTED_LEROBOT_WHEEL_SHA256,
        "codebase_version": codebase_version,
        "repo_id": repo_id,
        "root": str(root),
        "num_frames": len(dataset),
        "num_episodes": dataset.num_episodes,
        "fps": dataset.fps,
        "features": dataset.meta.info.get("features"),
        "video_info": dataset.meta.info.get("video_path"),
        "splits": dataset.meta.info.get("splits"),
        "all_samples_checked": len(dataset),
        "all_video_frames_decoded": True,
        "single_samples": samples,
        "batch_size": batch_size,
        "batch_shapes": batch_shapes,
    }


def write_profile_dataset(
    *,
    output_root: str | Path,
    profile_name: str,
    profile: ExportProfile,
    reader: EpisodeReader,
    alignment: AlignmentResult,
    repo_id: str,
    source_file_hashes_before: dict[str, str],
) -> dict[str, Any]:
    LeRobotDataset, _, write_info = _official_api()
    rows = alignment.rows_for(profile_name)
    if not rows:
        raise ValueError(f"{profile_name} alignment has no exportable rows")
    segments = alignment.contiguous_segments(profile_name)
    dataset_root = Path(output_root) / f"{profile_name}_dev"
    if dataset_root.exists():
        raise FileExistsError(f"refusing to overwrite existing dataset {dataset_root}")
    dataset_root.parent.mkdir(parents=True, exist_ok=True)
    features = _feature_spec(
        profile,
        profile_name,
        image_height=rows[0].image.height,
        image_width=rows[0].image.width,
    )
    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        fps=int(profile.grid_rate_hz),
        features=features,
        root=dataset_root,
        robot_type="landerpi_mecanum_5dof",
        use_videos=True,
        tolerance_s=1e-4,
        video_backend="pyav",
        batch_encoding_size=1,
        vcodec="h264",
        encoder_threads=2,
    )
    for segment in segments:
        for row in segment:
            issued = row.profiles[profile_name]
            frame = {
                profile.image_key: _load_image(
                    reader.episode_dir / row.image.rgb_uri,
                    expected_width=row.image.width,
                    expected_height=row.image.height,
                ),
                "observation.state": np.asarray(
                    issued.state, dtype=np.float32
                ),
                "action": np.asarray(issued.action, dtype=np.float32),
                "task": reader.task_language,
            }
            dataset.add_frame(frame)
        dataset.save_episode(parallel_encoding=False)
    dataset.finalize()
    dataset.meta.info["splits"] = {"dev": f"0:{len(segments)}"}
    write_info(dataset.meta.info, dataset.root)
    source_file_hashes_after = reader.source_file_hashes()
    if source_file_hashes_after != source_file_hashes_before:
        raise RuntimeError(
            "source Episode file hashes changed during read-only export"
        )
    expected_state = len(profile.feature_spec(profile_name)["state_names"])
    expected_action = len(profile.feature_spec(profile_name)["action_names"])
    loader = verify_with_official_loader(
        dataset_root,
        repo_id,
        expected_rows=len(rows),
        expected_state_shape=expected_state,
        expected_action_shape=expected_action,
        expected_image_shape=(
            3,
            rows[0].image.height,
            rows[0].image.width,
        ),
        expected_row_data=rows,
        expected_segment_lengths=[len(item) for item in segments],
    )
    sidecar = write_dataset_sidecar(
        dataset_root=dataset_root,
        reader=reader,
        alignment=alignment,
        profile=profile,
        profile_name=profile_name,
        segment_row_counts=[len(item) for item in segments],
        source_file_hashes_before=source_file_hashes_before,
        source_file_hashes_after=source_file_hashes_after,
        environment=environment_identity(),
        loader_result=loader,
    )
    manifest = write_output_manifest(dataset_root)
    return {
        "profile": profile_name.upper(),
        "dataset_root": str(dataset_root),
        "repo_id": repo_id,
        "rows": len(rows),
        "segments": len(segments),
        "loader": loader,
        "sidecar": sidecar,
        "manifest": {
            "file_count": manifest["file_count"],
            "total_bytes": manifest["total_bytes"],
            "sha256": sha256_file(dataset_root / "manifest.json"),
        },
        "format_valid": bool(loader.get("passed")),
    }
