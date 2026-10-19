"""Read-only G4A-2 human cohort reader and locked RGB-D LeRobot export."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image

from .export.models import sha256_file, sha256_json
from .export.reader import EpisodeReader


ZERO_MODEL = [0.0, 0.0, 0.0, -math.pi / 2.0, 0.0]
DIRECTION = [1.0, 1.0, 1.0, 1.0, 1.0]


@dataclass(frozen=True)
class G4A2Row:
    reader: EpisodeReader
    image: Any
    feedback: Any
    decision_backend_monotonic_s: float
    human_intent_id: str
    display_sample_id: str
    action_frame_id: str
    action_seq: int
    state: tuple[float, ...]
    action: tuple[float, ...]
    assist_mode: str
    control_generator: str
    control_tick_id: str | None


def _read_human_actions(reader: EpisodeReader) -> tuple[list[dict[str, Any]], list[str]]:
    linked = {
        str(row.get("payload", {}).get("action_frame_id")): row
        for row in reader.events
        if row.get("event_type") == "g4a2_action_linked"
    }
    approved = {
        str(row.get("payload", {}).get("request", {}).get("action_frame_id")): row
        for row in reader.events
        if row.get("event_type") == "g4a1_action_approved"
    }
    intents = {
        str(row.get("payload", {}).get("human_intent_id")): row
        for row in reader.events
        if row.get("event_type") == "human_intent"
    }
    rows = []
    reasons = []
    for frame_id, linked_event in sorted(
        linked.items(), key=lambda item: item[1].get("backend_monotonic_s", 0.0)
    ):
        approved_event = approved.get(frame_id)
        if approved_event is None:
            reasons.append(f"{frame_id}:missing_g4a1_approved")
            continue
        payload = linked_event.get("payload") or {}
        request = (approved_event.get("payload") or {}).get("request") or {}
        if request.get("source") != "human_teleop":
            reasons.append(f"{frame_id}:not_human_source")
            continue
        if request.get("teacher_plan_id") is not None:
            reasons.append(f"{frame_id}:hidden_teacher_plan")
            continue
        if request.get("joint_order") != [f"joint{i}" for i in range(1, 6)]:
            reasons.append(f"{frame_id}:joint_order")
            continue
        if request.get("units") != ["rad", "rad", "rad", "rad", "rad", "m"]:
            reasons.append(f"{frame_id}:units")
            continue
        intent = intents.get(str(payload.get("human_intent_id")))
        if intent is None:
            reasons.append(f"{frame_id}:missing_human_intent")
            continue
        rows.append(
            {
                "linked": linked_event,
                "approved": approved_event,
                "request": request,
                "intent": intent,
            }
        )
    return rows, reasons


def extract_g4a2_rows(
    reader: EpisodeReader,
    *,
    image_max_age_s: float = 0.4,
    feedback_max_age_s: float = 0.2,
) -> tuple[list[G4A2Row], list[dict[str, Any]]]:
    actions, reasons = _read_human_actions(reader)
    images = {image.sample_id: image for image in reader.images}
    rows: list[G4A2Row] = []
    rejected: list[dict[str, Any]] = [
        {"reason": reason} for reason in reasons
    ]
    for item in actions:
        request = item["request"]
        linked = item["linked"]
        display_id = str(
            (linked.get("payload") or {}).get("display_sample_id", "")
        )
        image = images.get(display_id)
        if image is None:
            rejected.append({"action_frame_id": request.get("action_frame_id"), "reason": "display_image_missing"})
            continue
        decision = float(
            (item["approved"].get("payload") or {}).get(
                "approved_backend_monotonic_s",
                (item["approved"].get("payload") or {}).get("decision_backend_monotonic_s"),
            )
            or item["approved"].get("backend_monotonic_s", 0.0)
        )
        if image.arrival_backend_monotonic_s > decision + 1e-9:
            rejected.append({"action_frame_id": request.get("action_frame_id"), "reason": "future_image"})
            continue
        if decision - image.capture_backend_monotonic_s > image_max_age_s:
            rejected.append({"action_frame_id": request.get("action_frame_id"), "reason": "image_age"})
            continue
        feedback_candidates = [
            sample
            for sample in reader.feedback
            if sample.arrival_backend_monotonic_s <= decision + 1e-9
        ]
        if not feedback_candidates:
            rejected.append({"action_frame_id": request.get("action_frame_id"), "reason": "feedback_missing"})
            continue
        feedback = max(feedback_candidates, key=lambda sample: sample.arrival_backend_monotonic_s)
        if decision - feedback.capture_backend_monotonic_s > feedback_max_age_s:
            rejected.append({"action_frame_id": request.get("action_frame_id"), "reason": "feedback_age"})
            continue
        model_q = list((feedback.value or {}).get("model_joint_positions_rad") or [])
        gripper = (feedback.value or {}).get("gripper_width_m")
        if len(model_q) != 5 or not isinstance(gripper, (int, float)):
            rejected.append({"action_frame_id": request.get("action_frame_id"), "reason": "feedback_state"})
            continue
        state = tuple(
            (float(model_q[index]) - ZERO_MODEL[index]) * DIRECTION[index]
            for index in range(5)
        ) + (float(gripper),)
        action_values = list(request.get("arm_user_joint_positions_rad") or []) + [
            request.get("gripper_width_m")
        ]
        if len(action_values) != 6 or not all(
            isinstance(value, (int, float)) and math.isfinite(float(value))
            for value in action_values
        ):
            rejected.append({"action_frame_id": request.get("action_frame_id"), "reason": "action_values"})
            continue
        receipt = linked.get("payload", {}).get("receipt") or {}
        rows.append(
            G4A2Row(
                reader=reader,
                image=image,
                feedback=feedback,
                decision_backend_monotonic_s=decision,
                human_intent_id=str(item["intent"].get("payload", {}).get("human_intent_id")),
                display_sample_id=display_id,
                action_frame_id=str(request.get("action_frame_id")),
                action_seq=int(request.get("action_seq", 0)),
                state=state,
                action=tuple(float(value) for value in action_values),
                assist_mode=str(item["intent"].get("payload", {}).get("assist_mode", "manual")),
                control_generator=str(item["intent"].get("payload", {}).get("control_generator", "unknown")),
                control_tick_id=receipt.get("control_tick_id") or request.get("control_tick_id"),
            )
        )
    return rows, rejected


def _rgb_array(path: Path) -> np.ndarray:
    with Image.open(path) as image:
        return np.asarray(image.convert("RGB"), dtype=np.uint8).transpose(2, 0, 1)


def _depth_array(path: Path) -> np.ndarray:
    value = np.load(path).astype(np.float32)
    if value.ndim != 2:
        raise ValueError(f"depth must be 2-D: {path}")
    return value[None, :, :]


def _mask_array(path: Path) -> np.ndarray:
    with Image.open(path) as image:
        value = np.asarray(image.convert("L"), dtype=np.uint8)
    return (value > 0).astype(np.uint8)[None, :, :]


def _features(height: int, width: int) -> dict[str, Any]:
    return {
        "observation.images.link4_rgb": {
            "dtype": "video",
            "shape": (3, height, width),
            "names": ["channels", "height", "width"],
        },
        "observation.depth_m": {
            "dtype": "float32",
            "shape": (1, height, width),
            "names": ["channel", "height", "width"],
        },
        "observation.depth_valid_mask": {
            "dtype": "uint8",
            "shape": (1, height, width),
            "names": ["channel", "height", "width"],
        },
        "observation.state": {
            "dtype": "float32",
            "shape": (6,),
            "names": ["joint1", "joint2", "joint3", "joint4", "joint5", "gripper_width_m"],
        },
        "action": {
            "dtype": "float32",
            "shape": (6,),
            "names": ["joint1", "joint2", "joint3", "joint4", "joint5", "gripper_width_m"],
        },
    }


def write_g4a2_cohort(
    entries: list[dict[str, Any]],
    *,
    run_root: str | Path,
    dataset_root: str | Path,
    repo_id: str = "mobilecar/g4a2-human-rgbd-dev",
    split_labels: list[str] | None = None,
) -> dict[str, Any]:
    """Write a small cohort using only the locked official LeRobot API."""

    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.datasets.utils import write_info

    root = Path(dataset_root)
    if root.exists():
        raise FileExistsError(f"refusing to overwrite {root}")
    materialized: list[tuple[dict[str, Any], EpisodeReader, list[G4A2Row], dict[str, str]]] = []
    for entry in entries:
        reader = EpisodeReader(run_root, entry["run_id"], entry["episode_id"])
        rows, rejected = extract_g4a2_rows(reader)
        if not rows:
            raise ValueError(f"no human export rows for {entry['episode_id']}: {rejected[:3]}")
        before = reader.source_file_hashes()
        materialized.append((entry, reader, rows, before))
    first = materialized[0][2][0]
    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        fps=5,
        features=_features(first.image.height, first.image.width),
        root=root,
        robot_type="landerpi_mecanum_5dof",
        use_videos=True,
        video_backend="pyav",
        batch_encoding_size=1,
    )
    episode_lengths = []
    for entry, reader, rows, _ in materialized:
        for row in rows:
            dataset.add_frame(
                {
                    "observation.images.link4_rgb": _rgb_array(reader.episode_dir / row.image.rgb_uri),
                    "observation.depth_m": _depth_array(reader.episode_dir / row.image.depth_uri),
                    "observation.depth_valid_mask": _mask_array(reader.episode_dir / row.image.valid_mask_uri),
                    "observation.state": np.asarray(row.state, dtype=np.float32),
                    "action": np.asarray(row.action, dtype=np.float32),
                    "task": reader.task_language,
                }
            )
        dataset.save_episode(parallel_encoding=False)
        episode_lengths.append(len(rows))
    dataset.finalize()
    if split_labels is not None:
        if len(split_labels) != len(materialized):
            raise ValueError("split_labels must match cohort entries")
        ranges: dict[str, list[int]] = {}
        for index, split in enumerate(split_labels):
            ranges.setdefault(split, [index, index + 1])
            ranges[split][1] = index + 1
        dataset.meta.info["splits"] = {
            split: f"{bounds[0]}:{bounds[1]}" for split, bounds in ranges.items()
        }
        write_info(dataset.meta.info, dataset.root)
    loaded = LeRobotDataset(repo_id, root=root, video_backend="pyav")
    if len(loaded) != sum(episode_lengths):
        raise AssertionError("official loader frame count mismatch")
    sample_shapes = {}
    for index in range(len(loaded)):
        item = loaded[index]
        sample_shapes[index] = {
            "rgb": list(item["observation.images.link4_rgb"].shape),
            "depth": list(item["observation.depth_m"].shape),
            "mask": list(item["observation.depth_valid_mask"].shape),
            "state": list(item["observation.state"].shape),
            "action": list(item["action"].shape),
        }
    source_hashes = {}
    for entry, reader, _, before in materialized:
        after = reader.source_file_hashes()
        if after != before:
            raise RuntimeError(f"source changed during export: {entry['episode_id']}")
        source_hashes[entry["episode_id"]] = {
            "before": sha256_json(sorted(before.items())),
            "after": sha256_json(sorted(after.items())),
            "unchanged": before == after,
        }
    return {
        "status": "synthetic_fixture" if all(e.get("status") == "synthetic_fixture" for e in entries) else "dev",
        "training_qualified": False,
        "export_valid": True,
        "num_frames": sum(episode_lengths),
        "num_episodes": len(materialized),
        "episode_lengths": episode_lengths,
        "splits": dataset.meta.info.get("splits"),
        "source_hashes": source_hashes,
        "sample_shapes": sample_shapes,
        "dataset_root": str(root),
    }
