#!/usr/bin/env python3
"""Create and read back a tiny synthetic multi-Episode RGB-D LeRobot dataset."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--repo-id", default="mobilecar/g4a2-rgbd-smoke-dev")
    args = parser.parse_args()
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    root = args.root.resolve()
    if root.exists():
        raise FileExistsError(root)
    height = width = 32
    features = {
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
            "names": ["q1", "q2", "q3", "q4", "q5", "gripper_width_m"],
        },
        "action": {
            "dtype": "float32",
            "shape": (6,),
            "names": ["q1", "q2", "q3", "q4", "q5", "gripper_width_m"],
        },
    }
    dataset = LeRobotDataset.create(
        repo_id=args.repo_id,
        fps=5,
        features=features,
        root=root,
        robot_type="landerpi_mecanum_5dof",
        use_videos=True,
        video_backend="pyav",
        batch_encoding_size=1,
    )
    for episode in range(2):
        for frame in range(3):
            rgb = np.zeros((3, height, width), dtype=np.uint8)
            rgb[0] = 20 + episode + frame
            depth = np.full((1, height, width), 0.2 + 0.01 * frame, dtype=np.float32)
            mask = np.ones((1, height, width), dtype=np.uint8)
            state = np.zeros(6, dtype=np.float32)
            action = np.zeros(6, dtype=np.float32)
            action[5] = 0.025
            dataset.add_frame(
                {
                    "observation.images.link4_rgb": rgb,
                    "observation.depth_m": depth,
                    "observation.depth_valid_mask": mask,
                    "observation.state": state,
                    "action": action,
                    "task": "synthetic G4A-2 RGB-D writer smoke",
                }
            )
        dataset.save_episode(parallel_encoding=False)
    dataset.finalize()
    loaded = LeRobotDataset(args.repo_id, root=root, video_backend="pyav")
    assert len(loaded) == 6
    shapes = {}
    for index in range(len(loaded)):
        item = loaded[index]
        shapes[index] = {
            "rgb": list(item["observation.images.link4_rgb"].shape),
            "depth": list(item["observation.depth_m"].shape),
            "mask": list(item["observation.depth_valid_mask"].shape),
            "state": list(item["observation.state"].shape),
            "action": list(item["action"].shape),
        }
    result = {
        "status": "synthetic_fixture",
        "training_qualified": False,
        "num_frames": len(loaded),
        "num_episodes": loaded.num_episodes,
        "shapes": shapes,
    }
    (root.parent / "g4a2_rgbd_writer_smoke.json").write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
