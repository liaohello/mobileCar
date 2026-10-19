"""Save and serialize an actual slam_toolbox map, then freeze its manifest."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
import yaml
from slam_toolbox.srv import SaveMap, SerializePoseGraph

from .map_package import build_manifest


def call_service(node, client, request, timeout_s=10.0):
    if not client.wait_for_service(timeout_sec=timeout_s):
        raise TimeoutError(f"service unavailable: {client.srv_name}")
    future = client.call_async(request)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and not future.done():
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done():
        raise TimeoutError(f"service timeout: {client.srv_name}")
    return future.result()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-root", type=Path, required=True)
    parser.add_argument("--map-id", required=True)
    parser.add_argument("--scene-id", required=True)
    parser.add_argument("--model-sha256", required=True)
    parser.add_argument("--profile-id", required=True)
    parser.add_argument("--profile-version", default="1.0")
    parser.add_argument("--profile-sha256", required=True)
    parser.add_argument("--navigation-profile-sha256", required=True)
    parser.add_argument("--sensor-profile-sha256", required=True)
    parser.add_argument("--scan-config-sha256", required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--episode-id", required=True)
    parser.add_argument("--navigation-epoch", type=int, required=True)
    parser.add_argument("--resolution-m", type=float, default=0.05)
    parser.add_argument("--occupied-thresh", type=float, default=0.65)
    parser.add_argument("--free-thresh", type=float, default=0.196)
    parser.add_argument("--initial-x", type=float, default=0.0)
    parser.add_argument("--initial-y", type=float, default=0.0)
    parser.add_argument("--initial-yaw", type=float, default=0.0)
    parser.add_argument("--output-manifest", type=Path, required=True)
    args = parser.parse_args()

    map_root = args.map_root.resolve()
    map_root.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node("g3b_mapping_manager")
    save_client = node.create_client(SaveMap, "/slam_toolbox/save_map")
    serialize_client = node.create_client(
        SerializePoseGraph, "/slam_toolbox/serialize_map"
    )
    report = {
        "map_id": args.map_id,
        "map_root": str(map_root),
        "save": {},
        "serialize": {},
    }
    try:
        save_request = SaveMap.Request()
        save_request.name.data = str(map_root / "map")
        save_result = call_service(node, save_client, save_request)
        report["save"] = {"result": int(save_result.result)}
        if int(save_result.result) != SaveMap.Response.RESULT_SUCCESS:
            raise RuntimeError(f"save_map failed: {save_result.result}")

        serialize_request = SerializePoseGraph.Request()
        serialize_request.filename = str(map_root / "posegraph")
        serialize_result = call_service(
            node, serialize_client, serialize_request
        )
        report["serialize"] = {"result": int(serialize_result.result)}
        if int(serialize_result.result) != SerializePoseGraph.Response.RESULT_SUCCESS:
            raise RuntimeError(
                f"serialize_map failed: {serialize_result.result}"
            )

        yaml_path = map_root / "map.yaml"
        map_yaml = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
        if not 0.0 < args.free_thresh < args.occupied_thresh < 1.0:
            raise ValueError("map thresholds must satisfy 0 < free < occupied < 1")
        map_yaml["mode"] = "trinary"
        map_yaml["occupied_thresh"] = float(args.occupied_thresh)
        map_yaml["free_thresh"] = float(args.free_thresh)
        yaml_path.write_text(
            yaml.safe_dump(map_yaml, sort_keys=False),
            encoding="utf-8",
        )
        report["thresholds"] = {
            "mode": "trinary",
            "occupied_thresh": float(args.occupied_thresh),
            "free_thresh": float(args.free_thresh),
            "unknown_is_traversable": False,
        }
        prior = {
            "x_m": args.initial_x,
            "y_m": args.initial_y,
            "yaw_rad": args.initial_yaw,
            "covariance": [0.0] * 36,
        }
        manifest = build_manifest(
            package_root=map_root,
            profile_id=args.profile_id,
            profile_version=args.profile_version,
            profile_sha256=args.profile_sha256,
            map_id=args.map_id,
            scene_id=args.scene_id,
            model_sha256=args.model_sha256,
            navigation_profile_sha256=args.navigation_profile_sha256,
            sensor_profile_sha256=args.sensor_profile_sha256,
            scan_config_sha256=args.scan_config_sha256,
            resolution_m=float(map_yaml.get("resolution", args.resolution_m)),
            origin_xy_yaw=list(map_yaml.get("origin", [0.0, 0.0, 0.0])),
            occupied_thresh=float(map_yaml.get("occupied_thresh", 0.65)),
            free_thresh=float(map_yaml["free_thresh"]),
            created_run_id=args.run_id,
            created_episode_id=args.episode_id,
            created_navigation_epoch=args.navigation_epoch,
            initial_prior=prior,
        )
        args.output_manifest.parent.mkdir(parents=True, exist_ok=True)
        args.output_manifest.write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        report["manifest"] = str(args.output_manifest)
        report["pose_graph_serialized"] = True
        report["success"] = True
    except Exception as exc:
        report["success"] = False
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(report, indent=2))
    return 0 if report["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
