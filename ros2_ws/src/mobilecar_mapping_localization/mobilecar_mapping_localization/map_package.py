"""Map package hashing, compatibility and manifest helpers without ROS imports."""

from __future__ import annotations

import hashlib
import json
import math
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping


REQUIRED_ROLES = {
    "occupancy_yaml",
    "occupancy_image",
    "pose_graph",
    "pose_graph_data",
    "map_manifest",
}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def atomic_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def canonical_manifest_digest(payload: Mapping[str, Any]) -> str:
    canonical = json.loads(json.dumps(payload, sort_keys=True))
    canonical["assets"] = [
        item for item in canonical.get("assets", []) if item.get("role") != "map_manifest"
    ]
    encoded = json.dumps(
        canonical, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def asset(root: Path, role: str, path: Path) -> dict[str, Any]:
    relative = path.resolve().relative_to(root.resolve()).as_posix()
    return {
        "role": role,
        "relative_path": relative,
        "sha256": sha256_file(path),
        "bytes": path.stat().st_size,
    }


def build_manifest(
    *,
    package_root: Path,
    profile_id: str,
    profile_version: str,
    profile_sha256: str,
    map_id: str,
    scene_id: str,
    model_sha256: str,
    navigation_profile_sha256: str,
    sensor_profile_sha256: str,
    scan_config_sha256: str,
    resolution_m: float,
    origin_xy_yaw: list[float],
    occupied_thresh: float,
    free_thresh: float,
    created_run_id: str,
    created_episode_id: str,
    created_navigation_epoch: int,
    initial_prior: Mapping[str, Any] | None,
    rviz_config: Path | None = None,
) -> dict[str, Any]:
    package_root = package_root.resolve()
    yaml_path = package_root / "map.yaml"
    image_candidates = list(package_root.glob("map.pgm")) + list(
        package_root.glob("map.png")
    )
    pose_graph = package_root / "posegraph.posegraph"
    pose_graph_data = package_root / "posegraph.data"
    required = [yaml_path, pose_graph, pose_graph_data]
    if len(image_candidates) != 1:
        raise FileNotFoundError("map package requires exactly one map.pgm or map.png")
    required.append(image_candidates[0])
    for path in required:
        if not path.is_file() or path.stat().st_size <= 0:
            raise FileNotFoundError(f"map package asset missing or empty: {path}")
    manifest_path = package_root / "manifest.json"
    digest_path = package_root / "manifest.sha256"
    assets = [
        asset(package_root, "occupancy_yaml", yaml_path),
        asset(package_root, "occupancy_image", image_candidates[0]),
        asset(package_root, "pose_graph", pose_graph),
        asset(package_root, "pose_graph_data", pose_graph_data),
    ]
    if rviz_config is not None and rviz_config.is_file():
        assets.append(asset(package_root, "rviz_config", rviz_config))
    payload = {
        "schema_version": 1,
        "interface_profile": "g3b-1.0",
        "profile_id": profile_id,
        "profile_version": profile_version,
        "profile_sha256": profile_sha256,
        "map_id": map_id,
        "scene_id": scene_id,
        "model_sha256": model_sha256,
        "navigation_profile_sha256": navigation_profile_sha256,
        "sensor_profile_sha256": sensor_profile_sha256,
        "scan_config_sha256": scan_config_sha256,
        "map_frame": "map",
        "odom_frame": "odom",
        "base_frame": "base_link",
        "resolution_m": float(resolution_m),
        "origin_xy_yaw": [float(value) for value in origin_xy_yaw],
        "occupied_thresh": float(occupied_thresh),
        "free_thresh": float(free_thresh),
        "created_run_id": created_run_id,
        "created_episode_id": created_episode_id,
        "created_navigation_epoch": int(created_navigation_epoch),
        "serialized_pose_graph": True,
        "initial_prior": dict(initial_prior) if initial_prior is not None else None,
        "assets": assets,
        "compatibility_fields": [
            "scene_id",
            "model_sha256",
            "navigation_profile_sha256",
            "sensor_profile_sha256",
            "scan_config_sha256",
            "resolution_m",
            "map_frame",
            "odom_frame",
            "base_frame",
        ],
        "created_wall_utc": datetime.now(timezone.utc)
        .isoformat()
        .replace("+00:00", "Z"),
    }
    digest = canonical_manifest_digest(payload)
    digest_path.write_text(f"{digest}  manifest.json\n", encoding="utf-8")
    payload["assets"].append(
        asset(package_root, "map_manifest", digest_path)
    )
    atomic_json(manifest_path, payload)
    return payload


def load_manifest(package_root: Path) -> dict[str, Any]:
    manifest_path = package_root / "manifest.json"
    payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    if payload.get("interface_profile") != "g3b-1.0":
        raise ValueError("unsupported map package interface")
    roles = {item["role"] for item in payload["assets"]}
    if not REQUIRED_ROLES.issubset(roles):
        raise ValueError("map package missing required assets")
    for item in payload["assets"]:
        path = package_root / item["relative_path"]
        if not path.is_file():
            raise FileNotFoundError(f"map asset missing: {item['relative_path']}")
        if sha256_file(path) != item["sha256"]:
            raise ValueError(f"map asset hash mismatch: {item['relative_path']}")
    digest_asset = next(
        (item for item in payload["assets"] if item["role"] == "map_manifest"),
        None,
    )
    if digest_asset is None:
        raise ValueError("map package manifest digest missing")
    digest_line = (package_root / digest_asset["relative_path"]).read_text(
        encoding="utf-8"
    ).strip().split()
    if not digest_line or digest_line[0] != canonical_manifest_digest(payload):
        raise ValueError("canonical map manifest digest mismatch")
    return payload


def compatibility_diff(
    manifest: Mapping[str, Any], current: Mapping[str, Any]
) -> dict[str, Any]:
    fields = manifest.get("compatibility_fields", [])
    mismatches = {
        field: {
            "map": manifest.get(field),
            "current": current.get(field),
        }
        for field in fields
        if manifest.get(field) != current.get(field)
    }
    return {
        "compatible": not mismatches,
        "mismatches": mismatches,
        "checked_fields": list(fields),
        "whole_source_aggregate_used": False,
    }


def read_pgm(path: Path) -> dict[str, Any]:
    """Read the binary P5 occupancy image emitted by slam_toolbox."""

    payload = path.read_bytes()
    if not payload.startswith(b"P5"):
        raise ValueError("occupancy image must be a binary P5 PGM")
    tokens: list[bytes] = []
    index = 2
    while len(tokens) < 3:
        while index < len(payload) and payload[index] in b" \t\r\n":
            index += 1
        if index < len(payload) and payload[index] == ord("#"):
            newline = payload.find(b"\n", index)
            if newline < 0:
                raise ValueError("unterminated PGM comment")
            index = newline + 1
            continue
        start = index
        while index < len(payload) and payload[index] not in b" \t\r\n":
            index += 1
        if start == index:
            raise ValueError("invalid PGM header")
        tokens.append(payload[start:index])
    if payload[index : index + 2] == b"\r\n":
        index += 2
    elif index < len(payload) and payload[index] in b" \t\r\n":
        index += 1
    width, height, maximum = (int(value) for value in tokens)
    if width <= 0 or height <= 0 or maximum != 255:
        raise ValueError("unsupported PGM dimensions or maximum value")
    pixels = payload[index : index + width * height]
    if len(pixels) != width * height:
        raise ValueError("truncated PGM pixel data")
    return {
        "width": width,
        "height": height,
        "maximum": maximum,
        "pixels": pixels,
    }


def load_occupancy_package(package_root: Path) -> dict[str, Any]:
    """Load and validate YAML/asset references without importing ROS."""

    import yaml

    package_root = package_root.resolve()
    manifest = load_manifest(package_root)
    yaml_path = package_root / "map.yaml"
    metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    image_name = str(metadata.get("image", ""))
    image_path = (package_root / image_name).resolve()
    try:
        image_path.relative_to(package_root)
    except ValueError as exc:
        raise ValueError("occupancy image escapes map package") from exc
    image = read_pgm(image_path)
    if str(metadata.get("mode", "trinary")) != "trinary":
        raise ValueError("only trinary occupancy maps are accepted")
    origin = [float(value) for value in metadata["origin"]]
    if len(origin) != 3 or not all(math.isfinite(value) for value in origin):
        raise ValueError("occupancy origin must contain three finite values")
    resolution = float(metadata["resolution"])
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("occupancy resolution must be positive")
    return {
        "manifest": manifest,
        "metadata": metadata,
        "image": image,
        "resolution_m": resolution,
        "origin_xy_yaw": origin,
        "occupied_thresh": float(metadata["occupied_thresh"]),
        "free_thresh": float(metadata["free_thresh"]),
        "negate": bool(metadata.get("negate", 0)),
    }


def world_to_grid(
    package: Mapping[str, Any], x_m: float, y_m: float
) -> tuple[int, int]:
    """Return row and column for a map-coordinate world point."""

    x = float(x_m)
    y = float(y_m)
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError("goal coordinates must be finite")
    resolution = float(package["resolution_m"])
    origin_x, origin_y, yaw = (float(value) for value in package["origin_xy_yaw"])
    dx = x - origin_x
    dy = y - origin_y
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    column = math.floor(local_x / resolution)
    row_from_bottom = math.floor(local_y / resolution)
    image = package["image"]
    return int(image["height"]) - 1 - row_from_bottom, column


def occupancy_goal_check(
    package_root: Path,
    x_m: float,
    y_m: float,
) -> dict[str, Any]:
    """Classify a goal cell and conservatively reject unknown or occupied cells."""

    package = load_occupancy_package(package_root)
    row, column = world_to_grid(package, x_m, y_m)
    image = package["image"]
    width = int(image["width"])
    height = int(image["height"])
    if not (0 <= row < height and 0 <= column < width):
        return {
            "accepted": False,
            "classification": "outside_map",
            "row": row,
            "column": column,
            "x_m": float(x_m),
            "y_m": float(y_m),
        }
    gray = int(image["pixels"][row * width + column])
    probability = gray / float(image["maximum"])
    if package["negate"]:
        probability = 1.0 - probability
    occupied_probability = 1.0 - probability
    if occupied_probability > package["occupied_thresh"]:
        classification = "occupied"
    elif occupied_probability < package["free_thresh"]:
        classification = "free"
    else:
        classification = "unknown"
    return {
        "accepted": classification == "free",
        "classification": classification,
        "row": row,
        "column": column,
        "gray_value": gray,
        "occupied_probability": occupied_probability,
        "free_thresh": package["free_thresh"],
        "occupied_thresh": package["occupied_thresh"],
        "x_m": float(x_m),
        "y_m": float(y_m),
    }
