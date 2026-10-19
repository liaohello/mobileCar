from __future__ import annotations

import math
from pathlib import Path

import mujoco
import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[2]
ROOM_B_MODEL = ROOT / "assets/landerpi/model/robot_g3b_l_shape.xml"
ROOM_B_SCENE = ROOT / "assets/landerpi/scenes/g3b_l_shape_wide_door.xml"
PROFILE = ROOT / "configs/profiles/g3b_navigation_v1.yaml"
NAV2_CONFIG = (
    ROOT / "ros2_ws/src/mobilecar_nav_bringup/config/nav2_g3b.yaml"
)


def model_and_data() -> tuple[mujoco.MjModel, mujoco.MjData]:
    model = mujoco.MjModel.from_xml_path(str(ROOM_B_MODEL))
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    return model, data


def ancestor_ids(model: mujoco.MjModel, body_id: int) -> set[int]:
    result = {body_id}
    current = body_id
    while current > 0:
        current = int(model.body_parentid[current])
        result.add(current)
    return result


def geom_points(
    model: mujoco.MjModel, data: mujoco.MjData, geom_id: int
) -> np.ndarray:
    geom_type = int(model.geom_type[geom_id])
    center = np.asarray(data.geom_xpos[geom_id], dtype=float)
    rotation = np.asarray(data.geom_xmat[geom_id], dtype=float).reshape(3, 3)
    if geom_type == mujoco.mjtGeom.mjGEOM_MESH:
        mesh_id = int(model.geom_dataid[geom_id])
        start = int(model.mesh_vertadr[mesh_id])
        count = int(model.mesh_vertnum[mesh_id])
        vertices = np.asarray(model.mesh_vert[start : start + count], dtype=float)
        return vertices @ rotation.T + center
    if geom_type == mujoco.mjtGeom.mjGEOM_BOX:
        size = np.asarray(model.geom_size[geom_id], dtype=float)
        signs = np.array(
            [
                [-1, -1, -1],
                [-1, -1, 1],
                [-1, 1, -1],
                [-1, 1, 1],
                [1, -1, -1],
                [1, -1, 1],
                [1, 1, -1],
                [1, 1, 1],
            ],
            dtype=float,
        )
        return (signs * size) @ rotation.T + center
    radius = float(model.geom_rbound[geom_id])
    offsets = np.array(
        [
            [x, y, z]
            for x in (-radius, radius)
            for y in (-radius, radius)
            for z in (-radius, radius)
        ],
        dtype=float,
    )
    return offsets + center


def robot_envelope(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, float]:
    base_id = int(model.body("base_link").id)
    descendants = {
        body_id
        for body_id in range(model.nbody)
        if base_id in ancestor_ids(model, body_id)
    }
    points = []
    for geom_id in range(model.ngeom):
        body_id = int(model.geom_bodyid[geom_id])
        if body_id not in descendants:
            continue
        points.append(geom_points(model, data, geom_id))
    cloud = np.concatenate(points, axis=0)
    base_position = np.asarray(data.xpos[base_id], dtype=float)
    local = cloud - base_position
    return {
        "x_min": float(local[:, 0].min()),
        "x_max": float(local[:, 0].max()),
        "y_min": float(local[:, 1].min()),
        "y_max": float(local[:, 1].max()),
        "z_max": float(local[:, 2].max()),
    }


def test_room_b_loads_with_free_base_arm_and_link4_camera():
    model, _data = model_and_data()
    assert model.body("base_link").id >= 0
    assert model.joint("base_free").type == mujoco.mjtJoint.mjJNT_FREE
    for index in range(1, 6):
        assert model.joint(f"joint{index}").id >= 0
    camera_body = int(model.body("depth_cam_link").id)
    assert int(model.body("link4").id) in ancestor_ids(model, camera_body)
    assert ROOM_B_SCENE.is_file()
    scene_text = ROOM_B_SCENE.read_text(encoding="utf-8")
    assert "divider_south" in scene_text
    assert "divider_north" in scene_text
    assert "divider_fin" in scene_text


def test_declared_arm_activity_stays_inside_frozen_footprint():
    model, data = model_and_data()
    profile = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))
    polygon = np.asarray(profile["nav2"]["footprint_polygon_m"], dtype=float)
    limits = {
        "x_min": float(polygon[:, 0].min()),
        "x_max": float(polygon[:, 0].max()),
        "y_min": float(polygon[:, 1].min()),
        "y_max": float(polygon[:, 1].max()),
    }
    joint_qpos = [
        int(model.jnt_qposadr[model.joint(f"joint{index}").id])
        for index in range(1, 6)
    ]
    low = np.asarray(data.qpos[joint_qpos], dtype=float)
    high = low.copy()
    high[0] += 0.12
    high[1] += 0.05
    for fraction in np.linspace(0.0, 1.0, 11):
        data.qpos[joint_qpos] = low + fraction * (high - low)
        mujoco.mj_forward(model, data)
        envelope = robot_envelope(model, data)
        for key, boundary in limits.items():
            value = envelope[key]
            assert value >= boundary - 1e-6 if key.endswith("min") else value <= boundary + 1e-6
    assert all(math.isfinite(value) for value in envelope.values())


def test_laser_height_intersects_room_b_barriers():
    model, data = model_and_data()
    base_id = int(model.body("base_link").id)
    origin = np.asarray(data.xpos[base_id], dtype=float) + np.array([0.073, 0.0, 0.08])
    hit_id = np.array([-1], dtype=np.int32)
    distance = float(
        mujoco.mj_ray(
            model,
            data,
            origin,
            np.array([1.0, 0.0, 0.0]),
            np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8),
            1,
            -1,
            hit_id,
        )
    )
    assert 0.0 < distance < 8.0
    assert hit_id[0] >= 0


def test_nav2_internal_yaw_goal_is_tighter_than_acceptance():
    profile = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))
    nav2 = yaml.safe_load(NAV2_CONFIG.read_text(encoding="utf-8"))
    internal_rad = float(
        nav2["controller_server"]["ros__parameters"]["goal_checker"][
            "yaw_goal_tolerance"
        ]
    )
    acceptance_rad = math.radians(
        profile["acceptance"]["goal_yaw_tolerance_deg"]
    )
    assert internal_rad < acceptance_rad
    progress = nav2["controller_server"]["ros__parameters"][
        "progress_checker"
    ]
    assert progress["plugin"] == "nav2_controller::PoseProgressChecker"
    assert float(progress["required_movement_angle"]) > 0.0
