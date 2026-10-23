#!/usr/bin/env python3
"""G4A-2R Phase 2: server-side validation and freezing of named postures.

For every candidate posture (user-coordinate arm angles + gripper width) this
tool checks, entirely on the MuJoCo model used by the live service:

  stage 1 (all candidates):
    - joint limits (model jnt_range)
    - self-collision at the posture (robot-tree geom contacts, excluding the
      model's base_link/link1 exclude pair)
    - link4 camera height, optical-axis pitch, RGB-D depth valid fraction
    - target cube and grasp_tcp inside the camera frame
  stage 2 (best candidates per posture):
    - stability: 3 s physics settle with position servos holding the target;
      base height/orientation drift and target reach error must stay small
    - 360 deg sweep: base yaw rotated in place; no arm-cube contact and no
      self-collision at any yaw

The best candidate per posture is written back into the posture profile
(frozen: true plus the validation summary), and a JSON evidence report is
written. Angles are never hand-guessed: the frozen values come from this run.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import time
from pathlib import Path

import mujoco
import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[2]

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5"]
GRIPPER_DRIVE = "gripper_base_joint"
MIMIC_JOINTS = ["gripper_left_join", "gripper_right_joint", "left_jaw_joint", "right_jaw_joint"]


def _load_params() -> dict:
    import sys

    sys.path.insert(0, str(REPO / "src"))
    from mobilecar.backends.modelgen.parameters import load_parameters

    return load_parameters()


def _quat_from_yaw(yaw: float) -> np.ndarray:
    return np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])


def _robot_body_ids(model: mujoco.MjModel) -> set[int]:
    base_id = model.body("base_link").id
    robot = {base_id}
    for body_id in range(1, model.nbody):
        current = body_id
        while current not in (0, base_id):
            current = int(model.body_parentid[current])
        if current == base_id:
            robot.add(body_id)
    return robot


def _contacts_between(model, data, bodies_a: set[int], bodies_b: set[int]) -> list[dict]:
    hits = []
    for index in range(data.ncon):
        contact = data.contact[index]
        geom_a, geom_b = contact.geom1, contact.geom2
        body_a = int(model.geom_bodyid[geom_a])
        body_b = int(model.geom_bodyid[geom_b])
        pair = {body_a, body_b}
        if (body_a in bodies_a and body_b in bodies_b) or (
            body_a in bodies_b and body_b in bodies_a
        ):
            if pair == {model.body("base_link").id, model.body("link1").id}:
                continue  # explicit model exclude pair
            hits.append(
                {
                    "geom1": model.geom(geom_a).name,
                    "geom2": model.geom(geom_b).name,
                    "dist_m": float(contact.dist),
                }
            )
    return hits


def _apply_posture(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    coordinates,
    arm_spec: dict,
    user_angles: list[float],
    width_m: float,
    base_qpos: list[float],
    cube_position: list[float],
    base_yaw: float = 0.0,
) -> None:
    mujoco.mj_resetData(model, data)
    base = list(base_qpos)
    base[3:7] = _quat_from_yaw(base_yaw)
    data.qpos[model.jnt_qposadr[model.joint("base_free").id] :][:7] = np.asarray(base)
    model_angles = coordinates.to_model(np.asarray(user_angles, dtype=float))
    for index, name in enumerate(ARM_JOINTS):
        data.qpos[model.jnt_qposadr[model.joint(name).id]] = float(model_angles[index])
    gripper = arm_spec["gripper"]
    drive = float(
        np.clip(
            (width_m - gripper["max_width_m"]) / gripper["width_per_drive_rad_m"],
            *[-1.638, 0.0],
        )
    )
    data.qpos[model.jnt_qposadr[model.joint(GRIPPER_DRIVE).id]] = drive
    for joint, mimic in arm_spec["mimic"].items():
        data.qpos[model.jnt_qposadr[model.joint(joint).id]] = (
            float(mimic["multiplier"]) * drive + float(mimic.get("offset", 0.0))
        )
    cube_qpos = int(model.jnt_qposadr[model.body_jntadr[model.body("target_cube").id]])
    data.qpos[cube_qpos : cube_qpos + 3] = np.asarray(cube_position, dtype=float)
    mujoco.mj_forward(model, data)


def _project(point, cam_pos, cam_mat, fovy_deg, width, height) -> tuple[float, float, bool]:
    v = np.asarray(cam_mat, dtype=float) @ (np.asarray(point, dtype=float) - np.asarray(cam_pos))
    if v[2] >= -1e-6:
        return 0.0, 0.0, False
    tanf = math.tan(math.radians(fovy_deg) / 2.0)
    x_ndc = v[0] / (-v[2] * tanf * (width / height))
    y_ndc = v[1] / (-v[2] * tanf)
    in_frame = abs(x_ndc) <= 1.0 and abs(y_ndc) <= 1.0
    return (x_ndc + 1) / 2 * width, (1 - y_ndc) / 2 * height, in_frame


def _angular_offset(cam_pos, cam_mat, point) -> float:
    """Angle between the camera view axis and the direction to a point."""
    view = -np.asarray(cam_mat, dtype=float)[2, :]
    to_point = np.asarray(point, dtype=float) - np.asarray(cam_pos, dtype=float)
    norm = np.linalg.norm(to_point)
    if norm < 1e-9:
        return 0.0
    cosine = float(np.clip(np.dot(view, to_point / norm), -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def _camera_metrics(model, data, renderer, camera_cfg: dict) -> dict:
    cam_id = model.camera("rgbd").id
    cam_pos = np.asarray(data.cam_xpos[cam_id], dtype=float)
    cam_mat = np.asarray(data.cam_xmat[cam_id], dtype=float).reshape(3, 3)
    view_dir = -cam_mat[2, :]
    pitch_deg = math.degrees(math.asin(float(np.clip(view_dir[2], -1.0, 1.0))))
    width, height = int(camera_cfg["width"]), int(camera_cfg["height"])
    fovy = float(model.cam_fovy[cam_id])
    cube_id = model.body("target_cube").id
    tcp_id = model.site("grasp_tcp").id
    left_finger_id = model.site("left_finger_contact").id
    right_finger_id = model.site("right_finger_contact").id
    cube_px, cube_py, cube_in = _project(
        data.xpos[cube_id], cam_pos, cam_mat, fovy, width, height
    )
    tcp_px, tcp_py, tcp_in = _project(
        data.site_xpos[tcp_id], cam_pos, cam_mat, fovy, width, height
    )
    _, _, left_finger_in = _project(
        data.site_xpos[left_finger_id], cam_pos, cam_mat, fovy, width, height
    )
    _, _, right_finger_in = _project(
        data.site_xpos[right_finger_id], cam_pos, cam_mat, fovy, width, height
    )
    cube_distance = float(np.linalg.norm(data.xpos[cube_id] - cam_pos))
    renderer.enable_depth_rendering()
    renderer.update_scene(data, camera="rgbd")
    depth = renderer.render().copy()
    renderer.disable_depth_rendering()
    valid = np.isfinite(depth) & (depth >= camera_cfg["depth_min_m"]) & (
        depth <= camera_cfg["depth_max_m"]
    )
    return {
        "camera_height_m": float(data.xpos[model.cam_bodyid[cam_id]][2]),
        "camera_position_m": [float(v) for v in cam_pos],
        "camera_view_yaw_deg": math.degrees(math.atan2(float(view_dir[1]), float(view_dir[0]))),
        "optical_axis_pitch_deg": pitch_deg,
        "cube_angular_offset_deg": _angular_offset(cam_pos, cam_mat, data.xpos[cube_id]),
        "tcp_angular_offset_deg": _angular_offset(cam_pos, cam_mat, data.site_xpos[tcp_id]),
        "depth_valid_fraction": float(np.mean(valid)),
        "cube": {
            "in_frame": bool(cube_in),
            "pixel": [round(float(cube_px), 1), round(float(cube_py), 1)],
            "distance_m": cube_distance,
        },
        "tcp": {
            "in_frame": bool(tcp_in),
            "pixel": [round(float(tcp_px), 1), round(float(tcp_py), 1)],
        },
        "gripper": {
            "left_finger_in_frame": bool(left_finger_in),
            "right_finger_in_frame": bool(right_finger_in),
            "in_frame": bool(left_finger_in or right_finger_in),
        },
    }


def _settle_check(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    coordinates,
    arm_spec: dict,
    user_angles: list[float],
    width_m: float,
    base_qpos: list[float],
    cube_position: list[float],
    seconds: float = 3.0,
) -> dict:
    _apply_posture(model, data, coordinates, arm_spec, user_angles, width_m, base_qpos, cube_position)
    model_angles = coordinates.to_model(np.asarray(user_angles, dtype=float))
    gripper = arm_spec["gripper"]
    drive = float(
        np.clip(
            (width_m - gripper["max_width_m"]) / gripper["width_per_drive_rad_m"],
            -1.638,
            0.0,
        )
    )
    data.ctrl[:] = 0.0
    for index, name in enumerate(ARM_JOINTS):
        data.ctrl[model.actuator(name + "_motor").id] = float(model_angles[index])
    data.ctrl[model.actuator("gripper_motor").id] = drive
    base_id = model.body("base_link").id
    base_qpos_adr = model.jnt_qposadr[model.joint("base_free").id]
    initial_z = float(data.qpos[base_qpos_adr + 2])
    initial_quat = np.asarray(data.qpos[base_qpos_adr + 3 : base_qpos_adr + 7], dtype=float)
    steps = int(seconds / float(model.opt.timestep))
    for _ in range(steps):
        mujoco.mj_step(model, data)
    final_quat = np.asarray(data.qpos[base_qpos_adr + 3 : base_qpos_adr + 7], dtype=float)
    drift_z = float(data.qpos[base_qpos_adr + 2]) - initial_z
    # angle between initial and final base orientation
    dot = abs(float(np.dot(initial_quat / np.linalg.norm(initial_quat), final_quat / np.linalg.norm(final_quat))))
    base_tilt_deg = math.degrees(2.0 * math.acos(min(1.0, dot)))
    reach_error = float(
        np.max(
            np.abs(
                data.qpos[[model.jnt_qposadr[model.joint(n).id] for n in ARM_JOINTS]]
                - model_angles
            )
        )
    )
    mujoco.mj_forward(model, data)
    return {
        "settle_seconds": seconds,
        "base_height_drift_m": drift_z,
        "base_tilt_deg": base_tilt_deg,
        "arm_reach_error_rad": reach_error,
        "stable": abs(drift_z) < 0.01 and base_tilt_deg < 3.0 and reach_error < 0.05,
    }


def _sweep_check(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    coordinates,
    arm_spec: dict,
    user_angles: list[float],
    width_m: float,
    base_qpos: list[float],
    cube_position: list[float],
    step_deg: float = 10.0,
) -> dict:
    robot = _robot_body_ids(model)
    cube_bodies = {model.body("target_cube").id}
    cube_visible = 0
    angles = 0
    collisions = []
    for index in range(int(round(360.0 / step_deg))):
        yaw = math.radians(index * step_deg)
        _apply_posture(
            model, data, coordinates, arm_spec, user_angles, width_m, base_qpos, cube_position, yaw
        )
        angles += 1
        arm_cube = _contacts_between(model, data, robot, cube_bodies)
        if arm_cube:
            collisions.append({"yaw_deg": round(math.degrees(yaw), 1), "contacts": arm_cube})
        self_hits = _contacts_between(model, data, robot, robot)
        if self_hits:
            collisions.append({"yaw_deg": round(math.degrees(yaw), 1), "self": self_hits})
        cam_id = model.camera("rgbd").id
        _, _, cube_in = _project(
            data.xpos[model.body("target_cube").id],
            np.asarray(data.cam_xpos[cam_id]),
            np.asarray(data.cam_xmat[cam_id]).reshape(3, 3),
            float(model.cam_fovy[cam_id]),
            int(model.vis.global_.offwidth),
            int(model.vis.global_.offheight),
        )
        if cube_in:
            cube_visible += 1
    return {
        "angles_checked": angles,
        "arm_cube_or_self_collisions": collisions,
        "cube_visible_angles": cube_visible,
        "sweep_clean": not collisions,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, default=REPO / "configs/g4a2/postures_v1.yaml")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--top-k", type=int, default=8)
    args = parser.parse_args()

    args.output.mkdir(parents=True, exist_ok=True)
    params = _load_params()
    from mobilecar.backends.modelgen.joint_coordinates import ArmCoordinates

    model_path = (REPO / "assets/landerpi/model/robot.xml").resolve()
    arm_spec = json.loads((model_path.parent / "arm_spec.json").read_text(encoding="utf-8"))
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    coordinates = ArmCoordinates(params)
    camera_cfg = params["simulation"]["camera"]
    base_qpos = [0.1473, 0.0, 0.0556, 1.0, 0.0, 0.0, 0.0]
    cube_position = [0.4, 0.0, 0.112]
    renderer = mujoco.Renderer(model, int(camera_cfg["height"]), int(camera_cfg["width"]))
    robot = _robot_body_ids(model)
    cube_bodies = {model.body("target_cube").id}

    profile = yaml.safe_load(args.profile.read_text(encoding="utf-8"))
    if profile.get("interface_profile") != "g4a2-postures-1.0":
        raise SystemExit("posture profile interface_profile mismatch")
    postures = profile["postures"]

    report: dict = {
        "test": "g4a2r_posture_validation_v1",
        "execution_location": "server_only",
        "model_sha256": hashlib.sha256(model_path.read_bytes()).hexdigest(),
        "profile": str(args.profile),
        "stage1": {},
        "stage2": {},
        "started_wall_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }

    # Stage 1a: coarse grid without j1 (fold/extend + roll combos). j1 is solved
    # by an aiming search in stage 1b because the camera aim is sensitive: the
    # link4 camera looks along the arm, so reaching the work area (extended
    # families) and folding low (folded families) both matter.
    folds = []
    for j2 in (-0.8, -0.4, 0.0, 0.4, 0.8):
        for j3 in (-0.8, -0.4, 0.0, 0.4, 0.8):
            for j4 in (0.0, -0.4, -0.8, -1.2):
                for j5 in (0.0, 0.4):
                    folds.append([0.0, j2, j3, j4, j5])
    j1_limit = float(
        model.jnt_range[model.joint("joint1").id][1]
    )

    def evaluate(user_angles: list[float], width_m: float) -> dict:
        _apply_posture(model, data, coordinates, arm_spec, user_angles, width_m, base_qpos, cube_position)
        self_hits = _contacts_between(model, data, robot, robot)
        arm_cube = _contacts_between(model, data, robot, cube_bodies)
        metrics = _camera_metrics(model, data, renderer, camera_cfg)
        model_angles = coordinates.to_model(np.asarray(user_angles, dtype=float))
        limits = model.jnt_range[[model.joint(n).id for n in ARM_JOINTS]]
        within = bool(np.all(model_angles >= limits[:, 0]) and np.all(model_angles <= limits[:, 1]))
        return {
            "user_angles": list(user_angles),
            "gripper_width_m": width_m,
            "within_limits": within,
            "self_collision": self_hits,
            "arm_cube_contact": arm_cube,
            "camera": metrics,
            "stage1_ok": (
                within
                and not self_hits
                and not arm_cube
                and metrics["depth_valid_fraction"] >= 0.5
                and -50.0 <= metrics["optical_axis_pitch_deg"] <= 5.0
            ),
        }

    def cube_center_offset(item: dict) -> float:
        """Distance of the cube from the frame centre in normalized units."""
        pixel = item["camera"]["cube"]["pixel"]
        return math.hypot(pixel[0] - 320.0, pixel[1] - 240.0) / 320.0

    stage1: list[dict] = []
    fold_diagnostics: list[dict] = []
    for fold in folds:
        base_item = evaluate(fold, 0.03)
        reasons = []
        if not base_item["within_limits"]:
            reasons.append("limits")
        if base_item["self_collision"]:
            reasons.append("self_collision")
        if base_item["arm_cube_contact"]:
            reasons.append("arm_cube")
        camera = base_item["camera"]
        if camera["depth_valid_fraction"] < 0.5:
            reasons.append("valid_frac")
        if not -50.0 <= camera["optical_axis_pitch_deg"] <= 5.0:
            reasons.append("pitch")
        fold_diagnostics.append(
            {
                "fold": fold,
                "reasons": reasons,
                "camera_height_m": camera["camera_height_m"],
                "pitch_deg": camera["optical_axis_pitch_deg"],
                "valid_fraction": camera["depth_valid_fraction"],
                "cube_angular_offset_deg": camera["cube_angular_offset_deg"],
                "tcp_angular_offset_deg": camera["tcp_angular_offset_deg"],
            }
        )
        if not base_item["stage1_ok"]:
            continue
        # Stage 1b: fine j1 sweep aiming the cube at the frame centre.
        best = None
        steps = int(round(2 * j1_limit / 0.1)) + 1
        for index in range(steps):
            j1 = -j1_limit + index * 0.1
            candidate = evaluate([j1] + fold[1:], 0.03)
            if not candidate["stage1_ok"]:
                continue
            score = cube_center_offset(candidate)
            if best is None or score < best[0]:
                best = (score, candidate)
        if best is not None:
            best[1]["cube_center_offset"] = best[0]
            stage1.append(best[1])

    report["stage1"]["evaluations"] = stage1
    report["stage1"]["ok_count"] = len(stage1)
    report["stage1"]["fold_diagnostics"] = fold_diagnostics

    # reference: current G4A-1 initial posture camera height
    reference = evaluate([0.0, 0.0, 0.0, 0.0, 0.0], 0.025)
    report["reference_initial"] = reference

    def select(name: str) -> dict | None:
        if name == "surround_view":
            # Prefer candidates whose start view already contains the work
            # area; otherwise take the lowest camera that still sees a valid
            # scene (the 360 deg sweep covers the rest anyway).
            pool = [
                item
                for item in stage1
                if item["stage1_ok"]
                and item["camera"]["optical_axis_pitch_deg"] <= 0.0
            ]
            pool.sort(
                key=lambda item: (
                    0 if item["camera"]["cube"]["in_frame"] else 1,
                    item["camera"]["camera_height_m"],
                )
            )
        else:
            # Nearfield observation: the work area (cube) and the gripper must
            # both be visible; prefer the lowest camera that achieves this.
            pool = [
                item
                for item in stage1
                if item["stage1_ok"]
                and item["camera"]["cube"]["in_frame"]
                and item["camera"]["gripper"]["in_frame"]
                and item["camera"]["camera_height_m"]
                <= reference["camera"]["camera_height_m"]
            ]
            pool.sort(
                key=lambda item: (
                    item.get("cube_center_offset", 99.0),
                    -item["camera"]["depth_valid_fraction"],
                    item["camera"]["camera_height_m"],
                )
            )
        return pool[0] if pool else None

    frozen: dict[str, dict] = {}
    for name, posture in postures.items():
        if posture.get("frozen"):
            frozen[name] = {
                "arm_user_joint_positions_rad": posture["arm_user_joint_positions_rad"],
                "gripper_width_m": posture["gripper_width_m"],
                "selection": "unchanged_frozen",
            }
            continue
        chosen = select(name)
        if chosen is None:
            report["stage2"][name] = {"error": "no candidate passed stage 1"}
            continue
        settle = _settle_check(
            model,
            data,
            coordinates,
            arm_spec,
            chosen["user_angles"],
            chosen["gripper_width_m"],
            base_qpos,
            cube_position,
        )
        sweep = _sweep_check(
            model,
            data,
            coordinates,
            arm_spec,
            chosen["user_angles"],
            chosen["gripper_width_m"],
            base_qpos,
            cube_position,
        )
        entry = {
            "candidate": chosen,
            "stability": settle,
            "sweep": sweep,
            "validated": bool(settle["stable"] and sweep["sweep_clean"]),
        }
        report["stage2"][name] = entry
        if entry["validated"]:
            frozen[name] = {
                "arm_user_joint_positions_rad": chosen["user_angles"],
                "gripper_width_m": chosen["gripper_width_m"],
                "selection": {
                    "camera_height_m": chosen["camera"]["camera_height_m"],
                    "optical_axis_pitch_deg": chosen["camera"]["optical_axis_pitch_deg"],
                    "depth_valid_fraction": chosen["camera"]["depth_valid_fraction"],
                    "cube_in_frame": chosen["camera"]["cube"]["in_frame"],
                    "tcp_in_frame": chosen["camera"]["tcp"]["in_frame"],
                    "settle": settle,
                    "sweep": sweep,
                },
            }

    report["frozen_postures"] = frozen
    report["all_validated"] = all(
        name in frozen for name, posture in postures.items() if not posture.get("frozen")
    )
    report["finished_wall_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    (args.output / "posture_validation.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )

    if report["all_validated"]:
        # Freeze validated angles back into the profile.
        lines = [
            "interface_profile: g4a2-postures-1.0",
            "status: frozen_by_server_validation",
            "frozen_by: tools/g4a2r/validate_postures.py",
            "frozen_wall_utc: " + report["finished_wall_utc"],
            "model_sha256: " + report["model_sha256"],
            "purpose: >-",
            "  G4A-2R 统一姿态配置入口。所有命名姿态（含 episode 初始姿态）都以本文件为准；",
            "  arm_spec.initial_positions 仅作为无本文件时的兼容回退。角度为用户坐标绝对角",
            "  （rad，与六维动作合同一致），夹爪为米制开口。相机保持 link4 挂载。",
            "postures:",
        ]
        for name, posture in postures.items():
            values = (
                frozen[name]["arm_user_joint_positions_rad"]
                if name in frozen
                else posture["arm_user_joint_positions_rad"]
            )
            width = (
                frozen[name]["gripper_width_m"]
                if name in frozen
                else posture["gripper_width_m"]
            )
            lines.append(f"  {name}:")
            lines.append(f"    purpose: {posture['purpose']}")
            lines.append("    frozen: true")
            lines.append(
                "    arm_user_joint_positions_rad: ["
                + ", ".join(f"{value:.6f}" for value in values)
                + "]"
            )
            lines.append(f"    gripper_width_m: {width}")
            if name in frozen and isinstance(frozen[name].get("selection"), dict):
                selection = frozen[name]["selection"]
                lines.append(
                    "    validation: "
                    + json.dumps(
                        {
                            "camera_height_m": round(selection["camera_height_m"], 4),
                            "optical_axis_pitch_deg": round(
                                selection["optical_axis_pitch_deg"], 2
                            ),
                            "depth_valid_fraction": round(
                                selection["depth_valid_fraction"], 3
                            ),
                            "cube_in_frame": selection["cube_in_frame"],
                            "tcp_in_frame": selection["tcp_in_frame"],
                            "stable": selection["settle"]["stable"],
                            "sweep_clean": selection["sweep"]["sweep_clean"],
                        }
                    )
                )
        args.profile.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print(
        json.dumps(
            {
                "all_validated": report["all_validated"],
                "stage1_ok": report["stage1"]["ok_count"],
                "frozen": {
                    name: {
                        "angles": values["arm_user_joint_positions_rad"],
                        "width": values["gripper_width_m"],
                    }
                    for name, values in frozen.items()
                },
            },
            ensure_ascii=False,
        )
    )
    return 0 if report["all_validated"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
