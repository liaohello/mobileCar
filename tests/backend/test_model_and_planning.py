from pathlib import Path

import mujoco
import numpy as np

from mobilecar.backends.mujoco.backend import MujocoBackend
from mobilecar.config import load_config


def _backend(tmp_path: Path) -> MujocoBackend:
    config = load_config()
    backend_config = dict(config.backend)
    backend_config["render_rgbd"] = False
    backend_config["realtime"] = False
    return MujocoBackend(
        backend_config,
        config.safety,
        config.data["acceptance"],
        tmp_path,
    )


def test_model_identity_and_camera_branch(tmp_path):
    backend = _backend(tmp_path)
    try:
        assert backend.params["simulation"]["fixed_arm"] is False
        assert backend.model.joint("joint5").id >= 0
        assert backend.model.actuator("gripper_motor").id >= 0
        assert backend.model.body(backend.model.cam_bodyid[backend.camera_id]).name == "depth_cam_link"
        branch = backend.camera_body_id
        while branch != backend.link4_id:
            branch = int(backend.model.body_parentid[branch])
        assert branch == backend.link4_id
        capabilities = backend.capabilities()
        scene = backend.scene_manifest()
        assert capabilities.model_sha256 == scene.model_sha256
        assert capabilities.camera_parent_body == "link4"
        assert capabilities.tcp_parent_body == "guide_rod_link"
    finally:
        backend.close()


def test_q5_and_gripper_do_not_move_camera_but_q5_moves_tcp(tmp_path):
    backend = _backend(tmp_path)
    try:
        with backend.lock:
            camera_id = backend.camera_id
            tcp_id = backend.tcp_id
            q5 = int(backend.arm.qpos[4])
            drive = int(backend.arm.qpos[5])
            q5_initial = float(backend.data.qpos[q5])
            drive_initial = float(backend.data.qpos[drive])
            mujoco.mj_forward(backend.model, backend.data)
            camera_position = backend.data.cam_xpos[camera_id].copy()
            camera_rotation = backend.data.cam_xmat[camera_id].copy()
            tcp_rotation = backend.data.site_xmat[tcp_id].copy()

            backend.data.qpos[q5] = q5_initial + 0.45
            backend.data.qpos[drive] = drive_initial - 0.20
            mujoco.mj_forward(backend.model, backend.data)
            assert np.allclose(backend.data.cam_xpos[camera_id], camera_position, atol=1e-10)
            assert np.allclose(backend.data.cam_xmat[camera_id], camera_rotation, atol=1e-10)
            assert not np.allclose(backend.data.site_xmat[tcp_id], tcp_rotation, atol=1e-4)
    finally:
        backend.close()


def test_image_transforms_use_nonzero_base_pose_and_q5_invariants(tmp_path):
    backend = _backend(tmp_path)
    try:
        with backend.lock:
            base_joint = backend.model.joint("base_free").id
            base_qpos = int(backend.model.jnt_qposadr[base_joint])
            yaw = 0.37
            backend.data.qpos[base_qpos : base_qpos + 3] = [0.23, -0.17, 0.02]
            backend.data.qpos[base_qpos + 3 : base_qpos + 7] = [
                np.cos(yaw / 2),
                0.0,
                0.0,
                np.sin(yaw / 2),
            ]
            backend.data.qvel[:] = 0.0
            mujoco.mj_forward(backend.model, backend.data)
            initial = backend._image_transforms(backend.data)
            world_base = np.eye(4)
            world_base[:3, :3] = backend.data.xmat[backend.base_id].reshape(3, 3)
            world_base[:3, 3] = backend.data.xpos[backend.base_id]
            world_link4 = np.eye(4)
            world_link4[:3, :3] = backend.data.xmat[backend.link4_id].reshape(3, 3)
            world_link4[:3, 3] = backend.data.xpos[backend.link4_id]
            world_camera = np.eye(4)
            world_camera[:3, :3] = backend.data.cam_xmat[backend.camera_id].reshape(3, 3)
            world_camera[:3, 3] = backend.data.cam_xpos[backend.camera_id]
            assert np.allclose(
                np.asarray(initial["T_base_link4"]),
                np.linalg.inv(world_base) @ world_link4,
            )
            assert np.allclose(
                np.asarray(initial["T_base_camera_render"]),
                np.linalg.inv(world_base) @ world_camera,
            )
            camera_before = np.asarray(initial["T_base_camera_render"])
            tcp_camera_before = np.asarray(initial["T_TCP_camera"])

            backend.data.qpos[backend.arm.qpos[4]] += 0.22
            mujoco.mj_forward(backend.model, backend.data)
            q5_changed = backend._image_transforms(backend.data)
            assert np.allclose(
                np.asarray(q5_changed["T_base_camera_render"]),
                camera_before,
                atol=1e-10,
            )
            assert not np.allclose(
                np.asarray(q5_changed["T_TCP_camera"]),
                tcp_camera_before,
                atol=1e-4,
            )
            tcp_camera_q5 = np.asarray(q5_changed["T_TCP_camera"])

            backend.data.qpos[backend.arm.qpos[5]] -= 0.15
            mujoco.mj_forward(backend.model, backend.data)
            opened = backend._image_transforms(backend.data)
            assert np.allclose(
                np.asarray(opened["T_base_camera_render"]),
                camera_before,
                atol=1e-10,
            )
            assert np.allclose(
                np.asarray(opened["T_TCP_camera"]),
                tcp_camera_q5,
                atol=1e-10,
            )
    finally:
        backend.close()


def test_planning_copy_does_not_modify_running_state(tmp_path):
    backend = _backend(tmp_path)
    try:
        from mobilecar.planning.ik import solve_pose

        with backend.lock:
            qpos = backend.data.qpos.copy()
            qvel = backend.data.qvel.copy()
            tcp_rotation = backend.data.site_xmat[backend.tcp_id].reshape(3, 3).copy()
            target = backend.data.site_xpos[backend.tcp_id] + np.array([0.025, 0.0, 0.0])
            result = solve_pose(
                backend,
                target,
                tcp_rotation,
                qpos_snapshot=qpos,
                max_iterations=120,
            )
            assert result.position_error_m <= 0.005
            assert np.array_equal(backend.data.qpos, qpos)
            assert np.array_equal(backend.data.qvel, qvel)
    finally:
        backend.close()


def test_position_priority_preserves_position_for_soft_orientation(tmp_path):
    backend = _backend(tmp_path)
    try:
        from mobilecar.planning.ik import solve_pose

        with backend.lock:
            target = backend.data.site_xpos[backend.tcp_id].copy()
            current = backend.data.site_xmat[backend.tcp_id].reshape(3, 3).copy()
            c, s = np.cos(1.2), np.sin(1.2)
            turned = current @ np.array(
                [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]
            )
            result = solve_pose(
                backend,
                target,
                turned,
                qpos_snapshot=backend.data.qpos.copy(),
                max_iterations=180,
                position_tolerance_m=0.002,
            )
            assert result.position_error_m <= 0.002
            # There is no promise of six-dimensional reachability; the
            # orientation cost is explicitly soft and reported.
            assert result.axis_error_rad >= 0.0
    finally:
        backend.close()
