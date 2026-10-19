"""Limited pose IK for the five-joint arm; all trials use separate MjData."""

from dataclasses import dataclass

import mujoco
import numpy as np


@dataclass
class IKResult:
    model_angles_rad: np.ndarray
    position_error_m: float
    axis_error_rad: float
    iterations: int


def grasp_rotation(pitch_rad, yaw_rad=0.):
    """Guide-rod +Z is approach; +Y is the jaw closing axis."""
    s, c = np.sin(pitch_rad), np.cos(pitch_rad)
    sy, cy = np.sin(yaw_rad), np.cos(yaw_rad)
    yaw = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return yaw @ np.array([[-s, 0, c], [0, 1, 0], [-c, 0, -s]])


def trial_data(sim, qpos_snapshot=None):
    """Make an isolated planning state; hypothetical parking never moves the robot."""
    qpos = sim.data.qpos if qpos_snapshot is None else np.asarray(qpos_snapshot, dtype=float)
    if qpos.shape != (sim.model.nq,) or not np.isfinite(qpos).all():
        raise ValueError("Planning snapshot must contain finite model qpos")
    data = mujoco.MjData(sim.model)
    data.qpos[:] = qpos
    return data


def solve_pose(
    sim,
    position_world_m,
    rotation_world,
    seed=None,
    max_iterations=150,
    qpos_snapshot=None,
    position_tolerance_m=0.0005,
    orientation_weight=0.06,
):
    """Position-first DLS with a separately reported soft orientation cost.

    A five-DOF arm cannot realize arbitrary six-DOF poses.  Stage one optimizes
    TCP position only.  Stage two improves the approach/closing axes while
    refusing candidates that worsen position beyond the requested tolerance.
    Callers must still check the complete interpolated path.
    """
    controller = sim.arm_controller
    position = np.asarray(position_world_m, dtype=float)
    rotation = np.asarray(rotation_world, dtype=float)
    if (position.shape != (3,) or rotation.shape != (3, 3)
            or not np.isfinite(position).all() or not np.isfinite(rotation).all()
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(rotation), 1.)):
        raise ValueError("IK needs a finite position and a proper rotation matrix")
    if not isinstance(max_iterations, int) or max_iterations <= 0:
        raise ValueError("IK iteration budget must be a positive integer")
    if not np.isfinite(position_tolerance_m) or position_tolerance_m <= 0:
        raise ValueError("Position tolerance must be positive")
    if not np.isfinite(orientation_weight) or orientation_weight < 0:
        raise ValueError("Orientation weight must be nonnegative")
    data = trial_data(sim, qpos_snapshot)
    q = data.qpos[controller.qpos[:5]].copy() if seed is None else np.asarray(seed, dtype=float).copy()
    if q.shape != (5,) or not np.isfinite(q).all():
        raise ValueError("IK seed must contain five finite angles")
    limits = controller.limits[:5]
    q = np.clip(q, limits[:, 0], limits[:, 1])
    jp, jr = np.zeros((3, sim.model.nv)), np.zeros((3, sim.model.nv))

    def evaluate_position(angles):
        data.qpos[controller.qpos[:5]] = angles
        mujoco.mj_forward(sim.model, data)
        current = data.site_xmat[controller.tcp_id].reshape(3, 3)
        residual = position - data.site_xpos[controller.tcp_id]
        return residual, current.copy()

    def evaluate_full(angles):
        position_error, current = evaluate_position(angles)
        orientation_error = orientation_weight * (
            rotation[:, 1:] - current[:, 1:]
        ).T.reshape(6)
        return np.r_[position_error, orientation_error], current

    def solve_stage(angles, budget, full):
        residual, current = evaluate_full(angles) if full else evaluate_position(angles)
        if full and np.linalg.norm(residual[:3]) > position_tolerance_m + 1e-9:
            return angles, residual, current, 0
        used = 0
        for _ in range(budget):
            used += 1
            mujoco.mj_jacSite(sim.model, data, jp, jr, controller.tcp_id)
            position_jacobian = jp[:, controller.dofs[:5]]
            if full:
                angular = jr[:, controller.dofs[:5]]
                axes_jacobian = [
                    np.cross(angular.T, current[:, axis]).T for axis in (1, 2)
                ]
                jacobian = np.vstack(
                    [
                        position_jacobian,
                        *[orientation_weight * item for item in axes_jacobian],
                    ]
                )
            else:
                jacobian = position_jacobian
            delta = np.linalg.solve(
                jacobian.T @ jacobian + 1e-6 * np.eye(5),
                jacobian.T @ residual,
            )
            delta *= min(1.0, 0.25 / max(1e-12, float(np.linalg.norm(delta))))
            improved = False
            for scale in (1.0, 0.5, 0.25, 0.1, 0.01):
                candidate = np.clip(angles + scale * delta, limits[:, 0], limits[:, 1])
                if full:
                    new_residual, new_current = evaluate_full(candidate)
                    position_ok = (
                        np.linalg.norm(new_residual[:3]) <= position_tolerance_m + 1e-9
                    )
                    better = (
                        position_ok
                        and np.linalg.norm(new_residual)
                        < np.linalg.norm(residual) - 1e-12
                    )
                else:
                    new_residual, new_current = evaluate_position(candidate)
                    better = (
                        np.linalg.norm(new_residual)
                        < np.linalg.norm(residual) - 1e-12
                    )
                if better:
                    angles = candidate
                    residual = new_residual
                    current = new_current
                    improved = True
                    break
            if not improved:
                break
        return angles, residual, current, used

    position_budget = max(1, min(max_iterations, max(30, max_iterations // 2)))
    q, position_residual, current, position_used = solve_stage(
        q, position_budget, full=False
    )
    used = position_used
    # Orientation is softened but cannot consume position margin.
    position_error = float(np.linalg.norm(position_residual))
    soft_budget = max(0, max_iterations - position_used)
    if position_error <= position_tolerance_m + 1e-9 and soft_budget:
        q, _, _, soft_used = solve_stage(q, soft_budget, full=True)
        used += soft_used
    # Backtracking evaluates rejected candidates; synchronize MjData with the
    # accepted q before reporting either residual.
    _, current = evaluate_position(q)
    position_error = float(
        np.linalg.norm(position - data.site_xpos[controller.tcp_id])
    )
    axis_error = max(
        np.arccos(np.clip(current[:, i] @ rotation[:, i], -1.0, 1.0))
        for i in (1, 2)
    )
    return IKResult(
        q,
        position_error,
        float(axis_error),
        max(1, used),
    )


def path_collisions(sim, start, goal, opening_m, samples=60, penetration_m=0.0005):
    """Check a joint-interpolated path against current scene and robot geoms."""
    c = sim.arm_controller
    data = mujoco.MjData(sim.model)
    data.qpos[:] = sim.data.qpos
    driver = c.width_to_angle(opening_m)
    data.qpos[c.qpos[5]] = driver
    for name, relation in sim.arm_spec["mimic"].items():
        address = sim.model.jnt_qposadr[sim.model.joint(name).id]
        data.qpos[address] = relation["multiplier"]*driver+relation["offset"]
    collisions = set()
    for t in np.linspace(0., 1., samples):
        data.qpos[c.qpos[:5]] = (1-t)*np.asarray(start)+t*np.asarray(goal)
        mujoco.mj_forward(sim.model, data)
        for contact in data.contact:
            names = (sim.model.geom(contact.geom1).name, sim.model.geom(contact.geom2).name)
            arm_contact = any(n.startswith(("link", "gripper", "guide_rod", "left_jaw", "right_jaw")) for n in names)
            if arm_contact and contact.dist < -penetration_m:
                collisions.add(tuple(sorted(names)))
    return sorted(collisions)


def sampled_path_collisions(sim, angles, opening_m, qpos_snapshot=None,
                            allow_finger_object=False, carried_object=False,
                            initial_support_tolerance_m=0.):
    """Check the reference path, including the carried cube in a planning copy.

    A carried-object transform predicts geometry only. No equality is added and
    execution still relies entirely on physical finger contact and friction.
    """
    c = sim.arm_controller
    if not np.isfinite(initial_support_tolerance_m) or not 0 <= initial_support_tolerance_m <= .003:
        raise ValueError("Initial support tolerance must be in [0, 3] mm")
    points = np.asarray(angles, dtype=float)
    if points.ndim != 2 or points.shape[1] != 5 or not len(points) or not np.isfinite(points).all():
        raise ValueError("Collision path must contain finite five-joint samples")
    data = trial_data(sim, qpos_snapshot)
    driver = c.width_to_angle(opening_m)
    data.qpos[c.qpos[5]] = driver
    for name, relation in sim.arm_spec["mimic"].items():
        address = sim.model.jnt_qposadr[sim.model.joint(name).id]
        data.qpos[address] = relation["multiplier"]*driver+relation["offset"]
    object_id = sim.model.body("target_cube").id
    object_joint = sim.model.body_jntadr[object_id]
    object_address = sim.model.jnt_qposadr[object_joint]
    if carried_object and sim.model.jnt_type[object_joint] != mujoco.mjtJoint.mjJNT_FREE:
        raise ValueError("Carried-object check requires a free cube")
    relative_position = relative_rotation = None
    if carried_object:
        data.qpos[c.qpos[:5]] = points[0]
        mujoco.mj_forward(sim.model, data)
        tcp_rotation = data.site_xmat[c.tcp_id].reshape(3, 3)
        relative_position = tcp_rotation.T @ (data.xpos[object_id]-data.site_xpos[c.tcp_id])
        relative_rotation = tcp_rotation.T @ data.xmat[object_id].reshape(3, 3)
        initial_object_z = float(data.xpos[object_id, 2])
        previous_object_z, releasing_support = initial_object_z, True
    collisions = set()
    for point_index, q in enumerate(points):
        data.qpos[c.qpos[:5]] = q
        mujoco.mj_forward(sim.model, data)
        if carried_object:
            rotation = data.site_xmat[c.tcp_id].reshape(3, 3)
            data.qpos[object_address:object_address+3] = data.site_xpos[c.tcp_id]+rotation @ relative_position
            quat = np.empty(4)
            mujoco.mju_mat2Quat(quat, (rotation @ relative_rotation).ravel())
            data.qpos[object_address+3:object_address+7] = quat
            mujoco.mj_forward(sim.model, data)
            current_z = float(data.xpos[object_id, 2])
            if current_z < previous_object_z-1e-9 or current_z > initial_object_z+initial_support_tolerance_m:
                releasing_support = False
            previous_object_z = current_z
        for contact in data.contact:
            names = tuple(sim.model.geom(i).name for i in (contact.geom1, contact.geom2))
            fingers = any(n.startswith(("left_jaw_link_", "right_jaw_link_")) for n in names)
            object_contact = "target_cube_contact" in names
            arm_contact = any(n.startswith(("link", "gripper", "guide_rod", "left_jaw", "right_jaw")) for n in names)
            if object_contact and fingers and allow_finger_object:
                continue
            # Only the first, monotonically upward release from the known
            # support may tolerate a bounded estimated-proxy height error.
            # Downward paths, floor/other obstacles and deeper overlaps fail.
            if (carried_object and object_contact and "pick_platform" in names
                    and initial_support_tolerance_m > 0
                    and releasing_support and point_index < len(points)-1
                    and contact.dist >= -initial_support_tolerance_m
                    and initial_object_z-1e-9 <= data.xpos[object_id, 2] <= initial_object_z+initial_support_tolerance_m):
                continue
            if contact.dist < -.0005 and (arm_contact or (carried_object and object_contact)):
                collisions.add(tuple(sorted(names)))
    return sorted(collisions)
