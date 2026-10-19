"""Synchronized, bounded quintic joint paths and Cartesian IK continuation."""

from dataclasses import dataclass

import mujoco
import numpy as np

from .ik import solve_pose, trial_data


@dataclass
class JointTrajectory:
    """Each segment follows one joint-space line, with zero endpoint velocity."""

    waypoints: np.ndarray
    durations_s: np.ndarray

    def __post_init__(self):
        self.waypoints = np.array(self.waypoints, dtype=float, copy=True)
        self.durations_s = np.array(self.durations_s, dtype=float, copy=True)
        if (self.waypoints.ndim != 2 or self.waypoints.shape[1] != 5
                or len(self.waypoints) < 2
                or self.durations_s.shape != (len(self.waypoints)-1,)
                or not np.isfinite(self.waypoints).all()
                or not np.isfinite(self.durations_s).all()
                or np.any(self.durations_s <= 0)):
            raise ValueError("Trajectory needs finite five-joint waypoints and positive durations")
        self.waypoints.flags.writeable = self.durations_s.flags.writeable = False

    @property
    def duration_s(self):
        return float(self.durations_s.sum())

    @property
    def peak_velocity(self):
        return np.max(1.875*np.abs(np.diff(self.waypoints, axis=0))/self.durations_s[:, None], axis=0)

    @property
    def peak_acceleration(self):
        return np.max((10/np.sqrt(3))*np.abs(np.diff(self.waypoints, axis=0))
                      /self.durations_s[:, None]**2, axis=0)

    def sample(self, time_s):
        if not np.isfinite(time_s):
            raise ValueError("Trajectory time must be finite")
        if time_s <= 0:
            return self.waypoints[0].copy(), np.zeros(5)
        if time_s >= self.duration_s:
            return self.waypoints[-1].copy(), np.zeros(5)
        ends = np.cumsum(self.durations_s)
        index = int(np.searchsorted(ends, time_s, side="right"))
        start = 0. if index == 0 else ends[index-1]
        u = (time_s-start)/self.durations_s[index]
        delta = self.waypoints[index+1]-self.waypoints[index]
        blend = 10*u**3-15*u**4+6*u**5
        derivative = 30*u**2-60*u**3+30*u**4
        return (self.waypoints[index]+blend*delta,
                derivative*delta/self.durations_s[index])

    def collision_samples(self, max_joint_step_rad=.025):
        if not np.isfinite(max_joint_step_rad) or max_joint_step_rad <= 0:
            raise ValueError("Collision sample spacing must be positive")
        points = [self.waypoints[0].copy()]
        for a, b in zip(self.waypoints[:-1], self.waypoints[1:]):
            count = max(1, int(np.ceil(np.max(np.abs(b-a))/max_joint_step_rad)))
            points.extend(a+t*(b-a) for t in np.linspace(0, 1, count+1)[1:])
        return np.asarray(points)

    def as_dict(self):
        return {"waypoints_rad": self.waypoints.tolist(), "durations_s": self.durations_s.tolist(),
                "duration_s": self.duration_s, "peak_velocity_rad_s": self.peak_velocity.tolist(),
                "peak_acceleration_rad_s2": self.peak_acceleration.tolist()}


def time_parameterize(waypoints, velocity_limits, acceleration_limits, speed_scale=.6,
                      minimum_segment_s=.12):
    points = np.asarray(waypoints, dtype=float)
    velocity, acceleration = np.asarray(velocity_limits), np.asarray(acceleration_limits)
    if (points.ndim != 2 or points.shape[1] != 5 or len(points) < 2
            or velocity.shape != (5,) or acceleration.shape != (5,)
            or not np.isfinite(points).all() or not np.isfinite(velocity).all()
            or not np.isfinite(acceleration).all() or np.any(velocity <= 0)
            or np.any(acceleration <= 0) or not np.isfinite(speed_scale)
            or not 0 < speed_scale <= 1 or not np.isfinite(minimum_segment_s)
            or minimum_segment_s <= 0):
        raise ValueError("Invalid trajectory limits, points, or speed scale")
    delta = np.abs(np.diff(points, axis=0))
    duration = np.maximum(np.max(1.875*delta/(speed_scale*velocity), axis=1),
                          np.max(np.sqrt((10/np.sqrt(3))*delta/(speed_scale*acceleration)), axis=1))
    return JointTrajectory(points, np.maximum(duration, minimum_segment_s))


def cartesian_path(sim, start_angles, end_position, rotation, config, qpos_snapshot=None):
    """Continue IK from the previous waypoint along a TCP line; no live writes."""
    data = trial_data(sim, qpos_snapshot)
    c = sim.arm_controller
    data.qpos[c.qpos[:5]] = start_angles
    mujoco.mj_forward(sim.model, data)
    start_position = data.site_xpos[c.tcp_id].copy()
    start_quat, end_quat = np.empty(4), np.empty(4)
    mujoco.mju_mat2Quat(start_quat, data.site_xmat[c.tcp_id])
    mujoco.mju_mat2Quat(end_quat, np.asarray(rotation).ravel())
    if start_quat @ end_quat < 0:
        end_quat *= -1
    cosine = np.clip(start_quat @ end_quat, -1., 1.)
    theta = np.arccos(cosine)
    end_position = np.asarray(end_position, dtype=float)
    if end_position.shape != (3,) or not np.isfinite(end_position).all():
        raise ValueError("Cartesian endpoint must be a finite three-vector")
    count = max(1, int(np.ceil(np.linalg.norm(end_position-start_position)/config["cartesian_step_m"])))
    if count > config["maximum_cartesian_waypoints"]:
        raise ValueError("Cartesian path exceeds waypoint budget")
    points, errors = [np.asarray(start_angles).copy()], []
    for fraction in np.linspace(0., 1., count+1)[1:]:
        # Blend out tracking error instead of imposing a sudden orientation
        # correction at the first Cartesian waypoint (especially near a limit).
        if theta < 1e-6:
            quat = (1-fraction)*start_quat+fraction*end_quat
            quat /= np.linalg.norm(quat)
        else:
            quat = (np.sin((1-fraction)*theta)*start_quat+np.sin(fraction*theta)*end_quat)/np.sin(theta)
        blended_rotation = np.empty(9)
        mujoco.mju_quat2Mat(blended_rotation, quat)
        result = solve_pose(sim, start_position+fraction*(end_position-start_position), blended_rotation.reshape(3, 3),
                            seed=points[-1], qpos_snapshot=qpos_snapshot)
        if (result.position_error_m > config["ik_position_tolerance_m"]
                or np.degrees(result.axis_error_rad) > config["ik_axis_tolerance_deg"]):
            raise ValueError("Cartesian path has an unreachable waypoint")
        if np.max(np.abs(result.model_angles_rad-points[-1])) > config["maximum_ik_joint_step_rad"]:
            raise ValueError("Cartesian IK changed branch or exceeded joint-step limit")
        points.append(result.model_angles_rad.copy())
        errors.append({"position_error_m": result.position_error_m,
                       "axis_error_deg": float(np.degrees(result.axis_error_rad))})
    return np.asarray(points), errors
