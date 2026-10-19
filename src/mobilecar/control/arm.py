"""Finite-torque PD arm and coupled gripper control, without state teleportation."""

import numpy as np

from ..backends.modelgen.model.arm import ARM_JOINTS, GRIPPER_DRIVE
from ..backends.modelgen.joint_coordinates import ArmCoordinates


class ArmController:
    def __init__(self, model, params, spec):
        self.model, self.params, self.spec = model, params, spec
        self.names = ARM_JOINTS + (GRIPPER_DRIVE,)
        self.joints = np.array([model.joint(n).id for n in self.names])
        self.qpos = model.jnt_qposadr[self.joints]
        self.dofs = model.jnt_dofadr[self.joints]
        self.actuators = np.array([model.actuator(n+"_motor" if n in ARM_JOINTS else "gripper_motor").id
                                  for n in self.names])
        self.limits = model.jnt_range[self.joints].copy()
        self.coordinates = ArmCoordinates(params)
        self.user_limits = self.coordinates.user_limits(self.limits[:5])
        cfg, gripper = params["simulation"]["arm_servo"], params["simulation"]["gripper_servo"]
        self.torque_limits = np.r_[cfg["torque_limits_nm"], gripper["torque_limit_nm"]]
        self.velocity_limits = np.r_[cfg["velocity_limits_rad_s"],
                                      gripper["speed_m_s"]/spec["gripper"]["width_per_drive_rad_m"]]
        self.acceleration_limits = np.r_[cfg["acceleration_limits_rad_s2"], 4.]
        self.tcp_id = model.site("grasp_tcp").id
        self.left_id = model.site("left_finger_contact").id
        self.right_id = model.site("right_finger_contact").id

    def reset(self, data):
        self.goal = data.qpos[self.qpos].copy()
        self.target = self.goal.copy()
        self.target_velocity = np.zeros(6)
        self.torque = np.zeros(6)
        self.trajectory = None

    def width_to_angle(self, width):
        cfg = self.spec["gripper"]
        if not np.isfinite(width) or not cfg["min_width_m"]-1e-6 <= width <= cfg["max_width_m"]+1e-6:
            raise ValueError(f"Gripper width must be in [{cfg['min_width_m']:.4f}, {cfg['max_width_m']:.4f}] m")
        return float(np.clip((width - cfg["max_width_m"]) / cfg["width_per_drive_rad_m"],
                             *self.limits[5]))

    def set_goal(self, arm=None, width=None):
        goal = self.goal.copy()
        if arm is not None:
            values = np.asarray(arm, dtype=float)
            if values.shape != (5,) or not np.isfinite(values).all():
                raise ValueError("Arm target must contain five finite angles in radians")
            if np.any(values < self.limits[:5, 0]) or np.any(values > self.limits[:5, 1]):
                raise ValueError("Arm target violates joint limits")
            goal[:5] = values
        if width is not None:
            goal[5] = self.width_to_angle(width)
        if arm is not None or width is not None:
            self.trajectory = None
        self.goal[:] = goal

    def follow_trajectory(self, trajectory, data):
        if (np.any(trajectory.waypoints < self.limits[:5, 0])
                or np.any(trajectory.waypoints > self.limits[:5, 1])
                or np.any(trajectory.peak_velocity > self.velocity_limits[:5]+1e-9)
                or np.any(trajectory.peak_acceleration > self.acceleration_limits[:5]+1e-9)
                or np.max(np.abs(trajectory.waypoints[0]-data.qpos[self.qpos[:5]])) > .06):
            raise ValueError("Trajectory violates limits or does not start at the actual arm pose")
        self.trajectory, self.trajectory_start_s = trajectory, data.time
        self.goal[:5] = trajectory.waypoints[-1]
        self.target[:5] = trajectory.waypoints[0]
        self.target_velocity[:5] = 0.

    def stop_trajectory(self, data):
        """Hold the measured arm and gripper pose with finite servos.

        This is intentionally unconditional: cancelling a trajectory, a joint
        goal, a gripper goal, or a lease must all freeze the current physical
        state.  It never opens a holding gripper or teleports state.
        """

        self.trajectory = None
        self.goal[:5] = self.target[:5] = data.qpos[self.qpos[:5]]
        self.target[5] = data.qpos[self.qpos[5]]
        self.goal[5] = self.target[5]
        self.target_velocity[:] = 0.0

    def set_user_goal(self, arm=None, width=None):
        model_arm = None if arm is None else self.coordinates.to_model(arm)
        self.set_goal(model_arm, width)

    def home(self):
        """Move through finite servos; never reset the base, object or physics state."""
        self.set_user_goal(np.zeros(5), self.spec["gripper"]["zero_width_m"])

    def update(self, data, dt):
        delta = self.goal - self.target
        wanted = np.sign(delta) * np.minimum(self.velocity_limits,
                                            np.sqrt(2*self.acceleration_limits*np.abs(delta)))
        self.target_velocity += np.clip(wanted-self.target_velocity,
                                        -self.acceleration_limits*dt, self.acceleration_limits*dt)
        step = self.target_velocity*dt
        reached = (np.sign(step) == np.sign(delta)) & (np.abs(step) >= np.abs(delta))
        self.target += np.where(reached, delta, step)
        self.target = np.clip(self.target, self.limits[:, 0], self.limits[:, 1])
        self.target_velocity[reached] = 0
        if self.trajectory is not None:
            self.target[:5], self.target_velocity[:5] = self.trajectory.sample(data.time-self.trajectory_start_s)
        # Native PD is evaluated at every physics substep with implicit damping.
        # Position actuators take radians in ctrl; wheel motors still take N*m.
        # Under gravity a pure position servo has steady-state sag. Add a
        # bounded model-derived gravity/Coriolis feed-forward through the same
        # position actuator; forcerange remains the authoritative torque limit.
        kp = np.asarray(self.params["simulation"]["arm_servo"]["kp_nm_per_rad"], dtype=float)
        bias = np.asarray(data.qfrc_bias[self.dofs[:5]], dtype=float)
        compensation = np.clip(bias / kp, -0.12, 0.12)
        data.ctrl[self.actuators[:5]] = np.clip(
            self.target[:5] + compensation,
            self.limits[:5, 0],
            self.limits[:5, 1],
        )
        data.ctrl[self.actuators[5]] = self.target[5]

    def state(self, data):
        self.torque = data.actuator_force[self.actuators].copy()
        state = {}
        for i, name in enumerate(self.names):
            state.update({name+"_target_rad": float(self.target[i]),
                          name+"_position_rad": float(data.qpos[self.qpos[i]]),
                          name+"_velocity_rad_s": float(data.qvel[self.dofs[i]]),
                          name+"_torque_nm": float(self.torque[i])})
        user_position = self.coordinates.to_user(data.qpos[self.qpos[:5]])
        user_target = self.coordinates.to_user(self.target[:5])
        for i, name in enumerate(ARM_JOINTS):
            state[name+"_user_position_rad"] = float(user_position[i])
            state[name+"_user_target_rad"] = float(user_target[i])
        state["gripper_user_position_rad"] = float(data.qpos[self.qpos[5]]-self.spec["gripper"]["zero_model_rad"])
        width = np.linalg.norm(data.site_xpos[self.left_id] - data.site_xpos[self.right_id])
        state["gripper_width_m"] = float(width)
        state["gripper_target_width_m"] = float(self.spec["gripper"]["max_width_m"] +
                                                   self.spec["gripper"]["width_per_drive_rad_m"]*self.goal[5])
        for axis, value in zip("xyz", data.site_xpos[self.tcp_id]):
            state["tcp_"+axis+"_m"] = float(value)
        return state
