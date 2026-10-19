"""Body velocity -> wheel velocities -> bounded wheel-axis torque, all in SI."""

from collections import deque

import mujoco
import numpy as np

from ..backends.modelgen.build_model import WHEEL_NAMES


def inverse_kinematics(command, params):
    vx, vy, wz = np.asarray(command, dtype=float)
    k = params["derived"]["mecanum_half_wheelbase_plus_half_track_m"]
    return np.array([vx-vy-k*wz, vx+vy-k*wz, vx+vy+k*wz, vx-vy+k*wz]) / params["derived"]["effective_wheel_radii_m"]


def forward_kinematics(wheel_rad_s, params):
    v = np.asarray(wheel_rad_s) * params["derived"]["effective_wheel_radii_m"]
    k = params["derived"]["mecanum_half_wheelbase_plus_half_track_m"]
    return np.array([v.sum()/4, (-v[0]+v[1]+v[2]-v[3])/4, (-v[0]-v[1]+v[2]+v[3])/(4*k)])


class WheelController:
    def __init__(self, model, params):
        self.params = params
        self.dofs = np.array([model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n+"_joint")] for n in WHEEL_NAMES])
        self.actuators = np.array([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n+"_motor") for n in WHEEL_NAMES])
        self.reset()

    def reset(self):
        self.command = np.zeros(3)
        self.target = np.zeros(4)
        self.torque = np.zeros(4)
        self.integral_torque = np.zeros(4)
        self.unlimited_torque = np.zeros(4)
        self.saturated = np.zeros(4, dtype=bool)
        self._pending = deque()
        self._delayed = np.zeros(3)

    def update(self, data, requested, dt):
        requested = np.asarray(requested, dtype=float).copy()
        if requested.shape != (3,) or not np.isfinite(requested).all():
            raise ValueError("Velocity command must contain finite [vx,vy,wz]")
        limits = self.params["command_limits"]
        norm = np.linalg.norm(requested[:2])
        if norm > limits["translation_norm_m_s"]:
            requested[:2] *= limits["translation_norm_m_s"] / norm
        requested[2] = np.clip(requested[2], -limits["yaw_rate_rad_s"], limits["yaw_rate_rad_s"])
        delay = self.params["simulation"]["wheel_servo"]["command_delay_s"]
        self._pending.append((data.time + delay, requested))
        while self._pending and self._pending[0][0] <= data.time + 1e-9:
            self._delayed = self._pending.popleft()[1]
        delta = self._delayed[:2] - self.command[:2]
        length = np.linalg.norm(delta)
        allowed = limits["translation_acceleration_m_s2"] * dt
        self.command[:2] += delta * min(1, allowed/max(length, 1e-12))
        yaw_step = limits["yaw_acceleration_rad_s2"] * dt
        self.command[2] += np.clip(self._delayed[2]-self.command[2], -yaw_step, yaw_step)
        self.target = inverse_kinematics(self.command, self.params)
        no_load = self.params["derived"]["output_no_load_rad_s_estimate"]
        self.target *= min(1, no_load/max(np.max(np.abs(self.target)), 1e-12))
        speed = data.qvel[self.dofs]
        servo = self.params["simulation"]["wheel_servo"]
        limit = np.asarray(self.params["derived"]["wheel_torque_limits_nm"])
        # Provisional linear motoring envelope. Braking torque remains available.
        positive = limit * np.clip(1-np.maximum(speed, 0)/no_load, 0, 1)
        negative = limit * np.clip(1-np.maximum(-speed, 0)/no_load, 0, 1)
        error = self.target-speed
        proportional = servo["kv_nm_per_rad_s"] * error
        # Integrate torque directly (Nm), with directional anti-windup against
        # the speed-dependent motor envelope. Zero body commands discharge it
        # immediately: cancellation must not leave stored drive torque behind.
        ki = servo.get("ki_nm_per_rad", 0.)
        bound = servo.get("integral_torque_limit_nm", 0.)
        candidate = np.clip(self.integral_torque+ki*error*dt, -bound, bound)
        raw = proportional+candidate
        winding = ((raw > positive) & (error > 0)) | ((raw < -negative) & (error < 0))
        self.integral_torque = np.where(winding, self.integral_torque, candidate)
        if ki == 0 or np.max(np.abs(self._delayed)) < 1e-12:
            self.integral_torque[:] = 0.
        self.unlimited_torque = proportional+self.integral_torque
        self.torque = np.clip(self.unlimited_torque, -negative, positive)
        self.saturated = np.abs(self.unlimited_torque-self.torque) > 1e-12
        data.ctrl[self.actuators] = self.torque
        return self.target.copy()

    def sdk_rps(self):
        signs = self.params["firmware_reference"]["sdk_sign_from_forward_wheel_rotation"]
        return self.target / (2*np.pi) * signs
