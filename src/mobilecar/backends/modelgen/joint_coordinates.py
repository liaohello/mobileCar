"""User photo-zero coordinates, independent of URDF and hardware servo commands."""

import numpy as np


class ArmCoordinates:
    def __init__(self, params):
        cfg = params["calibration"]
        self.zero = np.asarray(cfg["arm_zero_model_rad"], dtype=float)
        self.direction = np.asarray(cfg["arm_user_direction"], dtype=float)

    @staticmethod
    def _angles(values):
        values = np.asarray(values, dtype=float)
        if values.shape != (5,) or not np.isfinite(values).all():
            raise ValueError("Arm angles must contain five finite values")
        return values

    def to_model(self, user_angles):
        return self.zero+self.direction*self._angles(user_angles)

    def to_user(self, model_angles):
        return (self._angles(model_angles)-self.zero)*self.direction

    def user_limits(self, model_limits):
        values = (np.asarray(model_limits)-self.zero[:, None])*self.direction[:, None]
        return np.sort(values, axis=1)
