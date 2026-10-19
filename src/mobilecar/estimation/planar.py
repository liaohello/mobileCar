"""Pure wheel/gyro planar odometry with explicit health and covariance."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml

from ..contracts.navigation import (
    EstimatedBaseState,
    NavigationClockV1,
    NavigationHealth,
    NavigationHealthStatus,
    NavigationObservation,
)


@dataclass(frozen=True)
class OdometryStep:
    estimate: EstimatedBaseState | None
    health: NavigationHealth


def _rotated_vector_wxyz(rotation: list[float], values: list[float]) -> np.ndarray:
    w, x, y, z = (float(value) for value in rotation)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("IMU rotation is not a finite quaternion")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    matrix = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )
    return matrix @ np.asarray(values, dtype=float)


def _covariance(diagonal: list[float]) -> list[float]:
    matrix = np.zeros((6, 6), dtype=float)
    matrix[np.diag_indices(6)] = np.asarray(diagonal, dtype=float)
    return matrix.reshape(-1).tolist()


class WheelGyroPlanarOdometry:
    """Interpretable complementary planar fusion, not an EKF."""

    def __init__(
        self,
        profile: Mapping[str, Any],
        *,
        profile_sha256: str,
    ) -> None:
        if profile.get("interface_profile") != "g3a-1.0":
            raise ValueError("G3A navigation profile is required")
        estimate = profile["estimate"]
        if estimate.get("source") != "wheel_gyro_planar_fusion":
            raise ValueError("unsupported odometry source")
        if estimate.get("algorithm") != "complementary_planar":
            raise ValueError("unsupported planar fusion algorithm")
        self.profile_id = str(profile["profile_id"])
        self.profile_version = str(profile["profile_version"])
        self.profile_sha256 = str(profile_sha256)
        self.gain = float(estimate["complementary_gain"])
        if not 0.0 <= self.gain <= 1.0:
            raise ValueError("complementary gain must be in [0,1]")
        self.k = float(
            estimate["wheel_forward_kinematics"][
                "half_wheelbase_plus_half_track_m"
            ]
        )
        self.wheel_names = [
            str(value) for value in profile["sensors"]["wheel"]["wheel_names"]
        ]
        if len(self.wheel_names) != 4:
            raise ValueError("navigation profile must declare four wheel names")
        self.gyro_rotation = [
            float(value)
            for value in estimate["gyro_projection"]["rotation_wxyz"]
        ]
        self.max_measurement_age_s = float(
            estimate["max_measurement_age_s"]
        )
        self.max_scan_age_s = float(
            profile["sensors"]["scan"]["maximum_age_s"]
        )
        covariance = estimate["covariance"]
        self.pose_diagonal = [float(value) for value in covariance["pose_diagonal"]]
        self.twist_diagonal = [float(value) for value in covariance["twist_diagonal"]]
        if len(self.pose_diagonal) != 6 or len(self.twist_diagonal) != 6:
            raise ValueError("covariance diagonals must have six entries")
        self._lock = threading.RLock()
        self._reset_unlocked()

    @property
    def pose_covariance(self) -> list[float]:
        return _covariance(self.pose_diagonal)

    @property
    def twist_covariance(self) -> list[float]:
        return _covariance(self.twist_diagonal)

    def _reset_unlocked(self) -> None:
        self.x_m = 0.0
        self.y_m = 0.0
        self.yaw_rad = 0.0
        self.vx_body_m_s = 0.0
        self.vy_body_m_s = 0.0
        self.wz_body_rad_s = 0.0
        self.last_sim_time_s: float | None = None
        self.last_capture_monotonic_s: float | None = None
        self.last_scan_monotonic_s: float | None = None
        self.last_scan_control_tick_id: str | None = None
        self.last_scan_valid = False
        self.last_tick_id: str | None = None
        self.run_id: str | None = None
        self.episode_id: str | None = None
        self.backend_generation: int | None = None
        self.navigation_epoch: int | None = None
        self.sequence = 0
        self.observation_count = 0
        self.estimate_count = 0
        self.rejected_count = 0
        self.last_reasons: list[str] = []
        self.created_monotonic_s = time.monotonic()
        self.last_estimate_monotonic_s: float | None = None

    def reset(self) -> None:
        with self._lock:
            self._reset_unlocked()

    def _clock_matches(self, clock: NavigationClockV1) -> bool:
        return (
            clock.run_id == self.run_id
            and clock.episode_id == self.episode_id
            and clock.backend_generation == self.backend_generation
            and clock.navigation_epoch == self.navigation_epoch
        )

    def _initialize(self, clock: NavigationClockV1) -> None:
        self._reset_unlocked()
        self.run_id = clock.run_id
        self.episode_id = clock.episode_id
        self.backend_generation = clock.backend_generation
        self.navigation_epoch = clock.navigation_epoch

    def _health(
        self,
        *,
        clock: NavigationClockV1,
        observation: NavigationObservation,
        wheel_age_s: float | None,
        imu_age_s: float | None,
        scan_age_s: float | None,
        status: NavigationHealthStatus,
        reasons: list[str],
        stop_required: bool,
    ) -> NavigationHealth:
        self.sequence += 1
        return NavigationHealth(
            profile_id=self.profile_id,
            profile_version=self.profile_version,
            profile_sha256=self.profile_sha256,
            sequence=self.sequence,
            clock=clock,
            status=status,
            wheels_available=bool(observation.wheels.valid),
            imu_gyro_available=bool(observation.imu.gyro_valid),
            imu_acceleration_available=bool(observation.imu.acceleration_valid),
            scan_available=(
                self.last_scan_valid
                and scan_age_s is not None
                and scan_age_s <= self.max_scan_age_s
            ),
            wheel_age_s=wheel_age_s,
            imu_age_s=imu_age_s,
            scan_age_s=scan_age_s,
            rejection_reasons=reasons,
            stop_required=stop_required,
        )

    def observe(
        self,
        observation: NavigationObservation,
        *,
        received_backend_monotonic_s: float | None = None,
    ) -> OdometryStep:
        """Consume one frozen measurement and advance at most once."""

        now = (
            time.monotonic()
            if received_backend_monotonic_s is None
            else float(received_backend_monotonic_s)
        )
        with self._lock:
            clock = observation.clock
            if self.run_id is None or not self._clock_matches(clock):
                self._initialize(clock)
            self.observation_count += 1
            wheel_age_s = max(
                0.0, now - float(observation.wheels.clock.capture_backend_monotonic_s)
            )
            imu_age_s = max(
                0.0, now - float(observation.imu.clock.capture_backend_monotonic_s)
            )
            if observation.scan is not None:
                self.last_scan_monotonic_s = float(
                    observation.scan.clock.capture_backend_monotonic_s
                )
                self.last_scan_control_tick_id = observation.scan.clock.control_tick_id
                self.last_scan_valid = observation.scan.valid_count > 0
            scan_age_s = (
                None
                if self.last_scan_monotonic_s is None
                else max(0.0, now - self.last_scan_monotonic_s)
            )
            reasons: list[str] = []
            if not observation.wheels.valid:
                reasons.append("wheel_measurement_invalid")
            if observation.wheels.wheel_names != self.wheel_names:
                reasons.append("wheel_name_or_order_mismatch")
            if not observation.imu.gyro_valid or observation.imu.angular_velocity_rad_s is None:
                reasons.append("imu_gyro_invalid_or_missing")
            if self.last_scan_monotonic_s is None:
                reasons.append("scan_not_yet_available")
            elif scan_age_s is not None and scan_age_s > self.max_scan_age_s:
                reasons.append("scan_stale")
                reasons.append(f"scan_age_exceeds_{self.max_scan_age_s:.2f}s")
            elif not self.last_scan_valid:
                reasons.append("scan_invalid_no_valid_beams")
            if wheel_age_s > self.max_measurement_age_s:
                reasons.append("wheel_stale")
                reasons.append(
                    f"wheel_age_exceeds_{self.max_measurement_age_s:.2f}s"
                )
            if imu_age_s > self.max_measurement_age_s:
                reasons.append("imu_stale")
                reasons.append(
                    f"imu_age_exceeds_{self.max_measurement_age_s:.2f}s"
                )
            tick_duplicate = clock.control_tick_id == self.last_tick_id
            if tick_duplicate:
                reasons.append("duplicate_control_tick")
            capture_monotonic = float(clock.capture_backend_monotonic_s)
            monotonic_regression = (
                self.last_capture_monotonic_s is not None
                and not tick_duplicate
                and capture_monotonic <= self.last_capture_monotonic_s
            )
            if monotonic_regression:
                reasons.append("capture_monotonic_regression")
            critical_invalid = (
                not observation.wheels.valid
                or observation.wheels.wheel_names != self.wheel_names
                or not observation.imu.gyro_valid
                or observation.imu.angular_velocity_rad_s is None
                or wheel_age_s > self.max_measurement_age_s
                or imu_age_s > self.max_measurement_age_s
            )
            if critical_invalid:
                self.rejected_count += 1
                self.last_reasons = reasons
                status = NavigationHealthStatus.FAILED
                health = self._health(
                    clock=clock,
                    observation=observation,
                    wheel_age_s=wheel_age_s,
                    imu_age_s=imu_age_s,
                    scan_age_s=scan_age_s,
                    status=status,
                    reasons=reasons,
                    stop_required=True,
                )
                return OdometryStep(estimate=None, health=health)
            if tick_duplicate or monotonic_regression:
                self.rejected_count += 1
                self.last_reasons = reasons
                health = self._health(
                    clock=clock,
                    observation=observation,
                    wheel_age_s=wheel_age_s,
                    imu_age_s=imu_age_s,
                    scan_age_s=scan_age_s,
                    status=NavigationHealthStatus.DEGRADED,
                    reasons=reasons,
                    stop_required=False,
                )
                return OdometryStep(estimate=None, health=health)

            capture_sim = float(clock.capture_sim_time_s)
            if (
                self.last_sim_time_s is not None
                and capture_sim < self.last_sim_time_s
            ):
                # A simulator clock regression is a hard boundary. The backend
                # owns epoch increment; the estimator resets its local state so
                # old motion can never be integrated into the new timeline.
                self.x_m = self.y_m = self.yaw_rad = 0.0
                self.vx_body_m_s = self.vy_body_m_s = self.wz_body_rad_s = 0.0
                self.last_sim_time_s = None
                self.last_capture_monotonic_s = None
                self.last_scan_monotonic_s = None
                self.last_scan_valid = False
                reasons.append("sim_time_regression_reset")

            angles = np.asarray(observation.wheels.wheel_angles_rad, dtype=float)
            velocities = np.asarray(
                observation.wheels.wheel_velocities_rad_s, dtype=float
            )
            radii = np.asarray(observation.wheels.wheel_radius_m, dtype=float)
            signs = np.asarray(observation.wheels.wheel_rotation_sign, dtype=float)
            wheel_linear = velocities * radii * signs
            vx = float(np.sum(wheel_linear) / 4.0)
            vy = float(
                (
                    -wheel_linear[0]
                    + wheel_linear[1]
                    + wheel_linear[2]
                    - wheel_linear[3]
                )
                / 4.0
            )
            wheel_wz = float(
                (
                    -wheel_linear[0]
                    - wheel_linear[1]
                    + wheel_linear[2]
                    + wheel_linear[3]
                )
                / (4.0 * self.k)
            )
            gyro_base = _rotated_vector_wxyz(
                observation.imu.rotation_imu_to_base_wxyz,
                observation.imu.angular_velocity_rad_s,
            )
            gyro_wz = float(gyro_base[2])
            wz = (1.0 - self.gain) * wheel_wz + self.gain * gyro_wz
            if self.last_sim_time_s is None:
                dt = 0.0
            else:
                dt = max(0.0, capture_sim - self.last_sim_time_s)
            yaw_mid = self.yaw_rad + 0.5 * wz * dt
            world_vx = math.cos(yaw_mid) * vx - math.sin(yaw_mid) * vy
            world_vy = math.sin(yaw_mid) * vx + math.cos(yaw_mid) * vy
            self.x_m += world_vx * dt
            self.y_m += world_vy * dt
            self.yaw_rad = math.atan2(
                math.sin(self.yaw_rad + wz * dt),
                math.cos(self.yaw_rad + wz * dt),
            )
            self.vx_body_m_s = vx
            self.vy_body_m_s = vy
            self.wz_body_rad_s = wz
            self.last_sim_time_s = capture_sim
            self.last_capture_monotonic_s = float(
                clock.capture_backend_monotonic_s
            )
            self.last_tick_id = clock.control_tick_id
            self.last_reasons = reasons
            if any(reason.startswith("scan_") for reason in reasons):
                status = NavigationHealthStatus.STALE
                stop_required = True
            elif reasons:
                status = NavigationHealthStatus.DEGRADED
                stop_required = False
            else:
                status = NavigationHealthStatus.HEALTHY
                stop_required = False
            health = self._health(
                clock=clock,
                observation=observation,
                wheel_age_s=wheel_age_s,
                imu_age_s=imu_age_s,
                scan_age_s=scan_age_s,
                status=status,
                reasons=reasons,
                stop_required=stop_required,
            )
            self.estimate_count += 1
            self.last_estimate_monotonic_s = now
            estimate = EstimatedBaseState(
                profile_id=self.profile_id,
                profile_version=self.profile_version,
                profile_sha256=self.profile_sha256,
                sequence=self.estimate_count,
                clock=clock,
                x_m=self.x_m,
                y_m=self.y_m,
                yaw_rad=self.yaw_rad,
                vx_body_m_s=self.vx_body_m_s,
                vy_body_m_s=self.vy_body_m_s,
                wz_body_rad_s=self.wz_body_rad_s,
                pose_covariance=self.pose_covariance,
                twist_covariance=self.twist_covariance,
                wheel_sample_id=observation.wheels.sample_id,
                imu_sample_id=observation.imu.sample_id,
                wheel_age_s=wheel_age_s,
                imu_age_s=imu_age_s,
                health=status,
            )
            return OdometryStep(estimate=estimate, health=health)

    def status(self) -> dict[str, Any]:
        with self._lock:
            elapsed = max(1e-9, time.monotonic() - self.created_monotonic_s)
            return {
                "source": "wheel_gyro_planar_fusion",
                "algorithm": "complementary_planar",
                "profile_id": self.profile_id,
                "profile_version": self.profile_version,
                "profile_sha256": self.profile_sha256,
                "run_id": self.run_id,
                "episode_id": self.episode_id,
                "backend_generation": self.backend_generation,
                "navigation_epoch": self.navigation_epoch,
                "sequence": self.sequence,
                "observation_count": self.observation_count,
                "estimate_count": self.estimate_count,
                "rejected_count": self.rejected_count,
                "average_estimate_rate_hz": self.estimate_count / elapsed,
                "pose": {
                    "x_m": self.x_m,
                    "y_m": self.y_m,
                    "yaw_rad": self.yaw_rad,
                    "vx_body_m_s": self.vx_body_m_s,
                    "vy_body_m_s": self.vy_body_m_s,
                    "wz_body_rad_s": self.wz_body_rad_s,
                },
                "last_sim_time_s": self.last_sim_time_s,
                "last_capture_backend_monotonic_s": self.last_capture_monotonic_s,
                "last_scan_backend_monotonic_s": self.last_scan_monotonic_s,
                "last_scan_control_tick_id": self.last_scan_control_tick_id,
                "last_scan_valid": self.last_scan_valid,
                "last_reasons": list(self.last_reasons),
                "uses_world_truth": False,
            }


def load_planar_odometry(path: str | Path) -> WheelGyroPlanarOdometry:
    profile_path = Path(path).resolve()
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    import hashlib

    digest = hashlib.sha256(profile_path.read_bytes()).hexdigest()
    return WheelGyroPlanarOdometry(profile, profile_sha256=digest)
