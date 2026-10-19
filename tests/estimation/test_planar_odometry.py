import math
from pathlib import Path

import pytest

from mobilecar.contracts.navigation import (
    ImuMeasurement,
    LaserReturnStatus,
    LaserScanObservation,
    NavigationClockV1,
    NavigationHealthStatus,
    NavigationJointMeasurement,
    NavigationObservation,
    WheelMeasurement,
)
from mobilecar.estimation import WheelGyroPlanarOdometry, load_planar_odometry


ROOT = Path(__file__).resolve().parents[2]
PROFILE = ROOT / "configs/profiles/g3a_navigation_v1.yaml"


def _clock(time_s: float, tick: int, *, epoch: int = 0, episode: str = "episode") -> NavigationClockV1:
    return NavigationClockV1(
        run_id="run",
        episode_id=episode,
        backend_generation=1,
        navigation_epoch=epoch,
        control_tick_id=f"tick_{tick:09d}",
        capture_sim_time_s=time_s,
        capture_backend_monotonic_s=100.0 + time_s,
    )


def _scan(clock: NavigationClockV1) -> LaserScanObservation:
    return LaserScanObservation(
        profile_id="g3a-nav-50hz-laser10-v1",
        profile_version="1.0",
        profile_sha256="0" * 64,
        clock=clock,
        scan_id=f"scan_{clock.control_tick_id}",
        angle_min_rad=0.0,
        angle_max_rad=0.3,
        angle_increment_rad=0.1,
        range_min_m=0.1,
        range_max_m=8.0,
        ranges_m=[1.0, None, None, None],
        statuses=[
            LaserReturnStatus.HIT,
            LaserReturnStatus.NO_RETURN,
            LaserReturnStatus.INVALID,
            LaserReturnStatus.SELF_OCCLUDED,
        ],
        beam_count=4,
        valid_count=1,
        status_counts={
            "HIT": 1,
            "NO_RETURN": 1,
            "INVALID": 1,
            "SELF_OCCLUDED": 1,
        },
    )


def _observation(
    *,
    time_s: float,
    tick: int,
    wheel_velocity: list[float],
    gyro_z: float,
    epoch: int = 0,
    episode: str = "episode",
    wheel_valid: bool = True,
    gyro_valid: bool = True,
    with_scan: bool = True,
) -> NavigationObservation:
    clock = _clock(time_s, tick, epoch=epoch, episode=episode)
    return NavigationObservation(
        profile_id="g3a-nav-50hz-laser10-v1",
        profile_version="1.0",
        profile_sha256="0" * 64,
        sequence=tick,
        clock=clock,
        wheels=WheelMeasurement(
            sample_id=f"wheel_{tick}",
            clock=clock,
            wheel_names=[
                "front_left_wheel",
                "rear_left_wheel",
                "front_right_wheel",
                "rear_right_wheel",
            ],
            wheel_angles_rad=[0.0, 0.0, 0.0, 0.0],
            wheel_velocities_rad_s=wheel_velocity,
            wheel_radius_m=[0.0325] * 4,
            wheel_rotation_sign=[1.0] * 4,
            valid=wheel_valid,
            quality="simulated_nominal",
            source="mujoco_wheel_joint_feedback",
        ),
        imu=ImuMeasurement(
            sample_id=f"imu_{tick}",
            clock=clock,
            rotation_imu_to_base_wxyz=[1.0, 0.0, 0.0, 0.0],
            angular_velocity_rad_s=(
                [0.0, 0.0, gyro_z] if gyro_valid else None
            ),
            linear_acceleration_m_s2=[0.0, 0.0, 9.81],
            gyro_valid=gyro_valid,
            acceleration_valid=True,
            quality="simulated_nominal",
            source="mujoco_imu_sensor",
        ),
        joints=NavigationJointMeasurement(
            sample_id=f"joint_{tick}",
            clock=clock,
            joint_names=["joint1", "joint2", "joint3", "joint4", "joint5"],
            model_positions_rad=[0.0, 0.0, 0.0, 0.0, 0.0],
            available=True,
            quality="simulated_nominal",
        ),
        scan=_scan(clock) if with_scan else None,
    )


def _wheel_rad_s(vx: float, vy: float, wz: float, radius: float = 0.0325) -> list[float]:
    k = 0.141
    return [float(value / radius) for value in (
        vx - vy - k * wz,
        vx + vy - k * wz,
        vx + vy + k * wz,
        vx - vy + k * wz,
    )]


def test_forward_strafe_and_bidirectional_yaw():
    estimator = load_planar_odometry(PROFILE)
    estimator.observe(_observation(
        time_s=0.0,
        tick=1,
        wheel_velocity=_wheel_rad_s(0.0, 0.0, 0.0),
        gyro_z=0.0,
    ), received_backend_monotonic_s=100.0)
    for tick in range(2, 12):
        step = estimator.observe(
            _observation(
                time_s=(tick - 1) * 0.02,
                tick=tick,
                wheel_velocity=_wheel_rad_s(0.02, 0.0, 0.0),
                gyro_z=0.0,
            ),
            received_backend_monotonic_s=100.0 + (tick - 1) * 0.02,
        )
    assert step.estimate is not None
    assert step.estimate.x_m == pytest.approx(0.0040, abs=0.0003)
    assert step.estimate.y_m == pytest.approx(0.0, abs=1e-9)

    estimator.reset()
    for tick in range(1, 11):
        step = estimator.observe(
            _observation(
                time_s=tick * 0.02,
                tick=tick,
                wheel_velocity=_wheel_rad_s(0.0, 0.02, 0.0),
                gyro_z=0.0,
            ),
            received_backend_monotonic_s=100.0 + tick * 0.02,
        )
    assert step.estimate is not None
    assert step.estimate.y_m > 0.0

    estimator.reset()
    for tick, wz in enumerate([0.1, 0.1, 0.1, -0.1, -0.1], start=1):
        step = estimator.observe(
            _observation(
                time_s=tick * 0.02,
                tick=tick,
                wheel_velocity=_wheel_rad_s(0.0, 0.0, wz),
                gyro_z=wz,
            ),
            received_backend_monotonic_s=100.0 + tick * 0.02,
        )
    assert step.estimate is not None
    assert abs(step.estimate.yaw_rad) < 0.01


def test_invalid_and_stale_inputs_do_not_emit_new_pose():
    estimator = load_planar_odometry(PROFILE)
    invalid = estimator.observe(
        _observation(
            time_s=1.0,
            tick=1,
            wheel_velocity=[0.0] * 4,
            gyro_z=0.0,
            gyro_valid=False,
        ),
        received_backend_monotonic_s=101.0,
    )
    assert invalid.estimate is None
    assert invalid.health.status == NavigationHealthStatus.FAILED
    assert invalid.health.stop_required is True

    stale = estimator.observe(
        _observation(
            time_s=1.02,
            tick=2,
            wheel_velocity=[0.0] * 4,
            gyro_z=0.0,
        ),
        received_backend_monotonic_s=102.0,
    )
    assert stale.estimate is None
    assert "wheel_stale" in stale.health.rejection_reasons
    assert "imu_stale" in stale.health.rejection_reasons


def test_all_invalid_scan_is_not_treated_as_available():
    estimator = load_planar_odometry(PROFILE)
    observation = _observation(
        time_s=1.0,
        tick=1,
        wheel_velocity=[0.0] * 4,
        gyro_z=0.0,
    )
    invalid_scan = observation.scan.model_copy(
        update={
            "ranges_m": [None] * 4,
            "statuses": [LaserReturnStatus.INVALID] * 4,
            "valid_count": 0,
            "status_counts": {
                "HIT": 0,
                "NO_RETURN": 0,
                "INVALID": 4,
                "SELF_OCCLUDED": 0,
            },
        }
    )
    step = estimator.observe(
        observation.model_copy(update={"scan": invalid_scan}),
        received_backend_monotonic_s=101.0,
    )
    assert step.health.status == NavigationHealthStatus.STALE
    assert step.health.stop_required is True
    assert "scan_invalid_no_valid_beams" in step.health.rejection_reasons


def test_new_epoch_resets_origin_and_sequence():
    estimator = load_planar_odometry(PROFILE)
    for tick in range(1, 6):
        estimator.observe(
            _observation(
                time_s=tick * 0.02,
                tick=tick,
                wheel_velocity=_wheel_rad_s(0.02, 0.0, 0.0),
                gyro_z=0.0,
            ),
            received_backend_monotonic_s=100.0 + tick * 0.02,
        )
    step = estimator.observe(
        _observation(
            time_s=0.1,
            tick=1,
            epoch=1,
            episode="new_episode",
            wheel_velocity=[0.0] * 4,
            gyro_z=0.0,
        ),
        received_backend_monotonic_s=100.1,
    )
    assert step.estimate is not None
    assert step.estimate.clock.episode_id == "new_episode"
    assert step.estimate.x_m == 0.0
    assert step.estimate.y_m == 0.0


def test_covariance_is_nonzero_development_values():
    estimator = WheelGyroPlanarOdometry(
        {
            "interface_profile": "g3a-1.0",
            "profile_id": "g3a-nav-50hz-laser10-v1",
            "profile_version": "1.0",
            "sensors": {
                "scan": {"maximum_age_s": 0.3},
                "wheel": {
                    "wheel_names": [
                        "front_left_wheel",
                        "rear_left_wheel",
                        "front_right_wheel",
                        "rear_right_wheel",
                    ]
                },
            },
            "estimate": {
                "source": "wheel_gyro_planar_fusion",
                "algorithm": "complementary_planar",
                "complementary_gain": 0.08,
                "max_measurement_age_s": 0.2,
                "wheel_forward_kinematics": {
                    "half_wheelbase_plus_half_track_m": 0.141
                },
                "gyro_projection": {
                    "rotation_wxyz": [1.0, 0.0, 0.0, 0.0]
                },
                "covariance": {
                    "pose_diagonal": [0.0004, 0.0004, 0.0, 0.0, 0.0, 0.0025],
                    "twist_diagonal": [0.0004, 0.0004, 0.0, 0.0, 0.0, 0.0025],
                },
            },
        },
        profile_sha256="0" * 64,
    )
    assert len(estimator.pose_covariance) == 36
    assert estimator.pose_covariance[0] > 0.0
    assert estimator.twist_covariance[35] > 0.0
