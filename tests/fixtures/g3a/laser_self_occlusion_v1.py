"""Deterministic 360-beam geometry fixture for G3A scan/status conversion."""

from __future__ import annotations

import math

from mobilecar.contracts.navigation import (
    G3A_NAVIGATION_PROFILE_ID,
    LaserReturnStatus,
    LaserScanObservation,
    NavigationClockV1,
)


PROFILE_SHA256 = "0" * 64
BEAM_COUNT = 360
ANGLE_MIN_RAD = -math.pi
ANGLE_INCREMENT_RAD = math.pi / 180.0
RANGE_MIN_M = 0.10
RANGE_MAX_M = 8.00


def _ray_aabb_distance(
    origin: tuple[float, float],
    direction: tuple[float, float],
    minimum: tuple[float, float],
    maximum: tuple[float, float],
) -> float | None:
    near = -math.inf
    far = math.inf
    for start, delta, lower, upper in zip(
        origin,
        direction,
        minimum,
        maximum,
    ):
        if abs(delta) < 1e-12:
            if start < lower or start > upper:
                return None
            continue
        first = (lower - start) / delta
        second = (upper - start) / delta
        if first > second:
            first, second = second, first
        near = max(near, first)
        far = min(far, second)
        if near > far:
            return None
    if far < 0.0:
        return None
    return near if near >= 0.0 else far


def _fixture_clock() -> NavigationClockV1:
    return NavigationClockV1(
        run_id="run_laser_fixture",
        episode_id="episode_laser_fixture",
        backend_generation=1,
        navigation_epoch=0,
        control_tick_id="tick_fixture_000000001",
        capture_sim_time_s=1.25,
        capture_backend_monotonic_s=101.25,
    )


def build_self_occlusion_scan() -> LaserScanObservation:
    """Return an explicit synthetic scan; this is not MuJoCo evidence.

    The laser origin is 0.073 m forward of base_link. A conservative robot
    blocker occupies x=[-0.096, 0.020], y=[-0.048, 0.048] in base coordinates,
    excluding the sensor origin itself. Rear/side rays that hit this body are
    SELF_OCCLUDED. Two deterministic beams are injected as INVALID so tests can
    prove that invalid measurements never become fabricated finite ranges.
    """

    invalid_indices = {171, 189}
    ranges: list[float | None] = []
    statuses: list[LaserReturnStatus] = []
    for index in range(BEAM_COUNT):
        angle = ANGLE_MIN_RAD + index * ANGLE_INCREMENT_RAD
        direction = (math.cos(angle), math.sin(angle))
        if index in invalid_indices:
            ranges.append(None)
            statuses.append(LaserReturnStatus.INVALID)
            continue

        panel_distance: float | None = None
        if -0.55 <= angle <= 0.55:
            panel_distance = 1.40 / max(math.cos(angle), 1e-6) - 0.073
        elif 2.35 <= angle <= 2.80 or -2.80 <= angle <= -2.35:
            panel_distance = 1.10
        if panel_distance is not None and RANGE_MIN_M <= panel_distance <= RANGE_MAX_M:
            ranges.append(round(panel_distance, 9))
            statuses.append(LaserReturnStatus.HIT)
            continue

        self_distance = _ray_aabb_distance(
            origin=(0.073, 0.0),
            direction=direction,
            minimum=(-0.096, -0.048),
            maximum=(0.020, 0.048),
        )
        if self_distance is not None and RANGE_MIN_M <= self_distance <= RANGE_MAX_M:
            ranges.append(None)
            statuses.append(LaserReturnStatus.SELF_OCCLUDED)
        else:
            ranges.append(None)
            statuses.append(LaserReturnStatus.NO_RETURN)

    counts = {status.value: statuses.count(status) for status in LaserReturnStatus}
    return LaserScanObservation(
        profile_id=G3A_NAVIGATION_PROFILE_ID,
        profile_version="1.0",
        profile_sha256=PROFILE_SHA256,
        clock=_fixture_clock(),
        scan_id="scan_laser_self_occlusion_fixture",
        angle_min_rad=ANGLE_MIN_RAD,
        angle_max_rad=ANGLE_MIN_RAD + (BEAM_COUNT - 1) * ANGLE_INCREMENT_RAD,
        angle_increment_rad=ANGLE_INCREMENT_RAD,
        range_min_m=RANGE_MIN_M,
        range_max_m=RANGE_MAX_M,
        ranges_m=ranges,
        statuses=statuses,
        beam_count=BEAM_COUNT,
        valid_count=counts[LaserReturnStatus.HIT.value],
        status_counts=counts,
    )
