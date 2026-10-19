import math
from pathlib import Path
import sys


BRIDGE_SRC = (
    Path(__file__).resolve().parents[2]
    / "ros2_ws"
    / "src"
    / "mobilecar_navigation_bridge"
)
sys.path.insert(0, str(BRIDGE_SRC))

from mobilecar_navigation_bridge.bridge_node import scan_ranges, stamp_from_seconds
from mobilecar_navigation_bridge.math_utils import (
    axis_angle_xyzw,
    multiply_xyzw,
    normalize_xyzw,
    rotate_xyzw,
)


def test_scan_status_mapping_is_explicit():
    values = scan_ranges(
        {
            "ranges_m": [1.2, None, None, None],
            "statuses": ["HIT", "NO_RETURN", "INVALID", "SELF_OCCLUDED"],
        }
    )
    assert values[0] == 1.2
    assert math.isinf(values[1]) and values[1] > 0.0
    assert math.isnan(values[2])
    assert math.isnan(values[3])


def test_stamp_rounding_carries_nanoseconds():
    stamp = stamp_from_seconds(1.9999999999)
    assert (stamp.sec, stamp.nanosec) == (2, 0)


def test_quaternion_helpers_are_normalized():
    quaternion = axis_angle_xyzw([0.0, 0.0, 1.0], math.pi / 2.0)
    assert abs(sum(value * value for value in quaternion) - 1.0) < 1e-12
    x, y, z = rotate_xyzw(quaternion, [1.0, 0.0, 0.0])
    assert abs(x) < 1e-12
    assert abs(y - 1.0) < 1e-12
    assert abs(z) < 1e-12
