"""Small deterministic quaternion and transform helpers."""

from __future__ import annotations

import math


def normalize_xyzw(values) -> tuple[float, float, float, float]:
    x, y, z, w = (float(value) for value in values)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("invalid quaternion")
    return x / norm, y / norm, z / norm, w / norm


def multiply_xyzw(first, second) -> tuple[float, float, float, float]:
    ax, ay, az, aw = normalize_xyzw(first)
    bx, by, bz, bw = normalize_xyzw(second)
    return normalize_xyzw(
        (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )
    )


def rotate_xyzw(quaternion, vector) -> tuple[float, float, float]:
    x, y, z, w = normalize_xyzw(quaternion)
    vx, vy, vz = (float(value) for value in vector)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def add_vectors(first, second) -> tuple[float, float, float]:
    return tuple(float(a) + float(b) for a, b in zip(first, second))


def axis_angle_xyzw(axis, angle: float) -> tuple[float, float, float, float]:
    x, y, z = (float(value) for value in axis)
    norm = math.sqrt(x * x + y * y + z * z)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("joint axis must be nonzero")
    half = 0.5 * float(angle)
    scale = math.sin(half) / norm
    return normalize_xyzw((x * scale, y * scale, z * scale, math.cos(half)))


def yaw_xyzw(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * float(yaw)
    return 0.0, 0.0, math.sin(half), math.cos(half)


def transform_multiply(first, second):
    first_position, first_rotation = first
    second_position, second_rotation = second
    rotated = rotate_xyzw(first_rotation, second_position)
    return (
        add_vectors(first_position, rotated),
        multiply_xyzw(first_rotation, second_rotation),
    )
