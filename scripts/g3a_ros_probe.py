#!/usr/bin/env python3
"""Server-only ROS graph/message probe for G3A."""

from __future__ import annotations

import argparse
import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def plain_json(value):
    if value is None or isinstance(value, (str, bool, int, float)):
        return value
    if isinstance(value, dict):
        return {str(key): plain_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [plain_json(item) for item in value]
    if hasattr(value, "item"):
        return plain_json(value.item())
    return str(value)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--drive", action="store_true")
    parser.add_argument("--vx", type=float, default=0.02)
    parser.add_argument("--vy", type=float, default=0.0)
    parser.add_argument("--wz", type=float, default=0.05)
    args = parser.parse_args()

    received = {
        "clock": 0,
        "scan": 0,
        "imu": 0,
        "joint_states": 0,
        "odom": 0,
        "tf": 0,
        "health": 0,
    }
    samples = {}
    tf_children: dict[str, set[str]] = {}
    health_rows = []
    scan_status = {"finite": 0, "positive_inf": 0, "nan": 0}

    rclpy.init()
    node = rclpy.create_node("g3a_ros_acceptance_probe")
    sensor_qos = QoSProfile(depth=10)
    sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
    sensor_qos.durability = DurabilityPolicy.VOLATILE
    static_qos = QoSProfile(depth=10)
    static_qos.reliability = ReliabilityPolicy.RELIABLE
    static_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    command_publisher = node.create_publisher(
        Twist, "/mobilecar/cmd_vel_nav_test", 10
    )

    def on_clock(message: Clock):
        received["clock"] += 1
        samples["clock"] = {
            "sec": message.clock.sec,
            "nanosec": message.clock.nanosec,
        }

    def on_scan(message: LaserScan):
        received["scan"] += 1
        for value in message.ranges:
            scan_status[
                "nan"
                if math.isnan(value)
                else "positive_inf"
                if math.isinf(value) and value > 0
                else "finite"
            ] += 1
        samples["scan"] = {
            "frame_id": message.header.frame_id,
            "stamp": [message.header.stamp.sec, message.header.stamp.nanosec],
            "angle_min": message.angle_min,
            "angle_max": message.angle_max,
            "angle_increment": message.angle_increment,
            "time_increment": message.time_increment,
            "range_min": message.range_min,
            "range_max": message.range_max,
            "beam_count": len(message.ranges),
        }

    def on_imu(message: Imu):
        received["imu"] += 1
        samples["imu"] = {
            "frame_id": message.header.frame_id,
            "stamp": [message.header.stamp.sec, message.header.stamp.nanosec],
            "orientation_covariance_0": message.orientation_covariance[0],
            "gyro": [
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
            ],
        }

    def on_joint(message: JointState):
        received["joint_states"] += 1
        samples["joint_states"] = {
            "frame_id": message.header.frame_id,
            "names": list(message.name),
            "positions": list(message.position),
        }

    def on_odom(message: Odometry):
        received["odom"] += 1
        samples["odom"] = {
            "parent": message.header.frame_id,
            "child": message.child_frame_id,
            "stamp": [message.header.stamp.sec, message.header.stamp.nanosec],
            "pose_covariance_len": len(message.pose.covariance),
            "twist_covariance_len": len(message.twist.covariance),
        }

    def on_tf(message: TFMessage):
        received["tf"] += 1
        for transform in message.transforms:
            parent = transform.header.frame_id
            child = transform.child_frame_id
            tf_children.setdefault(parent, set()).add(child)

    def on_health(message: String):
        received["health"] += 1
        try:
            health_rows.append(json.loads(message.data))
        except json.JSONDecodeError:
            health_rows.append({"invalid_json": message.data})

    node.create_subscription(Clock, "/clock", on_clock, 20)
    node.create_subscription(LaserScan, "/scan", on_scan, sensor_qos)
    node.create_subscription(Imu, "/imu/data_raw", on_imu, sensor_qos)
    node.create_subscription(JointState, "/joint_states", on_joint, sensor_qos)
    node.create_subscription(Odometry, "/odom", on_odom, sensor_qos)
    node.create_subscription(TFMessage, "/tf", on_tf, 50)
    node.create_subscription(TFMessage, "/tf_static", on_tf, static_qos)
    node.create_subscription(
        String, "/mobilecar/navigation_health", on_health, 20
    )

    deadline = time.monotonic() + args.duration
    next_command = time.monotonic()
    while time.monotonic() < deadline:
        if args.drive and time.monotonic() >= next_command:
            message = Twist()
            message.linear.x = args.vx
            message.linear.y = args.vy
            message.angular.z = args.wz
            command_publisher.publish(message)
            next_command = time.monotonic() + 0.1
        rclpy.spin_once(node, timeout_sec=0.05)

    if args.drive:
        command_publisher.publish(Twist())
        end = time.monotonic() + 0.6
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    map_to_odom = "odom" in tf_children.get("map", set())
    report = {
        "generated_wall_utc": utc_now(),
        "duration_s": args.duration,
        "received": received,
        "samples": samples,
        "scan_status_counts": scan_status,
        "tf": {
            parent: sorted(children) for parent, children in tf_children.items()
        },
        "map_to_odom_published": map_to_odom,
        "health_latest": health_rows[-1] if health_rows else None,
        "gates": {
            "clock": received["clock"] > 0,
            "scan": received["scan"] > 0
            and samples.get("scan", {}).get("frame_id") == "laser_frame"
            and samples.get("scan", {}).get("beam_count") == 360
            and samples.get("scan", {}).get("time_increment") == 0.0,
            "imu": received["imu"] > 0
            and samples.get("imu", {}).get("frame_id") == "imu_link"
            and samples.get("imu", {}).get("orientation_covariance_0") == -1.0,
            "joint_states": received["joint_states"] > 0
            and samples.get("joint_states", {}).get("names")
            == ["joint1", "joint2", "joint3", "joint4", "joint5"],
            "odom": received["odom"] > 0
            and samples.get("odom", {}).get("parent") == "odom"
            and samples.get("odom", {}).get("child") == "base_link"
            and samples.get("odom", {}).get("pose_covariance_len") == 36
            and samples.get("odom", {}).get("twist_covariance_len") == 36,
            "tf": "base_link" in tf_children.get("odom", set())
            and "laser_frame" in tf_children.get("base_link", set())
            and "imu_link" in tf_children.get("base_link", set())
            and "link1" in tf_children.get("arm_base", set()),
            "no_map_to_odom": not map_to_odom,
            "health": received["health"] > 0,
            "nan_and_inf_present": scan_status["nan"] > 0
            and scan_status["positive_inf"] > 0,
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(plain_json(report), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    node.destroy_node()
    rclpy.shutdown()
    return 0 if all(report["gates"].values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
