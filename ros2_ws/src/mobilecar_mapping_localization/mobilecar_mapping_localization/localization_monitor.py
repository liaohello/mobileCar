"""Track recent scan-match pose and map TF independently."""

from __future__ import annotations

import json
import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener


class LocalizationMonitor(Node):
    def __init__(self) -> None:
        super().__init__("mobilecar_localization_monitor")
        self.declare_parameter("mode", "LOCALIZATION")
        self.declare_parameter("slam_node_name", "slam_toolbox")
        self.declare_parameter("scan_max_age_s", 0.30)
        self.declare_parameter("odom_max_age_s", 0.20)
        self.declare_parameter("moving_pose_max_age_s", 0.50)
        self.declare_parameter("stationary_pose_max_age_s", 2.00)
        self.declare_parameter("map_tf_max_age_s", 0.50)
        self.declare_parameter("node_max_age_s", 0.50)
        self.declare_parameter("motion_linear_threshold_m_s", 0.005)
        self.declare_parameter("motion_yaw_threshold_rad_s", 0.02)
        self.declare_parameter("enable_test_faults", False)
        self.mode = str(self.get_parameter("mode").value)
        self.slam_node_name = str(self.get_parameter("slam_node_name").value)
        self.last_scan_s: float | None = None
        self.last_odom_s: float | None = None
        self.last_pose_s: float | None = None
        self.last_map_tf_s: float | None = None
        self.last_node_seen_s: float | None = None
        self.last_scan_stamp_s: float | None = None
        self.moving = False
        self.match_fault_injected = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(
            String, "/mobilecar/localization_health", 20
        )
        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(LaserScan, "/scan", self.on_scan, sensor_qos)
        self.create_subscription(Odometry, "/odom", self.on_odom, sensor_qos)
        self.create_subscription(
            PoseWithCovarianceStamped, "/pose", self.on_pose, 20
        )
        self.create_subscription(
            Bool,
            "/mobilecar/localization/fault_match_failure",
            self.on_match_failure_fault,
            10,
        )
        self.sequence = 0
        self.stop_event = threading.Event()
        self.health_thread = threading.Thread(
            target=self._health_loop,
            name="g3b-localization-health",
            daemon=True,
        )
        self.health_thread.start()

    def _health_loop(self) -> None:
        while not self.stop_event.wait(0.1):
            self.publish_health()

    def on_scan(self, message: LaserScan) -> None:
        self.last_scan_s = time.monotonic()
        self.last_scan_stamp_s = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) * 1e-9
        )

    def on_odom(self, message: Odometry) -> None:
        self.last_odom_s = time.monotonic()
        linear = math.hypot(
            float(message.twist.twist.linear.x),
            float(message.twist.twist.linear.y),
        )
        yaw_rate = abs(float(message.twist.twist.angular.z))
        self.moving = (
            linear
            > float(self.get_parameter("motion_linear_threshold_m_s").value)
            or yaw_rate
            > float(self.get_parameter("motion_yaw_threshold_rad_s").value)
        )

    def on_pose(self, _message: PoseWithCovarianceStamped) -> None:
        self.last_pose_s = time.monotonic()

    def on_match_failure_fault(self, message: Bool) -> None:
        if not bool(self.get_parameter("enable_test_faults").value):
            return
        self.match_fault_injected = bool(message.data)

    @staticmethod
    def age(now: float, value: float | None) -> float | None:
        return None if value is None else max(0.0, now - value)

    def publish_health(self) -> None:
        now = time.monotonic()
        slam_alive = self.slam_node_name in set(self.get_node_names())
        if slam_alive:
            self.last_node_seen_s = now
        scan_age = self.age(now, self.last_scan_s)
        odom_age = self.age(now, self.last_odom_s)
        pose_age = self.age(now, self.last_pose_s)
        map_tf_age = None
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "odom", Time()
            )
            stamp = transform.header.stamp
            transform_sim_s = (
                float(stamp.sec) + float(stamp.nanosec) * 1e-9
            )
            if transform_sim_s > 0.0:
                self.last_map_tf_s = now
            map_tf_age = self.age(now, self.last_map_tf_s)
        except Exception:
            map_tf_age = self.age(now, self.last_map_tf_s)

        reasons = []
        if scan_age is None or scan_age > float(
            self.get_parameter("scan_max_age_s").value
        ):
            reasons.append("scan_stale")
        if odom_age is None or odom_age > float(
            self.get_parameter("odom_max_age_s").value
        ):
            reasons.append("odom_stale")
        if not slam_alive:
            reasons.append("slam_node_missing")
        if map_tf_age is None or map_tf_age > float(
            self.get_parameter("map_tf_max_age_s").value
        ):
            reasons.append("map_to_odom_stale_or_missing")

        pose_limit = float(
            self.get_parameter(
                "moving_pose_max_age_s"
                if self.moving
                else "stationary_pose_max_age_s"
            ).value
        )
        pose_fresh = (
            not self.match_fault_injected
            and pose_age is not None
            and pose_age <= pose_limit
        )
        if self.match_fault_injected:
            reasons.append("localization_match_fault_injected")
        if self.mode == "MAPPING":
            status = "READY" if slam_alive and scan_age is not None and odom_age is not None else "FAILED"
            stop_required = status != "READY"
        elif reasons:
            status = "FAILED" if "slam_node_missing" in reasons else "STALE"
            stop_required = True
        elif not pose_fresh:
            status = "STALE" if self.moving else "STATIONARY_HOLD"
            stop_required = self.moving
            if not self.moving:
                reasons.append("stationary_pose_hold")
        elif not self.moving:
            status = "STATIONARY_HOLD"
            stop_required = False
        else:
            status = "READY"
            stop_required = False

        self.sequence += 1
        payload = {
            "schema_version": 1,
            "interface_profile": "g3b-1.0",
            "sequence": self.sequence,
            "mode": self.mode,
            "status": status,
            "stop_required": stop_required,
            "moving": self.moving,
            "match_fault_injected": self.match_fault_injected,
            "slam_node_alive": slam_alive,
            "scan_fresh": scan_age is not None
            and scan_age <= float(self.get_parameter("scan_max_age_s").value),
            "odom_fresh": odom_age is not None
            and odom_age <= float(self.get_parameter("odom_max_age_s").value),
            "pose_match_fresh": pose_fresh,
            "map_to_odom_fresh": map_tf_age is not None
            and map_tf_age <= float(self.get_parameter("map_tf_max_age_s").value),
            "scan_age_s": scan_age,
            "odom_age_s": odom_age,
            "pose_match_age_s": pose_age,
            "map_to_odom_age_s": map_tf_age,
            "node_age_s": self.age(now, self.last_node_seen_s),
            "latest_scan_stamp_s": self.last_scan_stamp_s,
            "reasons": reasons,
            "clock_domain": "steady_monotonic_health; capture_sim_time_in_headers",
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"), allow_nan=False)
        self.publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = LocalizationMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop_event.set()
            node.health_thread.join(timeout=2.0)
            node.destroy_node()
        except (KeyboardInterrupt, RuntimeError):
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
