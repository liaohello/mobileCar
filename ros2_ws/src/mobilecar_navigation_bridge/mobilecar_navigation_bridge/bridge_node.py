"""G3A ROS 2 Humble bridge; this process never creates a MuJoCo simulation."""

from __future__ import annotations

import json
import math
import threading
import time
from pathlib import Path
from typing import Any

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from .ipc_client import JsonLineIpcClient
from .math_utils import (
    add_vectors,
    axis_angle_xyzw,
    multiply_xyzw,
    normalize_xyzw,
    rotate_xyzw,
    yaw_xyzw,
)


ARM_CHAIN = (
    {
        "parent": "arm_base",
        "child": "link1",
        "translation": [0.0, 0.0, 0.0],
        "rotation": [0.0, 0.0, 0.0, 1.0],
        "axis": [0.0, -0.00872653549837, 0.999961923064],
    },
    {
        "parent": "link1",
        "child": "link2",
        "translation": [0.0, -0.0182247260842, 0.0314158515121],
        "rotation": [
            5.65531454e-05,
            0.0129614374581,
            -0.00436277662887,
            0.999906477687,
        ],
        "axis": [0.00859171009913, -0.999923369532, 0.00891277624995],
    },
    {
        "parent": "link2",
        "child": "link3",
        "translation": [-0.000861228256677, 0.000635863697295, 0.074117298162],
        "rotation": [
            5.08689256953e-05,
            0.000136312396919,
            5.0865005279e-05,
            0.999999988122,
        ],
        "axis": [0.00859391749792, -0.999923336386, 0.00891436659751],
    },
    {
        "parent": "link3",
        "child": "link4",
        "translation": [0.0, 0.00464364220586, 0.0741383282674],
        "rotation": [0.0, -0.00945925729011, 0.0, 0.999955260225],
        "axis": [0.00859391749792, -0.999923336386, 0.00891436659751],
    },
    {
        "parent": "link4",
        "child": "link5",
        "translation": [
            -0.000167622119569,
            0.0159322086602,
            0.0497506513724,
        ],
        "rotation": [
            -0.00436280558595,
            -4.88052580014e-05,
            0.00436356375386,
            0.999980961247,
        ],
        "axis": [-0.000182847987243, 0.0, -0.999999983283],
    },
    {
        "parent": "link5",
        "child": "gripper_link",
        "translation": [-0.00905353309436, 0.0, 0.0217251857672],
        "rotation": [0.0, -0.000110747320076, 0.0, 0.999999993868],
        "axis": None,
    },
)

BASE_ARM_BASE = (
    [0.04458406638466714, -0.00003013824670391, 0.06913071455718081],
    [0.0, 0.0, 0.0, 1.0],
)
CAMERA_CONNECT = (
    [-0.035657, 0.015573, 0.044852],
    [-2.54728808161e-05, -0.704036586265, 0.00617009533991, 0.710136898404],
)
DEPTH_CAMERA = (
    [0.022543, -0.0010708, 0.0052065],
    [0.00436324590085, -0.0042954459009, 1.87424380415e-05, 0.999981255264],
)


def stamp_from_seconds(value: float) -> Time:
    seconds = float(value)
    whole = int(math.floor(seconds))
    nanoseconds = int(round((seconds - whole) * 1_000_000_000))
    if nanoseconds >= 1_000_000_000:
        whole += 1
        nanoseconds -= 1_000_000_000
    return Time(sec=whole, nanosec=nanoseconds)


def transform_message(
    *,
    parent: str,
    child: str,
    stamp: Time,
    translation,
    rotation_xyzw,
) -> TransformStamped:
    message = TransformStamped()
    message.header.stamp = stamp
    message.header.frame_id = parent
    message.child_frame_id = child
    message.transform.translation.x = float(translation[0])
    message.transform.translation.y = float(translation[1])
    message.transform.translation.z = float(translation[2])
    x, y, z, w = normalize_xyzw(rotation_xyzw)
    message.transform.rotation.x = x
    message.transform.rotation.y = y
    message.transform.rotation.z = z
    message.transform.rotation.w = w
    return message


def scan_ranges(scan: dict[str, Any]) -> list[float]:
    result: list[float] = []
    for range_m, status in zip(scan["ranges_m"], scan["statuses"]):
        if status == "HIT":
            if range_m is None or not math.isfinite(float(range_m)):
                raise ValueError("HIT beam has no finite range")
            result.append(float(range_m))
        elif status == "NO_RETURN":
            result.append(float("inf"))
        elif status in {"INVALID", "SELF_OCCLUDED"}:
            result.append(float("nan"))
        else:
            raise ValueError(f"unknown laser status {status!r}")
    return result


class NavigationBridge(Node):
    def __init__(self) -> None:
        super().__init__("mobilecar_navigation_bridge")
        self.declare_parameter("socket_path", "../../var/ipc/g3a_navigation.sock")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("laser_frame", "laser_frame")
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("command_topic", "/mobilecar/cmd_vel_nav_test")
        self.declare_parameter("health_topic", "/mobilecar/navigation_health")
        self.declare_parameter("command_timeout_s", 0.25)
        self.declare_parameter("command_lease_ttl_s", 0.35)
        self.declare_parameter("command_session_id", "g3a_ros_bridge")
        self.declare_parameter("control_mode", "g3a")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_static_tf", True)
        self.declare_parameter("max_command_translation_m_s", 0.05)
        self.declare_parameter("max_command_yaw_rad_s", 0.30)
        self.declare_parameter("sensor_qos_depth", 5)
        self.declare_parameter("tf_qos_depth", 30)
        self.declare_parameter("t_base_laser", [0.07302117405202284, 0.0, 0.08])
        self.declare_parameter("q_base_laser_xyzw", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("t_base_imu", [0.0, 0.0, 0.035])
        self.declare_parameter("q_base_imu_xyzw", [0.0, 0.0, 0.0, 1.0])
        sensor_depth = int(self.get_parameter("sensor_qos_depth").value)
        tf_depth = int(self.get_parameter("tf_qos_depth").value)
        self.sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=sensor_depth,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=tf_depth,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.clock_publisher = self.create_publisher(Clock, "/clock", 10)
        self.scan_publisher = self.create_publisher(
            LaserScan, "/scan", self.sensor_qos
        )
        self.imu_publisher = self.create_publisher(
            Imu, "/imu/data_raw", self.sensor_qos
        )
        self.joint_publisher = self.create_publisher(
            JointState, "/joint_states", self.sensor_qos
        )
        self.odom_publisher = self.create_publisher(
            Odometry, "/odom", self.sensor_qos
        )
        self.tf_publisher = self.create_publisher(TFMessage, "/tf", self.tf_qos)
        self.static_tf_publisher = self.create_publisher(
            TFMessage, "/tf_static", self.static_qos
        )
        self.health_publisher = self.create_publisher(
            String,
            str(self.get_parameter("health_topic").value),
            QoSProfile(depth=20),
        )
        self.command_subscription = self.create_subscription(
            Twist,
            str(self.get_parameter("command_topic").value),
            self.command_callback,
            QoSProfile(depth=10),
        )
        self.state_lock = threading.RLock()
        self.identity: dict[str, Any] | None = None
        self.latest_clock: dict[str, Any] | None = None
        self._gate_last_envelope_monotonic_s: float | None = None
        self._gate_last_sim_time_s: float | None = None
        self.last_command_monotonic_s: float | None = None
        self.command_active = False
        self.release_sent = True
        self.gyro_missing = False
        self.stop_event = threading.Event()
        self.last_release_reason = ""
        self.static_published = False
        self.session_id = str(self.get_parameter("command_session_id").value)
        self.control_mode = str(self.get_parameter("control_mode").value)
        socket_path = Path(str(self.get_parameter("socket_path").value)).resolve()
        self.ipc = JsonLineIpcClient(socket_path, self.ipc_message)
        self.ipc.start()
        self.watchdog = threading.Thread(
            target=self.watchdog_loop,
            name="mobilecar-navigation-watchdog",
            daemon=True,
        )
        self.watchdog.start()
        self.get_logger().info(f"navigation IPC: {socket_path}")
        if bool(self.get_parameter("publish_static_tf").value):
            self.publish_static_transforms()

    def _publish(self, publisher, message) -> None:
        publisher.publish(message)

    def publish_static_transforms(self) -> None:
        stamp = self.get_clock().now().to_msg()
        transforms = [
            transform_message(
                parent="base_link",
                child="laser_frame",
                stamp=stamp,
                translation=self.get_parameter("t_base_laser").value,
                rotation_xyzw=self.get_parameter("q_base_laser_xyzw").value,
            ),
            transform_message(
                parent="base_link",
                child="imu_link",
                stamp=stamp,
                translation=self.get_parameter("t_base_imu").value,
                rotation_xyzw=self.get_parameter("q_base_imu_xyzw").value,
            ),
            transform_message(
                parent="base_link",
                child="arm_base",
                stamp=stamp,
                translation=BASE_ARM_BASE[0],
                rotation_xyzw=BASE_ARM_BASE[1],
            ),
            transform_message(
                parent="link4",
                child="camera_connect_link",
                stamp=stamp,
                translation=CAMERA_CONNECT[0],
                rotation_xyzw=CAMERA_CONNECT[1],
            ),
            transform_message(
                parent="camera_connect_link",
                child="depth_cam_link",
                stamp=stamp,
                translation=DEPTH_CAMERA[0],
                rotation_xyzw=DEPTH_CAMERA[1],
            ),
        ]
        self.static_tf_publisher.publish(TFMessage(transforms=transforms))
        self.static_published = True

    def ipc_message(self, message: dict[str, Any]) -> None:
        message_type = message.get("message_type")
        if message_type == "control_receipt":
            self.on_control_receipt(message)
            return
        if message_type == "heartbeat":
            self.on_heartbeat(message)
            return
        clock = None
        payload = message.get("payload")
        if isinstance(payload, dict):
            clock = payload.get("clock")
        if (
            not isinstance(clock, dict)
            or not isinstance(clock.get("capture_sim_time_s"), (int, float))
            or not math.isfinite(float(clock["capture_sim_time_s"]))
        ):
            self.publish_health(
                "STALE",
                ["missing or invalid capture sim time; clock not advanced"],
                stop_required=True,
            )
            return
        envelope_monotonic = message.get("backend_monotonic_s")
        if not isinstance(envelope_monotonic, (int, float)) or not math.isfinite(
            float(envelope_monotonic)
        ):
            self.publish_health(
                "STALE",
                ["missing backend monotonic envelope identity"],
                stop_required=True,
            )
            return
        identity = {
            "run_id": message["run_id"],
            "episode_id": message["episode_id"],
            "backend_generation": int(message["backend_generation"]),
            "navigation_epoch": int(message["navigation_epoch"]),
        }
        capture_sim_time = float(clock["capture_sim_time_s"])
        with self.state_lock:
            previous_identity = self.identity
            identity_changed = self.identity != identity
            if (
                self._gate_last_envelope_monotonic_s is not None
                and float(envelope_monotonic)
                < self._gate_last_envelope_monotonic_s
            ):
                stale_envelope = True
            else:
                stale_envelope = False
            if identity_changed and (
                self._gate_last_envelope_monotonic_s is not None
                and float(envelope_monotonic)
                <= self._gate_last_envelope_monotonic_s
            ):
                stale_envelope = True
            if (
                not identity_changed
                and self._gate_last_sim_time_s is not None
                and capture_sim_time + 1e-9 < self._gate_last_sim_time_s
            ):
                clock_regression = True
            else:
                clock_regression = False
            if stale_envelope or clock_regression:
                self.latest_clock = dict(self.latest_clock or {})
            else:
                self._gate_last_envelope_monotonic_s = float(envelope_monotonic)
                self._gate_last_sim_time_s = capture_sim_time
                self.identity = identity
                self.latest_clock = clock
                if identity_changed:
                    self.last_command_monotonic_s = None
                    self.command_active = False
                    self.release_sent = True
        if stale_envelope:
            self.publish_health(
                "STALE",
                ["stale envelope or rejected old identity transition"],
                stop_required=True,
            )
            return
        if clock_regression:
            self.publish_health(
                "STALE",
                ["capture sim clock regression in same identity"],
                stop_required=True,
            )
            return
        if identity_changed:
            self.on_identity_changed(previous_identity, identity)
        self.publish_clock(capture_sim_time)
        if message_type == "observation":
            self.observation_message(payload)
        elif message_type == "estimate":
            self.estimate_message(payload)
        elif message_type == "health":
            self.health_message(payload)
        elif message_type != "heartbeat":
            self.publish_health(
                "FAILED",
                [f"unknown IPC message type: {message_type!r}"],
                stop_required=True,
            )

    def on_control_receipt(self, message: dict[str, Any]) -> None:
        return None

    def on_heartbeat(self, message: dict[str, Any]) -> None:
        return None

    def on_identity_changed(
        self,
        previous: dict[str, Any] | None,
        current: dict[str, Any],
    ) -> None:
        return None

    def current_transport_state(self) -> tuple[dict[str, Any] | None, dict[str, Any] | None]:
        with self.state_lock:
            identity = dict(self.identity) if self.identity is not None else None
            clock = dict(self.latest_clock) if self.latest_clock is not None else None
        return identity, clock

    def publish_clock(self, sim_time_s: float) -> None:
        message = Clock()
        message.clock = stamp_from_seconds(sim_time_s)
        self.clock_publisher.publish(message)

    def observation_message(self, payload: dict[str, Any]) -> None:
        if not isinstance(payload, dict):
            raise ValueError("observation payload must be an object")
        clock = payload["clock"]
        stamp = stamp_from_seconds(clock["capture_sim_time_s"])
        joints = payload["joints"]
        joint_message = JointState()
        joint_message.header.stamp = stamp
        joint_message.header.frame_id = "base_link"
        joint_message.name = [str(value) for value in joints["joint_names"]]
        joint_message.position = [float(value) for value in joints["model_positions_rad"]]
        self.joint_publisher.publish(joint_message)
        self.publish_arm_transforms(stamp, joint_message)

        imu_payload = payload["imu"]
        gyro_valid = bool(imu_payload["gyro_valid"])
        with self.state_lock:
            self.gyro_missing = not gyro_valid
        if gyro_valid:
            self.publish_imu(stamp, imu_payload)
        else:
            self.publish_health(
                "FAILED",
                ["imu_gyro_invalid_or_missing"],
                stop_required=True,
            )
        scan = payload.get("scan")
        if scan is not None:
            self.publish_scan(stamp, scan)

    def publish_imu(self, stamp: Time, payload: dict[str, Any]) -> None:
        message = Imu()
        message.header.stamp = stamp
        message.header.frame_id = str(self.get_parameter("imu_frame").value)
        message.orientation.w = 1.0
        message.orientation_covariance[0] = -1.0
        gyro = payload.get("angular_velocity_rad_s")
        if not isinstance(gyro, list):
            return
        message.angular_velocity.x = float(gyro[0])
        message.angular_velocity.y = float(gyro[1])
        message.angular_velocity.z = float(gyro[2])
        acceleration = payload.get("linear_acceleration_m_s2")
        if isinstance(acceleration, list):
            message.linear_acceleration.x = float(acceleration[0])
            message.linear_acceleration.y = float(acceleration[1])
            message.linear_acceleration.z = float(acceleration[2])
            for index in range(9):
                message.linear_acceleration_covariance[index] = (
                    0.0 if index == 0 else 0.0
                )
            message.linear_acceleration_covariance[0] = 0.01
        else:
            message.linear_acceleration_covariance[0] = -1.0
        for index in range(9):
            message.angular_velocity_covariance[index] = 0.0
        message.angular_velocity_covariance[0] = 0.0025
        self.imu_publisher.publish(message)

    def publish_scan(self, stamp: Time, payload: dict[str, Any]) -> None:
        message = LaserScan()
        message.header.stamp = stamp
        message.header.frame_id = str(self.get_parameter("laser_frame").value)
        message.angle_min = float(payload["angle_min_rad"])
        message.angle_max = float(payload["angle_max_rad"])
        message.angle_increment = float(payload["angle_increment_rad"])
        message.time_increment = 0.0
        message.scan_time = 0.1
        message.range_min = float(payload["range_min_m"])
        message.range_max = float(payload["range_max_m"])
        message.ranges = scan_ranges(payload)
        self.scan_publisher.publish(message)

    def _yaw_transform(self, parent: str, child: str, stamp: Time, x: float, y: float, yaw: float):
        return transform_message(
            parent=parent,
            child=child,
            stamp=stamp,
            translation=[x, y, 0.0],
            rotation_xyzw=yaw_xyzw(yaw),
        )

    def estimate_message(self, payload: dict[str, Any]) -> None:
        if not isinstance(payload, dict):
            raise ValueError("estimate payload must be an object")
        clock = payload["clock"]
        stamp = stamp_from_seconds(clock["capture_sim_time_s"])
        parent = str(self.get_parameter("odom_frame").value)
        child = str(self.get_parameter("base_frame").value)
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = parent
        message.child_frame_id = child
        message.pose.pose.position.x = float(payload["x_m"])
        message.pose.pose.position.y = float(payload["y_m"])
        message.pose.pose.position.z = 0.0
        x, y, z, w = yaw_xyzw(float(payload["yaw_rad"]))
        message.pose.pose.orientation.x = x
        message.pose.pose.orientation.y = y
        message.pose.pose.orientation.z = z
        message.pose.pose.orientation.w = w
        message.twist.twist.linear.x = float(payload["vx_body_m_s"])
        message.twist.twist.linear.y = float(payload["vy_body_m_s"])
        message.twist.twist.angular.z = float(payload["wz_body_rad_s"])
        pose_covariance = [float(value) for value in payload["pose_covariance"]]
        twist_covariance = [float(value) for value in payload["twist_covariance"]]
        if len(pose_covariance) != 36 or len(twist_covariance) != 36:
            raise ValueError("odometry covariance must contain 36 values")
        message.pose.covariance = pose_covariance
        message.twist.covariance = twist_covariance
        self.odom_publisher.publish(message)
        if bool(self.get_parameter("publish_tf").value):
            transform = self._yaw_transform(
                parent,
                child,
                stamp,
                float(payload["x_m"]),
                float(payload["y_m"]),
                float(payload["yaw_rad"]),
            )
            self.tf_publisher.publish(TFMessage(transforms=[transform]))

    def publish_arm_transforms(self, stamp: Time, joints: JointState) -> None:
        if not bool(self.get_parameter("publish_tf").value):
            return
        positions = {
            name: float(position)
            for name, position in zip(joints.name, joints.position)
        }
        transforms = []
        for index, item in enumerate(ARM_CHAIN):
            joint_name = f"joint{index + 1}"
            joint_rotation = (
                axis_angle_xyzw(item["axis"], positions[joint_name])
                if item["axis"] is not None and joint_name in positions
                else [0.0, 0.0, 0.0, 1.0]
            )
            rotation = multiply_xyzw(item["rotation"], joint_rotation)
            transforms.append(
                transform_message(
                    parent=str(item["parent"]),
                    child=str(item["child"]),
                    stamp=stamp,
                    translation=item["translation"],
                    rotation_xyzw=rotation,
                )
            )
        self.tf_publisher.publish(TFMessage(transforms=transforms))

    def health_message(self, payload: dict[str, Any]) -> None:
        if not isinstance(payload, dict):
            raise ValueError("health payload must be an object")
        self.publish_health(
            str(payload.get("status", "UNKNOWN")),
            [str(value) for value in payload.get("rejection_reasons", [])],
            stop_required=bool(payload.get("stop_required", False)),
            source_health=payload,
        )

    def publish_health(
        self,
        status: str,
        reasons: list[str],
        *,
        stop_required: bool,
        source_health: dict[str, Any] | None = None,
    ) -> None:
        with self.state_lock:
            identity = dict(self.identity) if self.identity is not None else None
            command = {
                "active": self.command_active,
                "release_sent": self.release_sent,
                "last_release_reason": self.last_release_reason,
            }
        payload = {
            "schema_version": 1,
            "status": status,
            "stop_required": bool(stop_required),
            "reasons": reasons,
            "identity": identity,
            "command": command,
            "ipc": {
                "connected": self.ipc.connected,
                "received": self.ipc.received,
                "invalid": self.ipc.invalid,
                "sent": self.ipc.sent,
                "send_failures": self.ipc.send_failures,
                "last_error": self.ipc.last_error,
            },
            "source_health": source_health,
            "clock_domain": "capture_sim_time_for_headers; steady_monotonic_watchdog",
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"), allow_nan=False)
        self.health_publisher.publish(message)

    def command_callback(self, message: Twist) -> None:
        with self.state_lock:
            identity = dict(self.identity) if self.identity is not None else None
            clock = dict(self.latest_clock) if self.latest_clock is not None else None
        if identity is None or clock is None:
            self.publish_health(
                "STALE",
                ["navigation stream identity not available"],
                stop_required=True,
            )
            return
        values = [
            float(message.linear.x),
            float(message.linear.y),
            float(message.angular.z),
        ]
        if not all(math.isfinite(value) for value in values):
            self.publish_health(
                "FAILED", ["cmd_vel contains non-finite value"], stop_required=True
            )
            return
        maximum_translation = float(
            self.get_parameter("max_command_translation_m_s").value
        )
        maximum_yaw = float(self.get_parameter("max_command_yaw_rad_s").value)
        norm = math.hypot(values[0], values[1])
        if norm > maximum_translation:
            scale = maximum_translation / norm
            values[0] *= scale
            values[1] *= scale
        values[2] = max(-maximum_yaw, min(maximum_yaw, values[2]))
        self.on_command_values(values)

    def on_command_values(self, values: list[float]) -> None:
        lease = float(self.get_parameter("command_lease_ttl_s").value)
        control = {
            "schema_version": 1,
            "message_type": "cmd_vel",
            "source": "ros_navigation_test",
            **identity,
            "session_id": self.session_id,
            "lease_ttl_s": lease,
            "vx_m_s": values[0],
            "vy_m_s": values[1],
            "wz_rad_s": values[2],
            "reason": "ros_twist",
        }
        if self.ipc.send(control):
            with self.state_lock:
                self.last_command_monotonic_s = time.monotonic()
                self.command_active = True
                self.release_sent = False
        else:
            self.publish_health(
                "STALE",
                ["navigation IPC command channel unavailable"],
                stop_required=True,
            )

    def send_g3b_control(self, message: dict[str, Any]) -> bool:
        return self.ipc.send(message)

    def send_release(self, reason: str) -> None:
        with self.state_lock:
            identity = dict(self.identity) if self.identity is not None else None
        if identity is None:
            return
        control = {
            "schema_version": 1,
            "message_type": "release",
            "source": "ros_navigation_test",
            **identity,
            "session_id": self.session_id,
            "lease_ttl_s": float(
                self.get_parameter("command_lease_ttl_s").value
            ),
            "vx_m_s": 0.0,
            "vy_m_s": 0.0,
            "wz_rad_s": 0.0,
            "reason": reason,
        }
        self.ipc.send(control)
        with self.state_lock:
            self.command_active = False
            self.release_sent = True
            self.last_release_reason = reason

    def watchdog_loop(self) -> None:
        timeout = float(self.get_parameter("command_timeout_s").value)
        last_connected = None
        while not self.stop_event.wait(0.05):
            with self.state_lock:
                active = self.command_active
                last_command = self.last_command_monotonic_s
            if (
                self.control_mode != "g3b"
                and active
                and last_command is not None
                and time.monotonic() - last_command > timeout
            ):
                self.send_release("cmd_vel_input_timeout")
                self.publish_health(
                    "STALE",
                    ["cmd_vel_input_timeout"],
                    stop_required=True,
                )
            if self.ipc.connected != last_connected:
                last_connected = self.ipc.connected
                self.publish_health(
                    "HEALTHY" if last_connected else "STALE",
                    [] if last_connected else ["navigation IPC disconnected"],
                    stop_required=not bool(last_connected),
                )

    def close(self) -> None:
        if self.command_active:
            self.send_release("bridge_shutdown")
        self.stop_event.set()
        self.watchdog.join(timeout=2.0)
        self.ipc.close()
        self.destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.close()
        except (KeyboardInterrupt, RuntimeError):
            pass
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
