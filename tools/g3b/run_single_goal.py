#!/usr/bin/env python3
"""Server-only G3B single-scene localization and Nav2 goal acceptance."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import signal
import socket
import subprocess
import sys
import threading
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from uuid import uuid4

import httpx
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
import yaml

from mobilecar.config import load_config


ROOT = Path(__file__).resolve().parents[2]
PROJECT_ROOT = ROOT.parents[1] if ROOT.parent.name == "releases" else ROOT
PYTHON = PROJECT_ROOT / "envs/g2a-py310-mujoco316/bin/python"


class ExpectedGoalRejection(RuntimeError):
    """Terminate a negative goal test after preserving its evidence."""


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def atomic_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{uuid4().hex}.tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def yaw_from_quaternion(values: list[float]) -> float:
    x, y, z, w = values
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def angle_error(first: float, second: float) -> float:
    return abs(math.atan2(math.sin(first - second), math.cos(first - second)))


def free_port() -> int:
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


class Process:
    def __init__(self, command: list[str], log: Path, env: dict[str, str]) -> None:
        self.handle = log.open("w", encoding="utf-8")
        self.process = subprocess.Popen(
            command,
            cwd=ROOT,
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=self.handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            text=True,
        )

    def alive(self) -> bool:
        return self.process.poll() is None

    def close(self, timeout_s: float = 15.0) -> int | None:
        if self.alive():
            try:
                os.killpg(self.process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                self.process.wait(timeout=timeout_s)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(self.process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    self.process.wait(timeout=8.0)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(self.process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    self.process.wait(timeout=5.0)
        self.handle.close()
        return self.process.returncode


def ros_command_output(command: list[str], env: dict[str, str]) -> str:
    result = subprocess.run(
        command,
        cwd=ROOT,
        env=env,
        text=True,
        capture_output=True,
        timeout=20.0,
    )
    return result.stdout


def wait_graph(
    env: dict[str, str],
    *,
    topics: set[str],
    actions: set[str],
    timeout_s: float,
) -> tuple[set[str], set[str]]:
    deadline = time.monotonic() + timeout_s
    latest_topics: set[str] = set()
    latest_actions: set[str] = set()
    while time.monotonic() < deadline:
        latest_topics = {
            value.strip()
            for value in ros_command_output(["ros2", "topic", "list"], env).splitlines()
            if value.strip()
        }
        latest_actions = {
            value.split()[0]
            for value in ros_command_output(["ros2", "action", "list", "-t"], env).splitlines()
            if value.strip()
        }
        if topics.issubset(latest_topics) and actions.issubset(latest_actions):
            break
        time.sleep(0.5)
    return latest_topics, latest_actions


def arm_activity(
    client: httpx.Client,
    stop_event: threading.Event,
    *,
    session: str,
    sequence: int,
    result_sink: dict[str, Any] | None = None,
) -> dict[str, Any]:
    snapshot = client.get("/api/v1/snapshot").json()
    initial = list(snapshot["arm"]["user_joint_positions_rad"])
    target = list(initial)
    target[0] += 0.04
    target[1] += 0.02
    request = {
        "contract_version": "mobilecar-1.0",
        "run_id": snapshot["run_id"],
        "backend_generation": snapshot["backend_generation"],
        "episode_id": snapshot["episode_id"],
        "session_id": session,
        "operator_id": "g3b_arm_activity",
        "operator_kind": "scripted",
        "view_source": "g3b_arm_activity",
        "command_id": f"g3b_arm_{uuid4().hex}",
        "command_seq": sequence,
        "command_type": "set_arm_joint_goal",
        "requested_ttl_s": 3.0,
        "payload": {
            "joint_names": [f"joint{index}" for index in range(1, 6)],
            "joint_positions_rad": target,
        },
    }
    response = client.post("/api/v1/commands", json=request)
    response.raise_for_status()
    command_response = response.json()
    result = {
        "initial_joints": initial,
        "target_joints": target,
        "response": command_response,
    }
    if result_sink is not None:
        result_sink.update(result)
    while not stop_event.wait(0.1):
        client.post("/api/v1/lease/renew", json={"session_id": session})
    return result


def submit_human_command(
    client: httpx.Client,
    *,
    command_type: str,
    payload: dict[str, Any],
    session: str,
    sequence: int,
) -> dict[str, Any]:
    snapshot = client.get("/api/v1/snapshot").json()
    request = {
        "contract_version": "mobilecar-1.0",
        "run_id": snapshot["run_id"],
        "backend_generation": snapshot["backend_generation"],
        "episode_id": snapshot["episode_id"],
        "session_id": session,
        "operator_id": "g3b_special_case_operator",
        "operator_kind": "agent_ui",
        "view_source": "g3b_acceptance_harness",
        "command_id": f"g3b_special_{uuid4().hex}",
        "command_seq": sequence,
        "command_type": command_type,
        "requested_ttl_s": 2.0,
        "payload": payload,
    }
    response = client.post("/api/v1/commands", json=request)
    response.raise_for_status()
    return response.json()


def start_episode(client: httpx.Client, case_id: str) -> dict[str, Any]:
    response = client.post(
        "/api/v1/episodes/start",
        json={
            "task_language": f"G3B navigation case {case_id}",
            "operator_id": "g3b_navigation_operator",
            "operator_kind": "scripted",
            "view_source": "g3b_nav2_goal",
            "map_mode": "localization",
            "require_recording": True,
        },
    )
    response.raise_for_status()
    return response.json()


def command_values(message: Twist) -> list[float]:
    return [
        float(message.linear.x),
        float(message.linear.y),
        float(message.angular.z),
    ]


def summarize_map_to_odom(records: list[dict[str, Any]]) -> dict[str, Any]:
    unique_stamps = {
        round(item["sim_stamp_s"], 9) for item in records
    }
    by_stamp: dict[float, set[tuple[float, ...]]] = {}
    by_stamp_count: dict[float, int] = {}
    for item in records:
        stamp_key = round(item["sim_stamp_s"], 9)
        by_stamp.setdefault(stamp_key, set()).add(
            tuple(item["translation_xyz"] + item["rotation_xyzw"])
        )
        by_stamp_count[stamp_key] = by_stamp_count.get(stamp_key, 0) + 1
    elapsed = (
        records[-1]["monotonic_s"] - records[0]["monotonic_s"]
        if len(records) > 1
        else 0.0
    )
    strict_conflicts = 0
    minor_updates = 0
    max_translation_delta = 0.0
    max_rotation_delta = 0.0
    for values in by_stamp.values():
        if len(values) <= 1:
            continue
        translation_delta = 0.0
        rotation_delta = 0.0
        value_list = list(values)
        for first_index, first in enumerate(value_list):
            for second in value_list[first_index + 1 :]:
                translation_delta = max(
                    translation_delta,
                    math.dist(first[:3], second[:3]),
                )
                dot = min(
                    1.0,
                    abs(
                        sum(
                            a * b
                            for a, b in zip(first[3:], second[3:])
                        )
                    ),
                )
                rotation_delta = max(
                    rotation_delta, 2.0 * math.acos(dot)
                )
        max_translation_delta = max(
            max_translation_delta, translation_delta
        )
        max_rotation_delta = max(max_rotation_delta, rotation_delta)
        if translation_delta > 1e-5 or rotation_delta > 1e-5:
            strict_conflicts += 1
        else:
            minor_updates += 1
    return {
        "message_count": len(records),
        "unique_stamp_count": len(unique_stamps),
        "duplicate_stamp_count": len(records) - len(unique_stamps),
        "conflicting_stamp_count": strict_conflicts,
        "minor_same_stamp_update_count": minor_updates,
        "max_same_stamp_translation_delta_m": max_translation_delta,
        "max_same_stamp_rotation_delta_rad": max_rotation_delta,
        "identical_repeat_count": sum(
            max(0, by_stamp_count[stamp] - len(values))
            for stamp, values in by_stamp.items()
        ),
        "observed_rate_hz": (
            (len(records) - 1) / elapsed if elapsed > 0.0 else None
        ),
    }


def make_navigation_goal(
    x_m: float,
    y_m: float,
    yaw_rad: float,
    frame_id: str = "map",
) -> NavigateToPose.Goal:
    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = frame_id
    goal.pose.pose.position.x = float(x_m)
    goal.pose.pose.position.y = float(y_m)
    qx, qy, qz, qw = quaternion_from_yaw(float(yaw_rad))
    goal.pose.pose.orientation.x = qx
    goal.pose.pose.orientation.y = qy
    goal.pose.pose.orientation.z = qz
    goal.pose.pose.orientation.w = qw
    return goal


def send_navigation_goal(
    probe: GoalProbe,
    *,
    x_m: float,
    y_m: float,
    yaw_rad: float,
    frame_id: str = "map",
    timeout_s: float = 20.0,
    allow_rejection: bool = False,
):
    goal = make_navigation_goal(x_m, y_m, yaw_rad, frame_id)
    goal.pose.header.stamp = probe.get_clock().now().to_msg()
    future = probe.client.send_goal_async(goal, feedback_callback=probe.feedback)
    deadline = time.monotonic() + timeout_s
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.05)
    if not future.done():
        raise TimeoutError("outer goal acceptance timeout")
    handle = future.result()
    if handle is None or not handle.accepted:
        if allow_rejection:
            return None
        raise RuntimeError("outer goal rejected")
    return handle


def wait_navigation_healthy(probe: GoalProbe, timeout_s: float = 30.0) -> bool:
    deadline = time.monotonic() + timeout_s
    healthy_since = None
    while time.monotonic() < deadline:
        if probe.system_health.get("healthy") is True:
            if healthy_since is None:
                healthy_since = time.monotonic()
            if time.monotonic() - healthy_since >= 1.0:
                return True
        else:
            healthy_since = None
        time.sleep(0.1)
    return False


class GoalProbe(Node):
    def __init__(self) -> None:
        super().__init__(
            "g3b_acceptance_goal_probe",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.goal_status: dict[str, Any] = {}
        self.system_health: dict[str, Any] = {}
        self.navigation_health: dict[str, Any] = {}
        self.localization_health: dict[str, Any] = {}
        self.goal_pose_relay_status: dict[str, Any] = {}
        self.localization_fault_publisher = self.create_publisher(
            Bool, "/mobilecar/localization/fault_match_failure", 10
        )
        self.raw_commands: list[dict[str, Any]] = []
        self.governed_commands: list[dict[str, Any]] = []
        self.system_health_records: list[dict[str, Any]] = []
        self.navigation_health_records: list[dict[str, Any]] = []
        self.localization_health_records: list[dict[str, Any]] = []
        self.goal_pose_messages: list[dict[str, Any]] = []
        self.scan_summary: dict[str, Any] = {}
        self.map_to_odom_records: list[dict[str, Any]] = []
        self.create_subscription(
            String, "/mobilecar/goal_supervisor_status", self.on_goal_status, 20
        )
        self.create_subscription(
            String,
            "/mobilecar/navigation_system_health",
            self.on_system_health,
            20,
        )
        self.create_subscription(
            String,
            "/mobilecar/localization_health",
            self.on_localization_health,
            20,
        )
        self.create_subscription(
            String,
            "/mobilecar/navigation_health",
            self.on_navigation_health,
            20,
        )
        self.create_subscription(
            String,
            "/mobilecar/goal_pose_relay_status",
            self.on_goal_pose_relay_status,
            20,
        )
        self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.on_goal_pose,
            10,
        )
        self.create_subscription(
            Twist,
            "/mobilecar/nav2/cmd_vel_raw",
            lambda message: self.on_velocity("raw", message),
            20,
        )
        self.create_subscription(
            Twist,
            "/mobilecar/nav2/cmd_vel_governed",
            lambda message: self.on_velocity("governed", message),
            20,
        )
        self.create_subscription(
            LaserScan, "/scan", self.on_scan, qos_profile_sensor_data
        )
        self.create_subscription(TFMessage, "/tf", self.on_tf, 30)
        self.feedback_count = 0
        self.last_feedback: Any = None
        self._spin_stop = threading.Event()
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()

    def _spin_loop(self) -> None:
        while not self._spin_stop.wait(0.01):
            rclpy.spin_once(self, timeout_sec=0.01)

    def on_goal_status(self, message: String) -> None:
        try:
            self.goal_status = json.loads(message.data)
        except json.JSONDecodeError:
            pass

    def on_system_health(self, message: String) -> None:
        try:
            self.system_health = json.loads(message.data)
            self.system_health_records.append(
                {"monotonic_s": time.monotonic(), "value": dict(self.system_health)}
            )
            self.system_health_records = self.system_health_records[-2000:]
        except json.JSONDecodeError:
            pass

    def on_localization_health(self, message: String) -> None:
        try:
            self.localization_health = json.loads(message.data)
            self.localization_health_records.append(
                {
                    "monotonic_s": time.monotonic(),
                    "value": dict(self.localization_health),
                }
            )
            self.localization_health_records = self.localization_health_records[-2000:]
        except json.JSONDecodeError:
            pass

    def on_navigation_health(self, message: String) -> None:
        try:
            self.navigation_health = json.loads(message.data)
            self.navigation_health_records.append(
                {
                    "monotonic_s": time.monotonic(),
                    "value": dict(self.navigation_health),
                }
            )
            self.navigation_health_records = self.navigation_health_records[-2000:]
        except json.JSONDecodeError:
            pass

    def on_goal_pose_relay_status(self, message: String) -> None:
        try:
            self.goal_pose_relay_status = json.loads(message.data)
        except json.JSONDecodeError:
            pass

    def on_goal_pose(self, message: PoseStamped) -> None:
        self.goal_pose_messages.append(
            {
                "monotonic_s": time.monotonic(),
                "frame_id": message.header.frame_id,
                "x_m": float(message.pose.position.x),
                "y_m": float(message.pose.position.y),
                "orientation_xyzw": [
                    float(message.pose.orientation.x),
                    float(message.pose.orientation.y),
                    float(message.pose.orientation.z),
                    float(message.pose.orientation.w),
                ],
            }
        )
        self.goal_pose_messages = self.goal_pose_messages[-20:]

    def on_scan(self, message: LaserScan) -> None:
        finite = sum(
            1 for value in message.ranges if math.isfinite(float(value))
        )
        positive_inf = sum(
            1
            for value in message.ranges
            if float(value) == float("inf")
        )
        nan = sum(
            1 for value in message.ranges if math.isnan(float(value))
        )
        self.scan_summary = {
            "beam_count": len(message.ranges),
            "finite_count": finite,
            "positive_inf_count": positive_inf,
            "nan_count": nan,
            "frame_id": message.header.frame_id,
            "range_min_m": float(message.range_min),
            "range_max_m": float(message.range_max),
        }

    def on_tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            if (
                transform.header.frame_id == "map"
                and transform.child_frame_id == "odom"
            ):
                stamp = transform.header.stamp
                self.map_to_odom_records.append(
                    {
                        "monotonic_s": time.monotonic(),
                        "sim_stamp_s": float(stamp.sec)
                        + float(stamp.nanosec) * 1e-9,
                        "translation_xyz": [
                            float(transform.transform.translation.x),
                            float(transform.transform.translation.y),
                            float(transform.transform.translation.z),
                        ],
                        "rotation_xyzw": [
                            float(transform.transform.rotation.x),
                            float(transform.transform.rotation.y),
                            float(transform.transform.rotation.z),
                            float(transform.transform.rotation.w),
                        ],
                    }
                )
                self.map_to_odom_records = self.map_to_odom_records[-5000:]

    def on_velocity(self, kind: str, message: Twist) -> None:
        record = {
            "monotonic_s": time.monotonic(),
            "values": command_values(message),
        }
        if kind == "raw":
            self.raw_commands.append(record)
            self.raw_commands = self.raw_commands[-5000:]
        else:
            self.governed_commands.append(record)
            self.governed_commands = self.governed_commands[-5000:]

    @staticmethod
    def first_nonzero_after(records: list[dict[str, Any]], started_s: float):
        return next(
            (
                item
                for item in records
                if item["monotonic_s"] >= started_s
                and any(abs(value) > 1e-9 for value in item["values"])
            ),
            None,
        )

    @staticmethod
    def first_zero_after(records: list[dict[str, Any]], started_s: float):
        return next(
            (
                item
                for item in records
                if item["monotonic_s"] >= started_s
                and all(abs(value) <= 1e-9 for value in item["values"])
            ),
            None,
        )

    def feedback(self, message) -> None:
        self.feedback_count += 1
        self.last_feedback = message.feedback

    def set_match_fault(self, active: bool) -> None:
        message = Bool()
        message.data = bool(active)
        self.localization_fault_publisher.publish(message)

    def close(self) -> None:
        self._spin_stop.set()
        self._spin_thread.join(timeout=2.0)
        self.destroy_node()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--map-root", type=Path, required=True)
    parser.add_argument("--map-id", required=True)
    parser.add_argument("--goal-id", default="goal_smoke")
    parser.add_argument("--goal-x", type=float, required=True)
    parser.add_argument("--goal-y", type=float, required=True)
    parser.add_argument("--goal-yaw", type=float, default=0.0)
    parser.add_argument("--goal-frame", default="map")
    parser.add_argument("--arm-mode", choices=["hold", "limited"], default="hold")
    parser.add_argument(
        "--special-case",
        choices=[
            "normal",
            "cancel_takeover",
            "scan_failure",
            "localization_failure",
            "reset_epoch",
        ],
        default="normal",
    )
    parser.add_argument("--post-goal-x", type=float, default=0.18)
    parser.add_argument("--post-goal-y", type=float, default=-0.34)
    parser.add_argument("--post-goal-yaw", type=float, default=0.0)
    parser.add_argument("--fault-trigger-progress-m", type=float, default=0.10)
    parser.add_argument("--settle-window-s", type=float, default=4.0)
    parser.add_argument("--enable-localization-test-faults", action="store_true")
    parser.add_argument("--expect-goal-rejection", action="store_true")
    parser.add_argument("--report-root", type=Path, required=True)
    parser.add_argument("--domain-id", type=int, default=74)
    parser.add_argument("--goal-timeout-wall", type=float, default=180.0)
    args = parser.parse_args()
    report_root = args.report_root.resolve()
    report_root.mkdir(parents=True, exist_ok=True)
    map_root = args.map_root.resolve()
    manifest = json.loads((map_root / "manifest.json").read_text(encoding="utf-8"))
    config = load_config(args.config.resolve()).data
    env = os.environ.copy()
    env.update(
        {
            "MUJOCO_GL": "egl",
            "PYOPENGL_PLATFORM": "egl",
            "ROS_DOMAIN_ID": str(args.domain_id),
            "RMW_IMPLEMENTATION": env.get("G3A_ROS_RMW", "rmw_fastrtps_cpp"),
            "PYTHONPATH": str(ROOT / "src")
            + (":" + env["PYTHONPATH"] if env.get("PYTHONPATH") else ""),
        }
    )
    os.environ.update(
        {
            "MUJOCO_GL": env["MUJOCO_GL"],
            "PYOPENGL_PLATFORM": env["PYOPENGL_PLATFORM"],
            "ROS_DOMAIN_ID": env["ROS_DOMAIN_ID"],
            "RMW_IMPLEMENTATION": env["RMW_IMPLEMENTATION"],
            "PYTHONPATH": env["PYTHONPATH"],
        }
    )
    port = free_port()
    report: dict[str, Any] = {
        "report_version": "g3b-single-goal-1",
        "case_id": args.goal_id,
        "map_id": args.map_id,
        "generated_wall_utc": utc_now(),
        "execution_location": "server_only",
        "goal": {
            "x_m": args.goal_x,
            "y_m": args.goal_y,
            "yaw_rad": args.goal_yaw,
        },
        "arm_mode": args.arm_mode,
        "special_case": args.special_case,
        "success": False,
    }
    app = Process(
        [
            str(PYTHON),
            "-m",
            "mobilecar",
            "--config",
            str(args.config.resolve()),
            "--host",
            "127.0.0.1",
            "--port",
            str(port),
        ],
        report_root / "mobilecar_server.log",
        env,
    )
    supervisor = slam = nav2 = None
    arm_thread = None
    arm_stop = threading.Event()
    arm_started = False
    goal_probe = None
    rclpy.init()
    try:
        base_url = f"http://127.0.0.1:{port}"
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            if not app.alive():
                raise RuntimeError("MuJoCo application exited")
            try:
                if httpx.get(f"{base_url}/api/v1/health", timeout=1.0).status_code == 200:
                    break
            except Exception:
                pass
            time.sleep(0.1)
        else:
            raise RuntimeError("MuJoCo application startup timeout")
        with httpx.Client(base_url=base_url, timeout=30.0) as client:
            episode = start_episode(client, args.goal_id)
            if not episode.get("accepted"):
                raise RuntimeError(f"Episode start rejected: {episode}")
            initial_snapshot = client.get("/api/v1/snapshot").json()
            report["episode_id"] = episode["episode_id"]
            report["run_id"] = initial_snapshot["run_id"]
            report["navigation_epoch"] = initial_snapshot["diagnostics"]["navigation"][
                "navigation_epoch"
            ]
            supervisor = Process(
                [
                    "ros2",
                    "launch",
                    "mobilecar_goal_supervisor",
                    "g3b_supervisor.launch.py",
                    f"map_id:={args.map_id}",
                    f"map_manifest:={map_root / 'manifest.json'}",
                    f"runtime_config:={args.config.resolve()}",
                    "socket_path:=../../var/ipc/g3b_navigation.sock",
                ],
                report_root / "g3b_supervisor.log",
                env,
            )
            slam = Process(
                [
                    "ros2",
                    "launch",
                    "mobilecar_mapping_localization",
                    "localization.launch.py",
                    f"map_file_name:={map_root / 'posegraph'}",
                    "use_sim_time:=true",
                    "enable_test_faults:="
                    + ("true" if args.enable_localization_test_faults else "false"),
                ],
                report_root / "slam_toolbox_localization.log",
                env,
            )
            nav2 = Process(
                [
                    "ros2",
                    "launch",
                    "mobilecar_nav_bringup",
                    "navigation_g3b.launch.py",
                    "use_sim_time:=true",
                ],
                report_root / "nav2.log",
                env,
            )
            topics, actions = wait_graph(
                env,
                topics={
                    "/clock",
                    "/scan",
                    "/odom",
                    "/map",
                    "/pose",
                    "/mobilecar/localization_health",
                    "/mobilecar/navigation_system_health",
                },
                actions={"/navigate_to_pose", "/nav2/navigate_to_pose"},
                timeout_s=60.0,
            )
            report["topics"] = sorted(topics)
            report["actions"] = sorted(actions)
            report["action_info"] = {
                "/navigate_to_pose": ros_command_output(
                    ["ros2", "action", "info", "/navigate_to_pose"], env
                ),
                "/nav2/navigate_to_pose": ros_command_output(
                    ["ros2", "action", "info", "/nav2/navigate_to_pose"], env
                ),
            }
            if not {
                "/clock",
                "/scan",
                "/odom",
                "/map",
                "/pose",
            }.issubset(topics):
                raise RuntimeError(f"localization graph incomplete: {sorted(topics)}")
            if not {"/navigate_to_pose", "/nav2/navigate_to_pose"}.issubset(actions):
                raise RuntimeError(f"navigation actions unavailable: {sorted(actions)}")
            goal_probe = GoalProbe()
            if not goal_probe.client.wait_for_server(timeout_sec=20.0):
                raise RuntimeError("outer navigate_to_pose action server unavailable")
            wait_navigation_healthy(goal_probe)
            report["pre_goal_system_health"] = goal_probe.system_health
            report["pre_goal_localization_health"] = goal_probe.localization_health
            if goal_probe.system_health.get("healthy") is not True:
                raise RuntimeError("navigation system did not become healthy")

            goal_handle = send_navigation_goal(
                goal_probe,
                x_m=args.goal_x,
                y_m=args.goal_y,
                yaw_rad=args.goal_yaw,
                frame_id=args.goal_frame,
                allow_rejection=args.expect_goal_rejection,
            )
            report["outer_goal_accepted"] = bool(goal_handle and goal_handle.accepted)
            if (
                goal_handle is None
                and args.expect_goal_rejection
            ):
                rejected_snapshot = client.get("/api/v1/snapshot").json()
                time.sleep(1.0)
                settled_snapshot = client.get("/api/v1/snapshot").json()
                displacement = math.dist(
                    rejected_snapshot["base"]["position_world_sim_m"][:2],
                    settled_snapshot["base"]["position_world_sim_m"][:2],
                )
                report["expected_rejection_observed"] = True
                report["goal_supervisor_status"] = goal_probe.goal_status
                report["final_snapshot"] = settled_snapshot
                report["position_error_m"] = None
                report["yaw_error_deg"] = None
                report["settled_duration_s"] = 0.0
                report["unexpected_contacts"] = settled_snapshot.get(
                    "contact_summary", {}
                ).get("unexpected", [])
                report["gates"] = {
                    "outer_goal_rejected": True,
                    "no_physical_motion": displacement <= 0.005,
                    "no_backend_authorization": settled_snapshot["diagnostics"][
                        "navigation"
                    ]["authorized_g3b_goal"]
                    is None,
                    "no_unexpected_contacts": not report["unexpected_contacts"],
                }
                report["success"] = all(report["gates"].values())
                stop_response = client.post(
                    "/api/v1/episodes/stop",
                    json={"reason": "G3B_EXPECTED_REJECTION_COMPLETE"},
                )
                stop_response.raise_for_status()
                report["stop_response"] = stop_response.json()
                raise ExpectedGoalRejection()
            if not goal_handle or not goal_handle.accepted:
                raise RuntimeError("outer goal rejected")
            authorization_snapshot = client.get("/api/v1/snapshot").json()
            probe_goal = goal_probe.goal_status
            stale_probe = {
                "schema_version": 2,
                "interface_profile": "g3b-1.0",
                "message_type": "cmd_vel",
                "source": "nav2_goal",
                "request_id": f"receipt_rejection_{uuid4().hex}",
                "goal_id": str(probe_goal.get("goal_id") or "stale_goal"),
                "goal_generation": max(
                    1, int(probe_goal.get("goal_generation") or 1)
                ),
                "map_id": args.map_id,
                "run_id": authorization_snapshot["run_id"],
                "episode_id": authorization_snapshot["episode_id"],
                "backend_generation": authorization_snapshot[
                    "backend_generation"
                ],
                "navigation_epoch": authorization_snapshot["diagnostics"][
                    "navigation"
                ]["navigation_epoch"],
                "session_id": "stale_receipt_probe_session",
                "lease_ttl_s": 0.35,
                "vx_m_s": 0.01,
                "vy_m_s": 0.0,
                "wz_rad_s": 0.0,
                "reason": "formal_rejection_receipt_probe",
            }
            stale_response = client.post(
                "/api/v1/g3b/control", json=stale_probe
            )
            stale_response.raise_for_status()
            report["backend_rejection_receipt"] = stale_response.json()
            if args.arm_mode == "limited":
                arm_stop.clear()
                arm_result: dict[str, Any] = {}

                def run_arm() -> None:
                    arm_activity(
                        client,
                        arm_stop,
                        session=f"g3b_arm_{uuid4().hex}",
                        sequence=1,
                        result_sink=arm_result,
                    )

                if args.special_case == "cancel_takeover":
                    arm_thread = threading.Thread(target=run_arm, daemon=True)
                    arm_thread.start()
                    arm_started = True
            action_started = time.monotonic()
            result_future = goal_handle.get_result_async()
            start_snapshot = client.get("/api/v1/snapshot").json()
            start_position = list(start_snapshot["base"]["position_world_sim_m"][:2])
            special: dict[str, Any] = {
                "triggered": False,
                "injection_monotonic_s": None,
                "injection_position_m": None,
                "cancel_requested_monotonic_s": None,
                "detection_monotonic_s": None,
                "governed_zero_monotonic_s": None,
                "physical_stop_monotonic_s": None,
                "maximum_braking_displacement_m": 0.0,
                "fault_cleared_monotonic_s": None,
                "post_recovery_position_m": None,
            }
            previous_speed = 0.0
            while (
                not result_future.done()
                and time.monotonic() - action_started < args.goal_timeout_wall
                and app.alive()
            ):
                current = client.get("/api/v1/snapshot").json()
                position = current["base"]["position_world_sim_m"][:2]
                progress = math.dist(position, start_position)
                if (
                    args.special_case != "normal"
                    and not special["triggered"]
                    and time.monotonic() - action_started >= 0.5
                    and progress >= args.fault_trigger_progress_m
                ):
                    special["triggered"] = True
                    special["injection_monotonic_s"] = time.monotonic()
                    special["injection_position_m"] = position
                    if args.special_case in {"cancel_takeover", "reset_epoch"}:
                        goal_handle.cancel_goal_async()
                        special["cancel_requested_monotonic_s"] = time.monotonic()
                    elif args.special_case == "scan_failure":
                        response = client.post(
                            "/api/v1/navigation/fault",
                            json={"fault": "scan", "active": True},
                        )
                        response.raise_for_status()
                        special["fault_response"] = response.json()
                    elif args.special_case == "localization_failure":
                        goal_probe.set_match_fault(True)
                if special["triggered"]:
                    injection_position = special["injection_position_m"]
                    special["maximum_braking_displacement_m"] = max(
                        special["maximum_braking_displacement_m"],
                        math.dist(position, injection_position),
                    )
                    speed = math.hypot(
                        current["base"]["vx_m_s"],
                        current["base"]["vy_m_s"],
                    )
                    previous_speed = max(previous_speed, speed)
                    if (
                        special["detection_monotonic_s"] is None
                        and (
                            goal_probe.system_health.get("healthy") is False
                            or goal_probe.localization_health.get(
                                "stop_required"
                            )
                            is True
                            or goal_probe.navigation_health.get(
                                "stop_required"
                            )
                            is True
                        )
                    ):
                        special["detection_monotonic_s"] = time.monotonic()
                    if special["governed_zero_monotonic_s"] is None:
                        zero_record = GoalProbe.first_zero_after(
                            goal_probe.governed_commands,
                            special["injection_monotonic_s"],
                        )
                        if zero_record is not None:
                            special["governed_zero_monotonic_s"] = zero_record[
                                "monotonic_s"
                            ]
                    if (
                        special["physical_stop_monotonic_s"] is None
                        and speed <= 0.01
                        and abs(current["base"]["wz_rad_s"]) <= 0.03
                    ):
                        special["physical_stop_monotonic_s"] = time.monotonic()
                    if (
                        args.special_case in {"scan_failure", "localization_failure"}
                        and special["physical_stop_monotonic_s"] is not None
                        and special["fault_cleared_monotonic_s"] is None
                    ):
                        if args.special_case == "scan_failure":
                            response = client.post(
                                "/api/v1/navigation/fault",
                                json={"fault": "scan", "active": False},
                            )
                            response.raise_for_status()
                        else:
                            goal_probe.set_match_fault(False)
                        special["fault_cleared_monotonic_s"] = time.monotonic()
                time.sleep(0.1)
            action_duration = time.monotonic() - action_started
            timed_out = not result_future.done()
            if timed_out:
                goal_handle.cancel_goal_async()
            action_result = result_future.result() if result_future.done() else None
            report["action_duration_wall_s"] = action_duration
            report["action_timed_out"] = timed_out
            report["action_status"] = (
                int(action_result.status) if action_result is not None else None
            )
            report["action_succeeded"] = (
                action_result is not None and int(action_result.status) == 4
            )
            report["feedback_count"] = goal_probe.feedback_count
            if args.special_case == "normal":
                if args.arm_mode == "limited" and not arm_started:
                    arm_thread = threading.Thread(target=run_arm, daemon=True)
                    arm_thread.start()
                    arm_started = True
                    arm_deadline = time.monotonic() + 10.0
                    while (
                        not arm_result
                        and time.monotonic() < arm_deadline
                        and app.alive()
                    ):
                        time.sleep(0.1)
                    time.sleep(1.0)
                    arm_stop.set()
                    arm_thread.join(timeout=2.0)
                report["first_action_status"] = report["action_status"]
                time.sleep(0.4)
                correction_handle = send_navigation_goal(
                    goal_probe,
                    x_m=args.goal_x,
                    y_m=args.goal_y,
                    yaw_rad=args.goal_yaw,
                )
                correction_future = correction_handle.get_result_async()
                correction_deadline = time.monotonic() + 90.0
                while (
                    not correction_future.done()
                    and time.monotonic() < correction_deadline
                    and app.alive()
                ):
                    time.sleep(0.1)
                if not correction_future.done():
                    correction_handle.cancel_goal_async()
                correction_status = (
                    int(correction_future.result().status)
                    if correction_future.done()
                    else None
                )
                report["final_alignment_action_status"] = correction_status
                report["action_status"] = correction_status
                report["action_succeeded"] = correction_status == 4
            if special["triggered"]:
                stop_deadline = time.monotonic() + 3.0
                while (
                    special["physical_stop_monotonic_s"] is None
                    and time.monotonic() < stop_deadline
                ):
                    current = client.get("/api/v1/snapshot").json()
                    position = current["base"]["position_world_sim_m"][:2]
                    special["maximum_braking_displacement_m"] = max(
                        special["maximum_braking_displacement_m"],
                        math.dist(position, special["injection_position_m"]),
                    )
                    speed = math.hypot(
                        current["base"]["vx_m_s"],
                        current["base"]["vy_m_s"],
                    )
                    if speed <= 0.01 and abs(current["base"]["wz_rad_s"]) <= 0.03:
                        special["physical_stop_monotonic_s"] = time.monotonic()
                    time.sleep(0.1)
                if special["governed_zero_monotonic_s"] is None:
                    zero_record = GoalProbe.first_zero_after(
                        goal_probe.governed_commands,
                        special["injection_monotonic_s"],
                    )
                    if zero_record is not None:
                        special["governed_zero_monotonic_s"] = zero_record[
                            "monotonic_s"
                        ]
                if special["detection_monotonic_s"] is None:
                    detected = next(
                        (
                            item
                            for item in goal_probe.system_health_records
                            if item["monotonic_s"]
                            >= special["injection_monotonic_s"]
                            and item["value"].get("healthy") is False
                        ),
                        None,
                    )
                    if detected is not None:
                        special["detection_monotonic_s"] = detected[
                            "monotonic_s"
                        ]
                if special["detection_monotonic_s"] is None:
                    source_records = (
                        goal_probe.localization_health_records
                        + goal_probe.navigation_health_records
                    )
                    detected = next(
                        (
                            item
                            for item in source_records
                            if item["monotonic_s"]
                            >= special["injection_monotonic_s"]
                            and (
                                item["value"].get("stop_required") is True
                                or item["value"].get("status")
                                in {"STALE", "LOST", "FAILED", "STOPPED"}
                            )
                        ),
                        None,
                    )
                    if detected is not None:
                        special["detection_monotonic_s"] = detected[
                            "monotonic_s"
                        ]
                if special["detection_monotonic_s"] is None:
                    transition = next(
                        (
                            item
                            for item in goal_probe.goal_status.get(
                                "health_transitions", []
                            )
                            if item.get("monotonic_s", -math.inf)
                            >= special["injection_monotonic_s"]
                            and item.get("status") == "STOPPED"
                        ),
                        None,
                    )
                    if transition is not None:
                        special["detection_monotonic_s"] = transition[
                            "monotonic_s"
                        ]
                if (
                    args.special_case in {"scan_failure", "localization_failure"}
                    and special["fault_cleared_monotonic_s"] is None
                ):
                    if args.special_case == "scan_failure":
                        response = client.post(
                            "/api/v1/navigation/fault",
                            json={"fault": "scan", "active": False},
                        )
                        response.raise_for_status()
                    else:
                        goal_probe.set_match_fault(False)
                    special["fault_cleared_monotonic_s"] = time.monotonic()
                if args.special_case in {"scan_failure", "localization_failure"}:
                    recovery_position = client.get("/api/v1/snapshot").json()[
                        "base"
                    ]["position_world_sim_m"][:2]
                    time.sleep(2.0)
                    recovered = client.get("/api/v1/snapshot").json()
                    special["post_recovery_position_m"] = recovered["base"][
                        "position_world_sim_m"
                    ][:2]
                    special["post_recovery_displacement_m"] = math.dist(
                        recovery_position, special["post_recovery_position_m"]
                    )
                    special["post_recovery_requested_zero"] = all(
                        abs(recovered["base"][key]) <= 1e-9
                        for key in (
                            "requested_vx_m_s",
                            "requested_vy_m_s",
                            "requested_wz_rad_s",
                        )
                    )
                for key in (
                    "detection_latency_s",
                    "governed_zero_latency_s",
                    "physical_stop_latency_s",
                ):
                    special.pop(key, None)
                injection_s = special["injection_monotonic_s"]
                if injection_s is not None:
                    for name, key in (
                        ("detection_latency_s", "detection_monotonic_s"),
                        ("governed_zero_latency_s", "governed_zero_monotonic_s"),
                        ("physical_stop_latency_s", "physical_stop_monotonic_s"),
                    ):
                        if special[key] is not None:
                            special[name] = special[key] - injection_s
                if (
                    special.get("detection_latency_s") is not None
                    and special.get("governed_zero_latency_s") is not None
                ):
                    special["detection_to_zero_s"] = max(
                        0.0,
                        special["governed_zero_latency_s"]
                        - special["detection_latency_s"],
                    )
                if (
                    special.get("detection_latency_s") is not None
                    and special.get("physical_stop_latency_s") is not None
                ):
                    special["detection_to_physical_stop_s"] = max(
                        0.0,
                        special["physical_stop_latency_s"]
                        - special["detection_latency_s"],
                    )
                gate_close = None
                gate_deadline = time.monotonic() + 1.0
                while time.monotonic() < gate_deadline:
                    gate_close = next(
                        (
                            item
                            for item in goal_probe.goal_status.get(
                                "gate_transitions", []
                            )
                            if item.get("monotonic_s", -math.inf)
                            >= special["injection_monotonic_s"]
                        ),
                        None,
                    )
                    if gate_close is not None:
                        break
                    time.sleep(0.05)
                if gate_close is not None:
                    closed_s = float(gate_close["monotonic_s"])
                    if special["detection_monotonic_s"] is None:
                        special["detection_monotonic_s"] = closed_s
                    if (
                        special["governed_zero_monotonic_s"] is None
                        and gate_close.get("zero_publish_monotonic_s")
                        is not None
                    ):
                        special["governed_zero_monotonic_s"] = float(
                            gate_close["zero_publish_monotonic_s"]
                        )
                    special["gate_closed_monotonic_s"] = closed_s
                    special["raw_commands_after_gate_close"] = sum(
                        1
                        for item in goal_probe.raw_commands
                        if item["monotonic_s"] >= closed_s
                    )
                    special["governed_nonzero_after_gate_close"] = sum(
                        1
                        for item in goal_probe.governed_commands
                        if item["monotonic_s"] >= closed_s
                        and any(abs(value) > 1e-9 for value in item["values"])
                    )
                    special["governed_zero_after_gate_close"] = sum(
                        1
                        for item in goal_probe.governed_commands
                        if item["monotonic_s"] >= closed_s
                        and all(abs(value) <= 1e-9 for value in item["values"])
                    )
                    special["late_accepted_after_gate_close_count"] = int(
                        goal_probe.goal_status.get(
                            "late_accepted_after_gate_close_count", 0
                        )
                    )
                    for name, key in (
                        ("detection_latency_s", "detection_monotonic_s"),
                        (
                            "governed_zero_latency_s",
                            "governed_zero_monotonic_s",
                        ),
                        (
                            "physical_stop_latency_s",
                            "physical_stop_monotonic_s",
                        ),
                    ):
                        if special.get(key) is not None:
                            special[name] = (
                                special[key]
                                - special["injection_monotonic_s"]
                            )
                    if (
                        special.get("detection_latency_s") is not None
                        and special.get("governed_zero_latency_s")
                        is not None
                    ):
                        special["detection_to_zero_s"] = max(
                            0.0,
                            special["governed_zero_latency_s"]
                            - special["detection_latency_s"],
                        )
                    if (
                        special.get("detection_latency_s") is not None
                        and special.get("physical_stop_latency_s")
                        is not None
                    ):
                        special["detection_to_physical_stop_s"] = max(
                            0.0,
                            special["physical_stop_latency_s"]
                            - special["detection_latency_s"],
                        )
                report["special_case_evidence"] = special
            settle_started = time.monotonic()
            settled_samples = []
            while time.monotonic() - settle_started < args.settle_window_s:
                current = client.get("/api/v1/snapshot").json()
                velocity = current["base"]
                settled = (
                    math.hypot(velocity["vx_m_s"], velocity["vy_m_s"]) <= 0.01
                    and abs(velocity["wz_rad_s"]) <= 0.03
                )
                settled_samples.append((time.monotonic(), settled, velocity))
                if (
                    len(settled_samples) >= 6
                    and all(item[1] for item in settled_samples[-6:])
                ):
                    break
                time.sleep(0.1)
            first_action_status = report.get(
                "first_action_status", report["action_status"]
            )
            report["first_action_status"] = first_action_status
            arrival_x = args.goal_x
            arrival_y = args.goal_y
            arrival_yaw = args.goal_yaw
            second_action_status = None
            if args.special_case == "cancel_takeover":
                manual_session = f"manual_takeover_{uuid4().hex}"
                before_takeover = client.get("/api/v1/snapshot").json()
                takeover_start_position = before_takeover["base"][
                    "position_world_sim_m"
                ][:2]
                takeover = None
                for sequence in range(1, 13):
                    takeover = submit_human_command(
                        client,
                        command_type="set_base_twist",
                        payload={
                            "frame_id": "base_link",
                            "vx_m_s": 0.02,
                            "vy_m_s": 0.0,
                            "wz_rad_s": 0.0,
                        },
                        session=manual_session,
                        sequence=sequence,
                    )
                    time.sleep(0.15)
                after_takeover = client.get("/api/v1/snapshot").json()
                takeover_displacement = math.dist(
                    takeover_start_position,
                    after_takeover["base"]["position_world_sim_m"][:2],
                )
                zero_takeover = submit_human_command(
                    client,
                    command_type="zero_base",
                    payload={"reason": "manual_takeover_release"},
                    session=manual_session,
                    sequence=13,
                )
                release_position = after_takeover["base"]["position_world_sim_m"][:2]
                time.sleep(1.2)
                after_release = client.get("/api/v1/snapshot").json()
                no_resume_displacement = math.dist(
                    release_position,
                    after_release["base"]["position_world_sim_m"][:2],
                )
                arm_preserved = (
                    after_release["arm_owner_session_id"]
                    == before_takeover["arm_owner_session_id"]
                    if args.arm_mode == "limited"
                    else True
                )
                arm_target_preserved = True
                if args.arm_mode == "limited":
                    requested_arm_target = arm_result.get("target_joints", [])
                    observed_arm_target = after_release["arm"][
                        "user_joint_targets_rad"
                    ]
                    arm_target_preserved = (
                        len(requested_arm_target) == len(observed_arm_target)
                        and max(
                            abs(a - b)
                            for a, b in zip(
                                requested_arm_target, observed_arm_target
                            )
                        )
                        <= 0.01
                    )
                special["manual_takeover"] = {
                    "command": takeover,
                    "zero_command": zero_takeover,
                    "displacement_m": takeover_displacement,
                    "after_release_displacement_m": no_resume_displacement,
                    "backend_authorized_after_cancel": after_release[
                        "diagnostics"
                    ]["navigation"]["authorized_g3b_goal"],
                    "arm_owner_before": before_takeover["arm_owner_session_id"],
                    "arm_owner_after": after_release["arm_owner_session_id"],
                    "arm_preserved": arm_preserved,
                    "arm_target_preserved": arm_target_preserved,
                }
                first_goal_id = (
                    goal_probe.goal_status.get("last_terminal_goal") or {}
                ).get("goal_id")
                report["first_goal_id"] = first_goal_id
                wait_navigation_healthy(goal_probe)
                time.sleep(0.4)
                second_handle = send_navigation_goal(
                    goal_probe,
                    x_m=args.post_goal_x,
                    y_m=args.post_goal_y,
                    yaw_rad=args.post_goal_yaw,
                )
                second_future = second_handle.get_result_async()
                second_started = time.monotonic()
                receipt_baseline = int(
                    goal_probe.goal_status.get(
                        "accepted_velocity_receipt_count", 0
                    )
                )
                second_goal_started = False
                second_goal_motion_observed = False
                while (
                    time.monotonic() - second_started < 12.0
                    and app.alive()
                ):
                    status = goal_probe.goal_status
                    terminal = status.get("last_terminal_goal") or {}
                    current_goal_id = status.get("goal_id")
                    if (
                        current_goal_id
                        and current_goal_id != report.get("first_goal_id")
                        and status.get("action_accepted") is True
                    ):
                        second_goal_started = True
                    if (
                        terminal.get("state")
                        in {"SUCCEEDED", "ABORTED", "CANCEL_REQUESTED"}
                        and terminal.get("requested_pose") == {
                            "frame_id": "map",
                            "x_m": float(args.post_goal_x),
                            "y_m": float(args.post_goal_y),
                            "yaw_rad": float(args.post_goal_yaw),
                        }
                    ):
                        second_goal_started = True
                    if (
                        int(
                            status.get(
                                "accepted_velocity_receipt_count", 0
                            )
                        )
                        > receipt_baseline
                        or GoalProbe.first_nonzero_after(
                            goal_probe.governed_commands, second_started
                        )
                        is not None
                    ):
                        second_goal_motion_observed = True
                    if second_goal_started and (
                        second_goal_motion_observed
                        or second_future.done()
                    ):
                        break
                    time.sleep(0.1)
                if not second_future.done():
                    second_handle.cancel_goal_async()
                    cancel_deadline = time.monotonic() + 5.0
                    while (
                        not second_future.done()
                        and time.monotonic() < cancel_deadline
                    ):
                        time.sleep(0.1)
                if second_future.done():
                    second_action_status = int(second_future.result().status)
                special["second_goal_run"] = {
                    "started": second_goal_started,
                    "motion_or_terminal_observed": (
                        second_goal_motion_observed or second_future.done()
                    ),
                    "action_status_after_probe": second_action_status,
                    "requested_pose": {
                        "x_m": args.post_goal_x,
                        "y_m": args.post_goal_y,
                        "yaw_rad": args.post_goal_yaw,
                    },
                }
                report["second_action_status"] = second_action_status
                report["second_goal"] = {
                    "x_m": args.post_goal_x,
                    "y_m": args.post_goal_y,
                    "yaw_rad": args.post_goal_yaw,
                }
                report["action_status"] = second_action_status
                report["action_succeeded"] = second_action_status == 4
                report["special_case_evidence"] = special
            elif args.special_case == "reset_epoch":
                old_episode_id = report["episode_id"]
                old_epoch = report["navigation_epoch"]
                old_run_id = report["run_id"]
                old_goal = goal_probe.goal_status
                stop_recording = client.post(
                    "/api/v1/episodes/stop",
                    json={"reason": "G3B_RESET_CASE_BOUNDARY"},
                )
                stop_recording.raise_for_status()
                reset_response = submit_human_command(
                    client,
                    command_type="reset",
                    payload={
                        "confirm": True,
                        "boundary_reason": "G3B_FULL_STACK_RESET",
                    },
                    session=f"reset_operator_{uuid4().hex}",
                    sequence=1,
                )
                reset_snapshot = client.get("/api/v1/snapshot").json()
                new_episode = start_episode(client, f"{args.goal_id}_post_reset")
                if not new_episode.get("accepted"):
                    raise RuntimeError(
                        f"post-reset Episode start rejected: {new_episode}"
                    )
                post_reset_snapshot = client.get("/api/v1/snapshot").json()
                old_nav2_exit = nav2.close()
                old_slam_exit = slam.close()
                old_supervisor_exit = supervisor.close()
                nav2 = Process(
                    [
                        "ros2",
                        "launch",
                        "mobilecar_nav_bringup",
                        "navigation_g3b.launch.py",
                        "use_sim_time:=true",
                    ],
                    report_root / "nav2_post_reset.log",
                    env,
                )
                slam = Process(
                    [
                        "ros2",
                        "launch",
                        "mobilecar_mapping_localization",
                        "localization.launch.py",
                        f"map_file_name:={map_root / 'posegraph'}",
                        "use_sim_time:=true",
                        "enable_test_faults:=false",
                    ],
                    report_root / "slam_toolbox_post_reset.log",
                    env,
                )
                supervisor = Process(
                    [
                        "ros2",
                        "launch",
                        "mobilecar_goal_supervisor",
                        "g3b_supervisor.launch.py",
                        f"map_id:={args.map_id}",
                        f"map_manifest:={map_root / 'manifest.json'}",
                        f"runtime_config:={args.config.resolve()}",
                        "socket_path:=../../var/ipc/g3b_navigation.sock",
                    ],
                    report_root / "g3b_supervisor_post_reset.log",
                    env,
                )
                post_topics, post_actions = wait_graph(
                    env,
                    topics={
                        "/clock",
                        "/scan",
                        "/odom",
                        "/map",
                        "/pose",
                        "/mobilecar/navigation_system_health",
                    },
                    actions={"/navigate_to_pose", "/nav2/navigate_to_pose"},
                    timeout_s=60.0,
                )
                goal_probe.system_health = {}
                goal_probe.localization_health = {}
                if not wait_navigation_healthy(goal_probe):
                    raise RuntimeError("post-reset navigation did not become healthy")
                stale_payload = {
                    "schema_version": 2,
                    "interface_profile": "g3b-1.0",
                    "message_type": "cmd_vel",
                    "source": "nav2_goal",
                    "request_id": f"stale_reset_{uuid4().hex}",
                    "goal_id": old_goal.get("goal_id") or "old_goal",
                    "goal_generation": max(
                        1, int(old_goal.get("goal_generation") or 1)
                    ),
                    "map_id": args.map_id,
                    "run_id": old_run_id,
                    "episode_id": old_episode_id,
                    "backend_generation": post_reset_snapshot["backend_generation"],
                    "navigation_epoch": old_epoch,
                    "session_id": old_goal.get("session_id") or "old_session",
                    "lease_ttl_s": 0.35,
                    "vx_m_s": 0.02,
                    "vy_m_s": 0.0,
                    "wz_rad_s": 0.0,
                    "reason": "post_reset_stale_goal_probe",
                }
                stale_response = client.post(
                    "/api/v1/g3b/control", json=stale_payload
                )
                stale_response.raise_for_status()
                stale_receipt = stale_response.json()
                second_handle = send_navigation_goal(
                    goal_probe,
                    x_m=args.post_goal_x,
                    y_m=args.post_goal_y,
                    yaw_rad=args.post_goal_yaw,
                )
                second_future = second_handle.get_result_async()
                second_started = time.monotonic()
                while (
                    not second_future.done()
                    and time.monotonic() - second_started < args.goal_timeout_wall
                    and app.alive()
                ):
                    time.sleep(0.1)
                if second_future.done():
                    second_action_status = int(second_future.result().status)
                special.update(
                    {
                        "old_episode_id": old_episode_id,
                        "new_episode_id": post_reset_snapshot["episode_id"],
                        "old_navigation_epoch": old_epoch,
                        "new_navigation_epoch": post_reset_snapshot["diagnostics"][
                            "navigation"
                        ]["navigation_epoch"],
                        "reset_response": reset_response,
                        "stop_recording": stop_recording.json(),
                        "new_episode_response": new_episode,
                        "old_process_exit_codes": {
                            "nav2": old_nav2_exit,
                            "slam": old_slam_exit,
                            "supervisor": old_supervisor_exit,
                        },
                        "post_reset_topics": sorted(post_topics),
                        "post_reset_actions": sorted(post_actions),
                        "stale_control_receipt": stale_receipt,
                    }
                )
                report["second_action_status"] = second_action_status
                report["second_goal"] = {
                    "x_m": args.post_goal_x,
                    "y_m": args.post_goal_y,
                    "yaw_rad": args.post_goal_yaw,
                }
                report["episode_id"] = post_reset_snapshot["episode_id"]
                report["navigation_epoch"] = post_reset_snapshot["diagnostics"][
                    "navigation"
                ]["navigation_epoch"]
                arrival_x = args.post_goal_x
                arrival_y = args.post_goal_y
                arrival_yaw = args.post_goal_yaw
                report["action_status"] = second_action_status
                report["action_succeeded"] = second_action_status == 4
                report["special_case_evidence"] = special
            if args.special_case in {"cancel_takeover", "reset_epoch"}:
                settle_started = time.monotonic()
                settled_samples = []
                while (
                    time.monotonic() - settle_started
                    < args.settle_window_s
                ):
                    current = client.get("/api/v1/snapshot").json()
                    velocity = current["base"]
                    settled = (
                        math.hypot(
                            velocity["vx_m_s"], velocity["vy_m_s"]
                        )
                        <= 0.01
                        and abs(velocity["wz_rad_s"]) <= 0.03
                    )
                    settled_samples.append(
                        (time.monotonic(), settled, velocity)
                    )
                    if (
                        len(settled_samples) >= 6
                        and all(
                            item[1] for item in settled_samples[-6:]
                        )
                    ):
                        break
                    time.sleep(0.1)
            final_snapshot = client.get("/api/v1/snapshot").json()
            final_yaw = yaw_from_quaternion(
                final_snapshot["base"]["orientation_world_sim_xyzw"]
            )
            position_error = math.hypot(
                final_snapshot["base"]["position_world_sim_m"][0] - arrival_x,
                final_snapshot["base"]["position_world_sim_m"][1] - arrival_y,
            )
            yaw_error = angle_error(final_yaw, arrival_yaw)
            sustained_samples = 0
            for _timestamp, settled, _velocity in reversed(settled_samples):
                if not settled:
                    break
                sustained_samples += 1
            settled_duration = sustained_samples * 0.1
            collisions = final_snapshot.get("contact_summary", {}).get(
                "unexpected", []
            )
            report["final_snapshot"] = final_snapshot
            report["position_error_m"] = position_error
            report["yaw_error_deg"] = math.degrees(yaw_error)
            report["settled_duration_s"] = settled_duration
            report["unexpected_contacts"] = collisions
            report["goal_supervisor_status"] = goal_probe.goal_status
            report["navigation_system_health"] = goal_probe.system_health
            report["localization_health"] = goal_probe.localization_health
            report["scan_policy_evidence"] = {
                "actual_scan": goal_probe.scan_summary,
                "nav2_inf_is_valid": False,
                "invalid_or_self_occluded_is_free": False,
                "positive_inf_policy": "conservative_no_clear",
            }
            map_tf_records = goal_probe.map_to_odom_records
            report["map_to_odom_runtime"] = {
                **summarize_map_to_odom(map_tf_records),
                "publisher_identity_scope": (
                    "runtime frame traffic plus static source ownership; "
                    "ROS /tf does not carry publisher identity"
                ),
            }
            if args.arm_mode == "limited":
                arm_stop.set()
                if arm_thread is not None:
                    arm_thread.join(timeout=2.0)
                report["arm_activity"] = arm_result
                arm_delta = max(
                    abs(a - b)
                    for a, b in zip(
                        final_snapshot["arm"]["user_joint_positions_rad"],
                        arm_result.get("initial_joints", [0.0] * 5),
                    )
                )
                report["arm_observed_delta_rad"] = arm_delta
            gates = {
                "action_succeeded": report["action_succeeded"],
                "first_action_succeeded": (
                    args.special_case != "normal"
                    or first_action_status == 4
                ),
                "backend_rejection_receipt": report.get(
                    "backend_rejection_receipt", {}
                ).get("accepted")
                is False,
                "physical_position": position_error <= 0.10,
                "physical_yaw": math.degrees(yaw_error) <= 10.0,
                "settled": settled_duration >= 0.60,
                "no_unexpected_contacts": not collisions,
            }
            if args.arm_mode == "limited":
                gates["arm_activity"] = report.get("arm_observed_delta_rad", 0.0) >= 0.01
            if args.special_case in {"scan_failure", "localization_failure"}:
                health_records = [
                    item
                    for item in goal_probe.system_health_records
                    if special.get("injection_monotonic_s") is not None
                    and item["monotonic_s"] >= special["injection_monotonic_s"]
                ]
                localization_records = [
                    item
                    for item in goal_probe.localization_health_records
                    if special.get("injection_monotonic_s") is not None
                    and item["monotonic_s"] >= special["injection_monotonic_s"]
                ]
                gates = {
                    "fault_detected": special.get(
                        "detection_monotonic_s"
                    )
                    is not None,
                    "detection_to_zero": (
                        special.get("detection_to_zero_s") is not None
                        and special["detection_to_zero_s"] <= 0.15
                    ),
                    "detection_to_physical_stop": (
                        special.get("detection_to_physical_stop_s") is not None
                        and special["detection_to_physical_stop_s"] <= 1.0
                    ),
                    "action_stopped_or_canceled": first_action_status in {5, 6},
                    "no_automatic_resume_after_recovery": (
                        special.get("post_recovery_displacement_m", math.inf)
                        <= 0.01
                        and special.get("post_recovery_requested_zero") is True
                    ),
                    "final_gate_blocks_late_nonzero": (
                        special.get(
                            "late_accepted_after_gate_close_count"
                        )
                        is not None
                        and special[
                            "late_accepted_after_gate_close_count"
                        ]
                        <= 2
                        and special.get("governed_zero_after_gate_close", 0) > 0
                    ),
                    "no_unexpected_contacts": not collisions,
                }
                if args.special_case == "scan_failure":
                    gates["scan_fault_observed"] = bool(
                        special.get("fault_response", {}).get("active_faults")
                    )
                    gates["stopped_health_observed"] = any(
                        item["value"].get("healthy") is False
                        for item in health_records
                    )
                else:
                    gates["scan_and_odom_remain_fresh"] = any(
                        item["value"].get("match_fault_injected") is True
                        and item["value"].get("scan_fresh") is True
                        and item["value"].get("odom_fresh") is True
                        and item["value"].get("slam_node_alive") is True
                        for item in localization_records
                    )
            elif args.special_case == "cancel_takeover":
                takeover = special.get("manual_takeover", {})
                second_run = special.get("second_goal_run", {})
                gates = {
                    "first_goal_canceled": first_action_status in {5, 6},
                    "manual_nonzero_takeover_accepted": (
                        takeover.get("command", {}).get("status")
                        in {"DISPATCHED", "APPLIED"}
                        and takeover.get("displacement_m", 0.0) > 0.002
                    ),
                    "no_resume_after_release": (
                        takeover.get("after_release_displacement_m", math.inf)
                        <= 0.01
                        and takeover.get("backend_authorized_after_cancel") is None
                    ),
                    "final_gate_blocks_late_nonzero": (
                        special.get(
                            "late_accepted_after_gate_close_count"
                        )
                        is not None
                        and special[
                            "late_accepted_after_gate_close_count"
                        ]
                        <= 2
                        and special.get("governed_zero_after_gate_close", 0) > 0
                    ),
                    "arm_preserved": (
                        takeover.get("arm_preserved") is True
                        or takeover.get("arm_target_preserved") is True
                    ),
                    "explicit_new_goal_started": (
                        second_run.get("started") is True
                    ),
                    "explicit_new_goal_output_observed": (
                        second_run.get("motion_or_terminal_observed") is True
                    ),
                    "second_goal_terminal": second_action_status in {4, 5, 6},
                    "backend_rejection_receipt": report.get(
                        "backend_rejection_receipt", {}
                    ).get("accepted")
                    is False,
                    "settled": settled_duration >= 0.60,
                    "no_unexpected_contacts": not collisions,
                }
                if args.arm_mode == "limited":
                    gates["arm_activity"] = (
                        report.get("arm_observed_delta_rad", 0.0) >= 0.01
                    )
            elif args.special_case == "reset_epoch":
                gates = {
                    "new_episode": (
                        special.get("new_episode_id") is not None
                        and special["new_episode_id"]
                        != special.get("old_episode_id")
                    ),
                    "navigation_epoch_advanced": (
                        special.get("new_navigation_epoch", -1)
                        > special.get("old_navigation_epoch", 0)
                    ),
                    "old_ros_stack_exited": all(
                        value is not None
                        for value in special.get(
                            "old_process_exit_codes", {}
                        ).values()
                    ),
                    "stale_goal_rejected": special.get(
                        "stale_control_receipt", {}
                    ).get("accepted")
                    is False,
                    "explicit_new_goal_succeeded": second_action_status == 4,
                    "backend_rejection_receipt": report.get(
                        "backend_rejection_receipt", {}
                    ).get("accepted")
                    is False,
                    "physical_position": position_error <= 0.10,
                    "physical_yaw": math.degrees(yaw_error) <= 10.0,
                    "settled": settled_duration >= 0.60,
                    "no_unexpected_contacts": not collisions,
                }
            report["gates"] = gates
            report["success"] = all(gates.values())
            stop_response = client.post(
                "/api/v1/episodes/stop",
                json={"reason": "G3B_NAVIGATION_CASE_COMPLETE"},
            )
            stop_response.raise_for_status()
            report["stop_response"] = stop_response.json()
    except ExpectedGoalRejection:
        pass
    except Exception:
        report["error"] = traceback.format_exc()
    finally:
        arm_stop.set()
        if arm_thread is not None:
            arm_thread.join(timeout=2.0)
        if goal_probe is not None:
            goal_probe.close()
        report["nav2_exit_code"] = nav2.close() if nav2 is not None else None
        report["slam_exit_code"] = slam.close() if slam is not None else None
        report["supervisor_exit_code"] = (
            supervisor.close() if supervisor is not None else None
        )
        report["application_exit_code"] = app.close()
        if rclpy.ok():
            rclpy.shutdown()
        atomic_json(report_root / "g3b_single_goal_report.json", report)
    print(
        json.dumps(
            {
                "success": report["success"],
                "report": str(report_root / "g3b_single_goal_report.json"),
                "case_id": args.goal_id,
            },
            indent=2,
        )
    )
    return 0 if report["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
