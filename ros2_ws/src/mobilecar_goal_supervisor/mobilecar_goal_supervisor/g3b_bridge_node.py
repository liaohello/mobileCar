"""G3B bridge, goal supervisor, authorization barrier and final speed gate."""

from __future__ import annotations

import json
import hashlib
import math
import queue
import threading
import time
import uuid
from pathlib import Path
from typing import Any

import yaml
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String

from mobilecar.config import load_config
from mobilecar.contracts.g3b import MapPackageManifest
from mobilecar_mapping_localization.map_package import (
    compatibility_diff,
    load_manifest,
    occupancy_goal_check,
)
from mobilecar_navigation_bridge.bridge_node import NavigationBridge


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def current_map_compatibility(
    *,
    runtime_config: Path,
) -> dict[str, Any]:
    config = load_config(runtime_config).data
    backend = config["backend"]
    g3b_profile_path = Path(config["g3b"]["profile_path"]).resolve()
    g3b_profile = yaml.safe_load(
        g3b_profile_path.read_text(encoding="utf-8")
    )
    navigation_profile_path = Path(
        backend["navigation_profile_path"]
    ).resolve()
    navigation_profile = yaml.safe_load(
        navigation_profile_path.read_text(encoding="utf-8")
    )
    sensor_profile_path = Path(
        backend["observation_profile_path"]
    ).resolve()
    scan_config = navigation_profile["sensors"]["scan"]
    scan_config_sha256 = hashlib.sha256(
        json.dumps(
            scan_config, sort_keys=True, separators=(",", ":")
        ).encode("utf-8")
    ).hexdigest()
    return {
        "scene_id": Path(backend["scene_path"]).stem,
        "model_sha256": sha256_file(Path(backend["model_path"]).resolve()),
        "navigation_profile_sha256": sha256_file(navigation_profile_path),
        "sensor_profile_sha256": sha256_file(sensor_profile_path),
        "scan_config_sha256": scan_config_sha256,
        "resolution_m": float(g3b_profile["map_package"]["resolution_m"]),
        "map_frame": str(g3b_profile["map_package"]["map_frame"]),
        "odom_frame": str(g3b_profile["map_package"]["odom_frame"]),
        "base_frame": str(g3b_profile["map_package"]["base_frame"]),
    }


class G3BSupervisor(NavigationBridge):
    def __init__(self) -> None:
        self._supervisor_lock = threading.RLock()
        self._receipt_condition = threading.Condition(self._supervisor_lock)
        self.receipts: dict[str, dict[str, Any]] = {}
        self.supervisor_ready = False
        self.active_goal: dict[str, Any] | None = None
        self.last_terminal_goal: dict[str, Any] | None = None
        self.command_gate_open = False
        self.backend_authorized = False
        self.cancel_requested = False
        self.abort_event = threading.Event()
        self.identity_reset_event = threading.Event()
        self.last_raw_command_s: float | None = None
        self.raw_command_count = 0
        self.nonzero_raw_command_count = 0
        self.last_raw_command_values = [0.0, 0.0, 0.0]
        self.last_accepted_control_s: float | None = None
        self.pending_velocity_request_id: str | None = None
        self.pending_velocity_since_s: float | None = None
        self.pending_velocity_requests: dict[str, float] = {}
        self.accepted_velocity_receipt_count = 0
        self.late_accepted_after_gate_close_count = 0
        self.last_rejected_receipt: dict[str, Any] | None = None
        self.goal_counter = 0
        self.current_map_id = "map_unconfigured"
        self.localization_health: dict[str, Any] = {}
        self.navigation_health: dict[str, Any] = {}
        self.health_reasons: list[str] = []
        self.health_status = "INITIALIZING"
        self.detection_started_s: float | None = None
        self.last_detection_latency_s: float | None = None
        self.health_transitions: list[dict[str, Any]] = []
        self.gate_transitions: list[dict[str, Any]] = []
        self._last_health_key: tuple[str, tuple[str, ...]] | None = None
        self._last_terminal_s: float | None = None
        self.map_validation: dict[str, Any] = {
            "compatible": False,
            "reason": "map package not validated",
        }
        self.last_goal_validation: dict[str, Any] | None = None
        self.raw_commands: queue.Queue[Twist | None] = queue.Queue(maxsize=2)
        super().__init__()
        self.declare_parameter("map_id", "map_unconfigured")
        self.declare_parameter("map_manifest", "")
        self.declare_parameter("runtime_config", "")
        self.declare_parameter("outer_action", "/navigate_to_pose")
        self.declare_parameter("inner_action", "/nav2/navigate_to_pose")
        self.declare_parameter("raw_cmd_topic", "/mobilecar/nav2/cmd_vel_raw")
        self.declare_parameter(
            "governed_cmd_topic", "/mobilecar/nav2/cmd_vel_governed"
        )
        self.declare_parameter(
            "localization_health_topic", "/mobilecar/localization_health"
        )
        self.declare_parameter(
            "navigation_system_health_topic",
            "/mobilecar/navigation_system_health",
        )
        self.declare_parameter(
            "goal_supervisor_status_topic", "/mobilecar/goal_supervisor_status"
        )
        self.declare_parameter("authorization_timeout_s", 0.5)
        self.declare_parameter("receipt_timeout_s", 0.30)
        self.declare_parameter("goal_barrier_s", 0.30)
        self.declare_parameter("raw_command_timeout_s", 0.50)
        self.current_map_id = str(self.get_parameter("map_id").value)
        self.map_validation = self._validate_map_package()
        self.callback_group = ReentrantCallbackGroup()
        self.governed_publisher = self.create_publisher(
            Twist, str(self.get_parameter("governed_cmd_topic").value), 10
        )
        self.raw_subscription = self.create_subscription(
            Twist,
            str(self.get_parameter("raw_cmd_topic").value),
            self.on_raw_twist,
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("localization_health_topic").value),
            self.on_localization_health,
            20,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("health_topic").value),
            self.on_navigation_health,
            20,
            callback_group=self.callback_group,
        )
        self.system_health_publisher = self.create_publisher(
            String,
            str(self.get_parameter("navigation_system_health_topic").value),
            20,
        )
        self.goal_status_publisher = self.create_publisher(
            String,
            str(self.get_parameter("goal_supervisor_status_topic").value),
            20,
        )
        self.action_server = ActionServer(
            self,
            NavigateToPose,
            str(self.get_parameter("outer_action").value),
            execute_callback=self.execute_goal,
            goal_callback=self.on_goal,
            cancel_callback=self.on_cancel,
            callback_group=self.callback_group,
        )
        self.nav2_client = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter("inner_action").value),
            callback_group=self.callback_group,
        )
        self.stop_event = threading.Event()
        self.command_worker = threading.Thread(
            target=self.command_loop, name="g3b-command-gate", daemon=True
        )
        self.health_worker = threading.Thread(
            target=self.health_loop, name="g3b-system-health", daemon=True
        )
        self.command_worker.start()
        self.health_worker.start()
        self.supervisor_ready = True

    def _validate_map_package(self) -> dict[str, Any]:
        manifest_name = str(self.get_parameter("map_manifest").value)
        runtime_config_name = str(self.get_parameter("runtime_config").value)
        if not manifest_name or not runtime_config_name:
            return {
                "compatible": False,
                "reason": "map_manifest_and_runtime_config_are_required",
                "map_id": self.current_map_id,
            }
        try:
            manifest_path = Path(manifest_name).resolve()
            package_root = manifest_path.parent
            manifest = load_manifest(package_root)
            MapPackageManifest.model_validate(manifest)
            current = current_map_compatibility(
                runtime_config=Path(runtime_config_name).resolve()
            )
            compatibility = compatibility_diff(manifest, current)
            prior = manifest.get("initial_prior") or {"x_m": 0.0, "y_m": 0.0}
            initial_cell = occupancy_goal_check(
                package_root, float(prior["x_m"]), float(prior["y_m"])
            )
            compatible = (
                compatibility["compatible"]
                and manifest.get("map_id") == self.current_map_id
                and initial_cell["accepted"]
            )
            return {
                "compatible": compatible,
                "map_id": self.current_map_id,
                "manifest_map_id": manifest.get("map_id"),
                "manifest_path": str(manifest_path),
                "manifest_sha256": sha256_file(manifest_path),
                "compatibility": compatibility,
                "initial_cell": initial_cell,
                "reason": (
                    "compatible"
                    if compatible
                    else "map_id_or_compatibility_or_initial_prior_rejected"
                ),
            }
        except Exception as exc:
            return {
                "compatible": False,
                "map_id": self.current_map_id,
                "reason": f"{type(exc).__name__}: {exc}",
            }

    def _clear_raw_commands(self) -> None:
        while True:
            try:
                self.raw_commands.get_nowait()
            except queue.Empty:
                break
        self.last_raw_command_s = None

    def _identity_matches_goal(self, goal: dict[str, Any]) -> bool:
        identity, _clock = self.current_transport_state()
        return identity is not None and identity == goal.get("identity")

    def _validate_goal_request(self, request) -> dict[str, Any]:
        pose = request.pose.pose
        frame_id = str(request.pose.header.frame_id)
        if frame_id != "map":
            return {
                "accepted": False,
                "reason": "goal_frame_must_be_map",
                "frame_id": frame_id,
            }
        if not self.map_validation.get("compatible", False):
            return {
                "accepted": False,
                "reason": "map_package_not_compatible",
                "map_validation": self.map_validation,
            }
        position = pose.position
        values = [
            float(position.x),
            float(position.y),
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        ]
        if not all(math.isfinite(value) for value in values):
            return {"accepted": False, "reason": "goal_contains_non_finite_value"}
        if abs(
            values[2] ** 2
            + values[3] ** 2
            + values[4] ** 2
            + values[5] ** 2
            - 1.0
        ) > 1e-3:
            return {"accepted": False, "reason": "goal_orientation_not_normalized"}
        map_root = Path(str(self.get_parameter("map_manifest").value)).resolve().parent
        cell = occupancy_goal_check(map_root, values[0], values[1])
        return {
            **cell,
            "reason": "goal_cell_free" if cell["accepted"] else cell["classification"],
            "frame_id": frame_id,
        }

    def on_control_receipt(self, message: dict[str, Any]) -> None:
        request_id = str(message.get("request_id", ""))
        with self._receipt_condition:
            self.receipts[request_id] = dict(message)
            if len(self.receipts) > 512:
                oldest = next(iter(self.receipts))
                self.receipts.pop(oldest, None)
            if message.get("accepted") is not True:
                self.last_rejected_receipt = dict(message)
                self.close_gate("backend_control_rejected")
            elif request_id in self.pending_velocity_requests:
                self.accepted_velocity_receipt_count += 1
                receipt_monotonic = message.get("backend_monotonic_s")
                if (
                    not self.command_gate_open
                    and
                    self.gate_transitions
                    and isinstance(receipt_monotonic, (int, float))
                    and float(receipt_monotonic)
                    >= float(self.gate_transitions[-1]["monotonic_s"])
                ):
                    self.late_accepted_after_gate_close_count += 1
                self.last_accepted_control_s = time.monotonic()
            if request_id == self.pending_velocity_request_id:
                self.pending_velocity_request_id = None
                self.pending_velocity_since_s = None
            self.pending_velocity_requests.pop(request_id, None)
            self._receipt_condition.notify_all()

    def on_identity_changed(
        self, previous: dict[str, Any] | None, current: dict[str, Any]
    ) -> None:
        if previous is None:
            return
        self.close_gate("navigation_identity_changed")
        self._clear_raw_commands()
        self.identity_reset_event.set()
        self.abort_event.set()

    def on_localization_health(self, message: String) -> None:
        try:
            self.localization_health = json.loads(message.data)
        except json.JSONDecodeError:
            self.localization_health = {
                "status": "FAILED",
                "stop_required": True,
                "reasons": ["invalid localization health JSON"],
            }

    def on_navigation_health(self, message: String) -> None:
        try:
            self.navigation_health = json.loads(message.data)
        except json.JSONDecodeError:
            self.navigation_health = {
                "status": "FAILED",
                "stop_required": True,
                "reasons": ["invalid navigation health JSON"],
            }

    def _health_state(self) -> tuple[bool, list[str]]:
        reasons: list[str] = []
        localization = self.localization_health
        navigation = self.navigation_health
        localization_status = str(localization.get("status", "INITIALIZING"))
        if localization_status not in {"READY", "STATIONARY_HOLD"}:
            reasons.append(f"localization_{localization_status.lower()}")
        if bool(localization.get("stop_required", True)):
            reasons.extend(str(value) for value in localization.get("reasons", []))
        if not bool(localization.get("scan_fresh", False)):
            reasons.append("navigation_scan_not_fresh")
        if not bool(localization.get("odom_fresh", False)):
            reasons.append("navigation_odom_not_fresh")
        navigation_status = str(navigation.get("status", "INITIALIZING"))
        if navigation_status not in {"HEALTHY", "DEGRADED"}:
            reasons.append(f"g3a_navigation_{navigation_status.lower()}")
        if bool(navigation.get("stop_required", True)):
            reasons.append("g3a_navigation_stop_required")
        source_health = navigation.get("source_health") or {}
        if source_health and not bool(source_health.get("scan_available", False)):
            reasons.append("g3a_scan_unavailable")
        return not reasons, sorted(set(reasons))

    def health_loop(self) -> None:
        while not self.stop_event.wait(0.05):
            healthy, reasons = self._health_state()
            self.health_reasons = reasons
            if healthy:
                self.health_status = (
                    "HEALTHY"
                    if str(self.localization_health.get("status")) == "READY"
                    else "STATIONARY_HOLD"
                )
                self.detection_started_s = None
            else:
                self.health_status = "STOPPED"
                if self.detection_started_s is None:
                    self.detection_started_s = time.monotonic()
                if self.command_gate_open or self.backend_authorized:
                    self.close_gate("navigation_system_health")
                    if self.last_detection_latency_s is None:
                        self.last_detection_latency_s = max(
                            0.0, time.monotonic() - self.detection_started_s
                        )
                    self.abort_event.set()
            health_key = (self.health_status, tuple(reasons))
            if health_key != self._last_health_key:
                self._last_health_key = health_key
                self.health_transitions.append(
                    {
                        "monotonic_s": time.monotonic(),
                        "status": self.health_status,
                        "reasons": list(reasons),
                    }
                )
                self.health_transitions = self.health_transitions[-50:]
                self.get_logger().warning(
                    f"navigation health {self.health_status}: {reasons}"
                )
            self.publish_system_health(healthy, reasons)
            self.publish_goal_status()

    def publish_system_health(self, healthy: bool, reasons: list[str]) -> None:
        identity, clock = self.current_transport_state()
        payload = {
            "schema_version": 1,
            "interface_profile": "g3b-1.0",
            "status": self.health_status,
            "healthy": healthy,
            "stop_required": not healthy,
            "command_gate_open": self.command_gate_open,
            "identity": identity,
            "clock": clock,
            "localization": self.localization_health,
            "g3a_navigation": self.navigation_health,
            "reasons": reasons,
            "detection_latency_s": self.last_detection_latency_s,
            "clock_domain": "capture_sim_time_headers; steady_monotonic_health_and_gate",
            "nav2_active": bool(self.active_goal),
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"), allow_nan=False)
        self.system_health_publisher.publish(message)

    def publish_goal_status(self) -> None:
        with self._supervisor_lock:
            goal = dict(self.active_goal) if self.active_goal is not None else {}
            payload = {
                "schema_version": 1,
                "interface_profile": "g3b-1.0",
                "goal_id": goal.get("goal_id"),
                "goal_generation": goal.get("goal_generation", 0),
                "map_id": goal.get("map_id"),
                "session_id": goal.get("session_id"),
                "state": goal.get("state", "IDLE"),
                "action_accepted": bool(goal.get("action_accepted", False)),
                "backend_authorized": self.backend_authorized,
                "command_gate_open": self.command_gate_open,
                "active_action_name": str(self.get_parameter("inner_action").value),
                "reasons": list(goal.get("reasons", [])),
                "last_rejected_receipt": self.last_rejected_receipt,
                "raw_command_count": self.raw_command_count,
                "nonzero_raw_command_count": self.nonzero_raw_command_count,
                "last_raw_command_values": list(self.last_raw_command_values),
                "health_transitions": list(self.health_transitions),
                "gate_transitions": list(self.gate_transitions),
                "accepted_velocity_receipt_count": self.accepted_velocity_receipt_count,
                "late_accepted_after_gate_close_count": (
                    self.late_accepted_after_gate_close_count
                ),
                "pending_velocity_request_id": self.pending_velocity_request_id,
                "identity": goal.get("identity"),
                "requested_pose": goal.get("requested_pose"),
                "last_terminal_goal": (
                    dict(self.last_terminal_goal)
                    if self.last_terminal_goal is not None
                    else None
                ),
                "map_validation": self.map_validation,
                "last_goal_validation": self.last_goal_validation,
            }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"), allow_nan=False)
        self.goal_status_publisher.publish(message)

    def close_gate(self, reason: str) -> None:
        transition = None
        with self._supervisor_lock:
            was_open = self.command_gate_open
            self.command_gate_open = False
            if was_open or self.backend_authorized:
                transition = {
                    "monotonic_s": time.monotonic(),
                    "reason": reason,
                    "accepted_velocity_receipt_count": (
                        self.accepted_velocity_receipt_count
                    ),
                }
            if self.active_goal is not None and reason:
                self.active_goal.setdefault("reasons", []).append(reason)
        self._clear_raw_commands()
        if was_open:
            zero = Twist()
            self.governed_publisher.publish(zero)
            if transition is not None:
                transition["zero_publish_monotonic_s"] = time.monotonic()
        if transition is not None:
            with self._supervisor_lock:
                self.gate_transitions.append(transition)
                self.gate_transitions = self.gate_transitions[-50:]

    def _control_message(
        self,
        *,
        goal: dict[str, Any],
        message_type: str,
        vx: float = 0.0,
        vy: float = 0.0,
        wz: float = 0.0,
        request_id: str | None = None,
        reason: str = "",
    ) -> dict[str, Any] | None:
        identity = goal.get("identity")
        if identity is None:
            return None
        request_id = request_id or f"req_{uuid.uuid4().hex}"
        return {
            "schema_version": 2,
            "interface_profile": "g3b-1.0",
            "message_type": message_type,
            "source": "nav2_goal",
            "request_id": request_id,
            "goal_id": goal["goal_id"],
            "goal_generation": int(goal["goal_generation"]),
            "map_id": goal["map_id"],
            **identity,
            "session_id": goal["session_id"],
            "lease_ttl_s": float(
                self.get_parameter("command_lease_ttl_s").value
            ),
            "vx_m_s": float(vx),
            "vy_m_s": float(vy),
            "wz_rad_s": float(wz),
            "reason": reason,
        }

    def send_and_wait(
        self, control: dict[str, Any], timeout_s: float | None = None
    ) -> dict[str, Any] | None:
        request_id = control["request_id"]
        timeout = (
            float(self.get_parameter("receipt_timeout_s").value)
            if timeout_s is None
            else float(timeout_s)
        )
        if not self.send_g3b_control(control):
            return None
        deadline = time.monotonic() + timeout
        with self._receipt_condition:
            while time.monotonic() < deadline:
                if request_id in self.receipts:
                    return self.receipts.pop(request_id)
                self._receipt_condition.wait(timeout=min(0.05, timeout))
        return None

    def authorize_goal(self, goal: dict[str, Any]) -> dict[str, Any] | None:
        control = self._control_message(
            goal=goal,
            message_type="authorize",
            request_id=f"authorize_{uuid.uuid4().hex}",
            reason="new_nav2_goal",
        )
        if control is None:
            return None
        return self.send_and_wait(
            control,
            timeout_s=float(self.get_parameter("authorization_timeout_s").value),
        )

    def revoke_goal(self, goal: dict[str, Any], reason: str) -> dict[str, Any] | None:
        control = self._control_message(
            goal=goal,
            message_type="revoke",
            request_id=f"revoke_{uuid.uuid4().hex}",
            reason=reason,
        )
        return None if control is None else self.send_and_wait(control)

    def release_goal(self, goal: dict[str, Any], reason: str) -> dict[str, Any] | None:
        control = self._control_message(
            goal=goal,
            message_type="release",
            request_id=f"release_{uuid.uuid4().hex}",
            reason=reason,
        )
        return None if control is None else self.send_and_wait(control)

    def on_raw_twist(self, message: Twist) -> None:
        values = [
            float(message.linear.x),
            float(message.linear.y),
            float(message.angular.z),
        ]
        self.last_raw_command_s = time.monotonic()
        with self._supervisor_lock:
            self.raw_command_count += 1
            self.last_raw_command_values = list(values)
            if any(abs(value) > 1e-9 for value in self.last_raw_command_values):
                self.nonzero_raw_command_count += 1
            goal = dict(self.active_goal) if self.active_goal is not None else None
            deliver = (
                self.command_gate_open
                and goal is not None
                and time.monotonic()
                >= float(goal.get("gate_opened_s", math.inf))
                and self._identity_matches_goal(goal)
            )
        if not all(math.isfinite(value) for value in values):
            self.close_gate("nav2_nonfinite_velocity")
            self.abort_event.set()
            return
        if not deliver:
            return
        try:
            self.raw_commands.put_nowait(message)
        except queue.Full:
            try:
                self.raw_commands.get_nowait()
            except queue.Empty:
                pass
            try:
                self.raw_commands.put_nowait(message)
            except queue.Full:
                pass

    def command_loop(self) -> None:
        next_keepalive_s = 0.0
        while not self.stop_event.is_set():
            try:
                message = self.raw_commands.get(timeout=0.1)
            except queue.Empty:
                message = None
            with self._supervisor_lock:
                gate_open = self.command_gate_open
            if gate_open:
                if (
                    self.last_accepted_control_s is None
                    or time.monotonic() - self.last_accepted_control_s > 1.0
                ):
                    self.close_gate("velocity_acceptance_timeout")
                    self.abort_event.set()
                    continue
                if (
                    self.last_raw_command_s is None
                    or time.monotonic() - self.last_raw_command_s
                    > float(self.get_parameter("raw_command_timeout_s").value)
                ):
                    self.close_gate("nav2_velocity_timeout")
                    self.abort_event.set()
                    continue
                if message is not None:
                    with self._supervisor_lock:
                        if self.command_gate_open:
                            self.governed_publisher.publish(message)
                            next_keepalive_s = time.monotonic() + 0.20
                elif time.monotonic() >= next_keepalive_s:
                    with self._supervisor_lock:
                        if self.command_gate_open:
                            self.governed_publisher.publish(Twist())
                            next_keepalive_s = time.monotonic() + 0.20

    def on_command_values(self, values: list[float]) -> None:
        if not self.command_gate_open or self.active_goal is None:
            return
        goal = dict(self.active_goal)
        if not self._identity_matches_goal(goal):
            self.close_gate("goal_identity_changed")
            self.abort_event.set()
            return
        control = self._control_message(
            goal=goal,
            message_type="cmd_vel",
            vx=values[0],
            vy=values[1],
            wz=values[2],
            request_id=f"velocity_{uuid.uuid4().hex}",
            reason="nav2_governed_velocity",
        )
        if control is None:
            self.close_gate("missing_transport_identity")
            self.abort_event.set()
            return
        with self._supervisor_lock:
            self.pending_velocity_request_id = control["request_id"]
            self.pending_velocity_since_s = time.monotonic()
            self.pending_velocity_requests[control["request_id"]] = (
                self.pending_velocity_since_s
            )
            if len(self.pending_velocity_requests) > 32:
                oldest = min(
                    self.pending_velocity_requests,
                    key=self.pending_velocity_requests.get,
                )
                self.pending_velocity_requests.pop(oldest, None)
        if not self.send_g3b_control(control):
            self.close_gate("backend_velocity_not_accepted")
            with self._supervisor_lock:
                self.pending_velocity_request_id = None
                self.pending_velocity_since_s = None
                self.pending_velocity_requests.pop(control["request_id"], None)
            self.abort_event.set()

    def on_goal(self, goal_request) -> GoalResponse:
        with self._supervisor_lock:
            if self.active_goal is not None:
                return GoalResponse.REJECT
            if (
                self._last_terminal_s is not None
                and time.monotonic() - self._last_terminal_s
                < float(self.get_parameter("goal_barrier_s").value)
            ):
                self.last_goal_validation = {
                    "accepted": False,
                    "reason": "goal_terminal_barrier_active",
                }
                return GoalResponse.REJECT
            healthy, reasons = self._health_state()
            if not healthy:
                self.last_goal_validation = {
                    "accepted": False,
                    "reason": "navigation_not_healthy",
                    "reasons": reasons,
                }
                return GoalResponse.REJECT
            validation = self._validate_goal_request(goal_request)
            self.last_goal_validation = validation
            if validation.get("accepted") is not True:
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def on_cancel(self, _goal_handle) -> CancelResponse:
        self.close_gate("cancel_requested")
        self.cancel_requested = True
        self.abort_event.set()
        return CancelResponse.ACCEPT

    def wait_inner_result(self, future, timeout_s: float = 5.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not future.done():
            time.sleep(0.02)
        return future.result() if future.done() else None

    def execute_goal(self, goal_handle):
        request = goal_handle.request
        result = NavigateToPose.Result()
        validation = self._validate_goal_request(request)
        self.last_goal_validation = validation
        if validation.get("accepted") is not True:
            goal_handle.abort()
            return result
        identity, clock = self.current_transport_state()
        if identity is None or clock is None:
            goal_handle.abort()
            return result
        if self.active_goal is not None:
            goal_handle.abort()
            return result
        with self._supervisor_lock:
            self.goal_counter += 1
            goal = {
                "goal_id": f"goal_{self.goal_counter:06d}_{uuid.uuid4().hex[:12]}",
                "goal_generation": self.goal_counter,
                "map_id": self.current_map_id,
                "session_id": f"nav2_goal_{self.goal_counter}_{uuid.uuid4().hex[:8]}",
                "state": "AUTHORIZING",
                "action_accepted": False,
                "reasons": [],
                "identity": identity,
                "requested_pose": {
                    "frame_id": str(request.pose.header.frame_id),
                    "x_m": float(request.pose.pose.position.x),
                    "y_m": float(request.pose.pose.position.y),
                    "yaw_rad": math.atan2(
                        2.0
                        * (
                            float(request.pose.pose.orientation.w)
                            * float(request.pose.pose.orientation.z)
                            + float(request.pose.pose.orientation.x)
                            * float(request.pose.pose.orientation.y)
                        ),
                        1.0
                        - 2.0
                        * (
                            float(request.pose.pose.orientation.y) ** 2
                            + float(request.pose.pose.orientation.z) ** 2
                        ),
                    ),
                },
            }
            self.active_goal = goal
            self.abort_event.clear()
            self.cancel_requested = False
            self.backend_authorized = False
            self._clear_raw_commands()
        authorization = self.authorize_goal(goal)
        if authorization is None or authorization.get("accepted") is not True:
            goal["state"] = "REJECTED"
            goal["reasons"].append("backend_authorization_not_accepted")
            with self._supervisor_lock:
                self.last_terminal_goal = dict(goal)
                self.active_goal = None
                self.backend_authorized = False
                self.command_gate_open = False
                self._last_terminal_s = time.monotonic()
            self._clear_raw_commands()
            goal_handle.abort()
            return result
        if not self._identity_matches_goal(goal):
            self.revoke_goal(goal, "identity_changed_during_authorization")
            goal["state"] = "ABORTED"
            goal["reasons"].append("identity_changed_during_authorization")
            with self._supervisor_lock:
                self.active_goal = None
                self.backend_authorized = False
                self.command_gate_open = False
                self._last_terminal_s = time.monotonic()
            self._clear_raw_commands()
            goal_handle.abort()
            return result
        self.backend_authorized = True
        if not self.nav2_client.wait_for_server(timeout_sec=10.0):
            self.release_goal(goal, "nav2_action_unavailable")
            goal["state"] = "ABORTED"
            goal["reasons"].append("nav2_action_unavailable")
            with self._supervisor_lock:
                self.active_goal = None
                self.backend_authorized = False
                self.command_gate_open = False
                self._last_terminal_s = time.monotonic()
            self._clear_raw_commands()
            goal_handle.abort()
            return result

        feedback_holder: dict[str, Any] = {}

        def feedback_callback(message) -> None:
            feedback_holder["feedback"] = message.feedback

        send_future = self.nav2_client.send_goal_async(
            request, feedback_callback=feedback_callback
        )
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and not send_future.done():
            time.sleep(0.02)
        if not send_future.done():
            self.release_goal(goal, "nav2_accept_timeout")
            goal_handle.abort()
            with self._supervisor_lock:
                self.active_goal = None
                self.backend_authorized = False
                self.command_gate_open = False
                self._last_terminal_s = time.monotonic()
            self._clear_raw_commands()
            return result
        inner_goal_handle = send_future.result()
        if inner_goal_handle is None or not inner_goal_handle.accepted:
            self.release_goal(goal, "nav2_goal_rejected")
            goal["state"] = "REJECTED"
            goal["reasons"].append("nav2_goal_rejected")
            with self._supervisor_lock:
                self.active_goal = None
                self.backend_authorized = False
                self.command_gate_open = False
                self._last_terminal_s = time.monotonic()
            self._clear_raw_commands()
            goal_handle.abort()
            return result
        if not self._identity_matches_goal(goal):
            self.release_goal(goal, "identity_changed_before_gate")
            goal["state"] = "ABORTED"
            goal["reasons"].append("identity_changed_before_gate")
            with self._supervisor_lock:
                self.active_goal = None
                self.backend_authorized = False
                self.command_gate_open = False
                self._last_terminal_s = time.monotonic()
            self._clear_raw_commands()
            goal_handle.abort()
            return result
        goal["state"] = "ACTIVE"
        goal["action_accepted"] = True
        self._clear_raw_commands()
        self.late_accepted_after_gate_close_count = 0
        goal["gate_opened_s"] = time.monotonic()
        self.command_gate_open = True
        self.last_raw_command_s = time.monotonic()
        self.last_accepted_control_s = time.monotonic()
        result_future = inner_goal_handle.get_result_async()
        terminal_reason = None
        while rclpy.ok() and not result_future.done():
            if goal_handle.is_cancel_requested or self.cancel_requested:
                terminal_reason = "cancel_requested"
                break
            if self.abort_event.is_set() or not self.command_gate_open:
                terminal_reason = "health_or_control_failure"
                break
            if "feedback" in feedback_holder:
                goal_handle.publish_feedback(feedback_holder["feedback"])
            time.sleep(0.05)
        if terminal_reason is not None:
            self.close_gate(terminal_reason)
            self.revoke_goal(goal, terminal_reason)
            cancel_future = inner_goal_handle.cancel_goal_async()
            self.wait_inner_result(cancel_future, timeout_s=3.0)
            self.wait_inner_result(result_future, timeout_s=3.0)
            time.sleep(float(self.get_parameter("goal_barrier_s").value))
            goal["state"] = (
                "CANCEL_REQUESTED" if terminal_reason == "cancel_requested" else "ABORTED"
            )
            goal["reasons"].append(terminal_reason)
            with self._supervisor_lock:
                self.active_goal = None
                self.backend_authorized = False
                self.command_gate_open = False
                self._last_terminal_s = time.monotonic()
            self._clear_raw_commands()
            if terminal_reason == "cancel_requested":
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return result
        inner_result = result_future.result()
        self.close_gate("goal_terminal")
        self.release_goal(goal, "goal_terminal")
        if inner_result.status == 4:
            goal["state"] = "SUCCEEDED"
            goal["action_result_status"] = 4
            goal_handle.succeed()
            with self._supervisor_lock:
                self.last_terminal_goal = dict(goal)
                self.active_goal = None
                self.backend_authorized = False
                self.command_gate_open = False
                self._last_terminal_s = time.monotonic()
            self._clear_raw_commands()
            return inner_result.result
        goal["state"] = "ABORTED"
        goal["action_result_status"] = int(inner_result.status)
        goal["reasons"].append(f"nav2_action_status_{inner_result.status}")
        goal_handle.abort()
        with self._supervisor_lock:
            self.last_terminal_goal = dict(goal)
            self.active_goal = None
            self.backend_authorized = False
            self.command_gate_open = False
            self._last_terminal_s = time.monotonic()
        self._clear_raw_commands()
        return inner_result.result

    def close(self) -> None:
        self.stop_event.set()
        try:
            self.raw_commands.put_nowait(None)
        except queue.Full:
            pass
        self.command_worker.join(timeout=2.0)
        self.health_worker.join(timeout=2.0)
        super().close()


class GoalPoseRelay(Node):
    """Turn RViz GoalTool output into the single public NavigateToPose goal."""

    def __init__(self, action_name: str) -> None:
        super().__init__("mobilecar_g3b_goal_pose_relay")
        self.action_name = action_name
        self.client = ActionClient(self, NavigateToPose, action_name)
        self.active_handle = None
        self.status_publisher = self.create_publisher(
            String, "/mobilecar/goal_pose_relay_status", 20
        )
        self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.on_goal_pose,
            10,
            callback_group=ReentrantCallbackGroup(),
        )
        self.create_subscription(
            Bool,
            "/mobilecar/nav2/cancel_goal",
            self.on_cancel,
            10,
            callback_group=ReentrantCallbackGroup(),
        )

    def publish_status(self, state: str, **values: Any) -> None:
        message = String()
        message.data = json.dumps(
            {
                "schema_version": 1,
                "interface_profile": "g3b-1.0",
                "state": state,
                "action_name": self.action_name,
                **values,
            },
            separators=(",", ":"),
            allow_nan=False,
        )
        self.status_publisher.publish(message)

    def on_goal_pose(self, pose: PoseStamped) -> None:
        if self.active_handle is not None:
            self.publish_status("REJECTED_ACTIVE_GOAL")
            return
        if not self.client.wait_for_server(timeout_sec=2.0):
            self.publish_status("REJECTED_ACTION_UNAVAILABLE")
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.publish_status(
            "REQUESTED",
            frame_id=pose.header.frame_id,
            x_m=float(pose.pose.position.x),
            y_m=float(pose.pose.position.y),
        )
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            self.publish_status("REJECTED", reason=str(exc))
            return
        if handle is None or not handle.accepted:
            self.publish_status("REJECTED")
            return
        self.active_handle = handle
        self.publish_status("ACCEPTED")
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_result(self, future) -> None:
        try:
            result = future.result()
            status = int(result.status)
            self.publish_status("TERMINAL", action_status=status)
        except Exception as exc:
            self.publish_status("TERMINAL", reason=str(exc))
        self.active_handle = None

    def on_cancel(self, message: Bool) -> None:
        if message.data and self.active_handle is not None:
            self.active_handle.cancel_goal_async()
            self.publish_status("CANCEL_REQUESTED")


def main() -> None:
    rclpy.init()
    node = G3BSupervisor()
    relay = GoalPoseRelay(str(node.get_parameter("outer_action").value))
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(relay)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.close()
        except (KeyboardInterrupt, RuntimeError):
            pass
        try:
            relay.destroy_node()
        except (KeyboardInterrupt, RuntimeError):
            pass
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
