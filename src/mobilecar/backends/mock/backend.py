"""Deterministic mock backend used to prove interface-level G1 integration."""

from __future__ import annotations

import json
import math
import threading
import time
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from ...contracts.models import (
    ArmState,
    BaseState,
    Capabilities,
    CommandRequest,
    CommandResponse,
    CommandStatus,
    CommandType,
    GripperState,
    OperatorKind,
    PlanPreview,
    RobotSnapshot,
    SafetyDecision,
    SceneManifest,
    SensorKind,
    SensorObservation,
    SensorState,
    TaskPhase,
    TCPState,
)
from ..common import EventPublisher, file_sha256, new_id, quat_wxyz_to_xyzw, utc_now


REPO_ROOT = Path(__file__).resolve().parents[4]
ARM_JOINTS = [f"joint{i}" for i in range(1, 6)]


class MockBackend:
    def __init__(
        self,
        backend_config: Mapping[str, Any],
        safety_config: Mapping[str, Any],
        acceptance_config: Mapping[str, Any],
        run_root: Path,
    ) -> None:
        self.backend_config = dict(backend_config)
        self.safety_config = dict(safety_config)
        self.acceptance_config = dict(acceptance_config)
        self.run_root = Path(run_root)
        self.run_id = new_id("run")
        self.backend_generation = int(self.backend_config.get("generation", 1))
        self.episode_id = new_id("episode")
        self.episode_started = time.monotonic()
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None
        self.closed = False
        self.publisher = EventPublisher(self.run_id, lambda: self.episode_id)
        self._command_cache: dict[str, CommandResponse] = {}
        self._session_last_seq: dict[str, int] = {}
        self._owners: dict[str, str | None] = {"base": None, "arm": None}
        self._owner_kind: dict[str, OperatorKind | None] = {"base": None, "arm": None}
        self._leases = {"base": 0.0, "arm": 0.0}
        self._base_pose = np.zeros(3)
        self._base_twist = np.zeros(3)
        self._requested_base = np.zeros(3)
        self._arm_q = np.zeros(5)
        self._arm_target = np.zeros(5)
        self._gripper = 0.025
        self._gripper_target = 0.025
        self._tcp = np.array([0.178, 0.002, 0.303])
        self._tcp_target: np.ndarray | None = None
        self._plans: dict[str, dict[str, Any]] = {}
        self._plan: PlanPreview | None = None
        self._plan_status = "PREVIEWED"
        self._phase = TaskPhase.OBSERVE
        self._latched = False
        self._stopped = True
        self._stopped_since = time.monotonic()
        self._sim_time = 0.0
        self._tick = 0
        self._last_tick = "mock_0"
        self._sensor_states = {
            kind: SensorState(
                kind=kind,
                available=True,
                fresh=True,
                sample_id=f"mock_{kind.value}_0",
                stream_id=kind.value,
                measurement_time_s=time.monotonic(),
                arrival_time_s=time.monotonic(),
                quality="synthetic_fixture",
            )
            for kind in SensorKind
        }
        self._scene, self._scene_assets, self._model_sha = self._load_scene()
        self._capabilities = Capabilities(
            backend_kind="mock",
            arm_joint_names=ARM_JOINTS,
            arm_user_limits_rad=[[-2.09, 2.09]] * 5,
            arm_velocity_limits_rad_s=[0.6] * 5,
            gripper_min_width_m=0.012,
            gripper_max_width_m=0.05,
            base_linear_limit_m_s=float(self.safety_config["translation_norm_m_s"]),
            base_yaw_limit_rad_s=float(self.safety_config["yaw_rate_rad_s"]),
            tcp_position_tolerance_m=float(self.safety_config["tcp_position_tolerance_m"]),
            tcp_orientation_is_soft=True,
            supported_commands=list(CommandType),
            sensor_streams=list(SensorKind),
            model_id="landerpi_mock_same_geometry",
            model_sha256=self._model_sha,
            scene_id="mobile_pick",
            camera_parent_body="link4",
            tcp_parent_body="guide_rod_link",
            ground_truth_policy_visible=False,
        )
        self._snapshot = self._build_snapshot()

    def _load_scene(self):
        import mujoco

        from ...sensors.scene import build_scene_manifest

        path = Path(self.backend_config["model_path"])
        path = path if path.is_absolute() else REPO_ROOT / path
        path = path.resolve()
        model = mujoco.MjModel.from_xml_path(str(path))
        scene = build_scene_manifest(
            model,
            path,
            model_id="landerpi_mock_same_geometry",
            scene_id="mobile_pick",
        )
        assets = {}
        import xml.etree.ElementTree as ET

        assets_root = ET.parse(path.parent / "assets.xml").getroot()
        files = {
            node.get("name"): node.get("file")
            for node in assets_root.findall(".//mesh")
            if node.get("name") and node.get("file")
        }
        for item in scene.mesh_assets:
            name = item.asset_id + "_mesh"
            assets[item.asset_id] = (path.parent / files[name]).resolve()
        return scene, assets, scene.model_sha256

    def set_event_sink(self, sink) -> None:
        self.publisher.set_sink(sink)

    def capabilities(self) -> Capabilities:
        return self._capabilities.model_copy(deep=True)

    def scene_manifest(self) -> SceneManifest:
        return self._scene.model_copy(deep=True)

    def scene_asset(self, asset_id: str) -> bytes:
        return self._scene_assets[asset_id].read_bytes()

    def start(self) -> None:
        if self.thread and self.thread.is_alive():
            return
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._loop, name="mobilecar-mock", daemon=True)
        self.thread.start()
        self.publisher.emit("backend_started", {"backend_kind": "mock"})

    def snapshot(self) -> RobotSnapshot:
        with self.lock:
            return self._snapshot.model_copy(deep=True)

    def reset(self, request: CommandRequest) -> CommandResponse:
        return self.submit(request)

    def renew_lease(self, session_id: str) -> bool:
        with self.lock:
            renewed = False
            now = time.monotonic()
            for channel in ("base", "arm"):
                if self._owners[channel] == session_id:
                    self._leases[channel] = now + float(
                        self.safety_config["lease_timeout_s"]
                    )
                    renewed = True
            return renewed

    def submit(self, request: CommandRequest) -> CommandResponse:
        with self.lock:
            cached = self._command_cache.get(request.command_id)
            if cached:
                return cached.model_copy(deep=True)
            self.publisher.emit(
                "requested",
                {"request": request.model_dump(mode="json")},
                command=request,
            )
            invalid = self._invalid(request)
            if invalid:
                response = self._response(request, CommandStatus.REJECTED, invalid[0], invalid[1])
                self._command_cache[request.command_id] = response
                self.publisher.emit(
                    "rejected",
                    {"reason": invalid[0]},
                    command=request,
                    sim_time_s=self._sim_time,
                )
                self._snapshot = self._build_snapshot()
                return response
            response = self._apply(request)
            self._command_cache[request.command_id] = response
            self._session_last_seq[request.session_id] = max(
                request.command_seq, self._session_last_seq.get(request.session_id, 0)
            )
            self._snapshot = self._build_snapshot()
            return response.model_copy(deep=True)

    def close(self, timeout_s: float = 5.0) -> None:
        self.closed = True
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=timeout_s)

    def _invalid(self, request: CommandRequest):
        if request.run_id != self.run_id:
            return "run_id does not match mock backend", None
        if request.backend_generation != self.backend_generation:
            return "stale backend_generation", None
        if request.episode_id != self.episode_id:
            return "command belongs to another Episode", None
        if request.command_seq <= self._session_last_seq.get(request.session_id, 0):
            return "older or duplicate command_seq", None
        return None

    def _approved(self, request: CommandRequest, payload: dict) -> SafetyDecision:
        decision = SafetyDecision(
            approved=True,
            reason="approved",
            effective_payload=payload,
            allowed_contacts=["wheel-ground", "finger-target-during-grasp"],
        )
        self.publisher.emit("approved", decision.model_dump(mode="json"), command=request)
        return decision

    def _response(self, request, status, reason="", decision=None, plan=None):
        return CommandResponse(
            command_id=request.command_id,
            command_seq=request.command_seq,
            status=status,
            reason=reason,
            backend_monotonic_s=time.monotonic(),
            sim_time_s=self._sim_time,
            decision=decision,
            plan=plan,
            control_tick_id=self._last_tick,
        )

    def _apply(self, request: CommandRequest) -> CommandResponse:
        command = CommandType(request.command_type)
        if self._latched and command not in (
            CommandType.STOP,
            CommandType.RESET,
            CommandType.HEARTBEAT,
        ):
            return self._response(request, CommandStatus.REJECTED, "software stop is latched")
        if command in {
            CommandType.SET_BASE_TWIST,
            CommandType.SET_ARM_JOINT_GOAL,
            CommandType.SET_GRIPPER_WIDTH_GOAL,
            CommandType.EXECUTE_TCP_TARGET,
        } and self._recording_required():
            return self._response(
                request,
                CommandStatus.REJECTED,
                "formal recording is required and command capture is not ready",
            )
        if command == CommandType.SET_BASE_TWIST:
            ok, reason = self._acquire("base", request)
            if not ok:
                return self._response(request, CommandStatus.REJECTED, reason)
            self._requested_base[:] = [
                request.payload["vx_m_s"],
                request.payload["vy_m_s"],
                request.payload["wz_rad_s"],
            ]
            return self._response(
                request,
                CommandStatus.DISPATCHED,
                decision=self._approved(request, request.payload),
            )
        if command == CommandType.ZERO_BASE:
            self._requested_base[:] = 0.0
            self._owners["base"] = None
            return self._response(
                request,
                CommandStatus.DISPATCHED,
                decision=self._approved(request, {"twist": [0.0, 0.0, 0.0]}),
            )
        if command == CommandType.HEARTBEAT:
            for channel in ("base", "arm"):
                if self._owners[channel] == request.session_id:
                    self._leases[channel] = time.monotonic() + float(
                        self.safety_config["lease_timeout_s"]
                    )
            return self._response(request, CommandStatus.APPLIED, "lease heartbeat")
        if command == CommandType.STOP:
            latch = bool(request.payload.get("latch", True))
            self._requested_base[:] = 0.0
            self._arm_target[:] = self._arm_q
            self._gripper_target = self._gripper
            self._tcp_target = None
            self._owners = {"base": None, "arm": None}
            self._latched = latch
            return self._response(
                request,
                CommandStatus.DISPATCHED,
                decision=self._approved(request, {"latch": latch, "gripper": "hold"}),
            )
        if command == CommandType.SET_ARM_JOINT_GOAL:
            ok, reason = self._acquire("arm", request)
            if not ok:
                return self._response(request, CommandStatus.REJECTED, reason)
            values = np.asarray(request.payload["joint_positions_rad"], dtype=float)
            if np.any(np.abs(values) > 2.09):
                return self._response(request, CommandStatus.REJECTED, "joint target violates user limits")
            self._arm_target[:] = values
            self._tcp_target = None
            return self._response(
                request,
                CommandStatus.DISPATCHED,
                decision=self._approved(request, request.payload),
            )
        if command == CommandType.SET_GRIPPER_WIDTH_GOAL:
            ok, reason = self._acquire("arm", request)
            if not ok:
                return self._response(request, CommandStatus.REJECTED, reason)
            width = float(request.payload["width_m"])
            if not 0.012 <= width <= 0.05:
                return self._response(request, CommandStatus.REJECTED, "gripper width outside limits")
            self._gripper_target = width
            return self._response(
                request,
                CommandStatus.DISPATCHED,
                decision=self._approved(request, request.payload),
            )
        if command == CommandType.PREVIEW_TCP_TARGET:
            ok, reason = self._acquire("arm", request)
            if not ok:
                return self._response(request, CommandStatus.REJECTED, reason)
            position = np.asarray(request.payload["position_m"], dtype=float)
            if request.payload["frame_id"] != "world_sim":
                rotation = self._yaw_matrix(self._base_pose[2])
                position = self._base_pose[:3] + rotation @ position
            collision_free = position[2] >= 0.02
            now = time.monotonic()
            plan = PlanPreview(
                plan_id=new_id("plan"),
                command_id=request.command_id,
                target_revision=1,
                frame_id=request.payload["frame_id"],
                position_m=position.tolist(),
                waypoints_model_rad=[self._arm_q.tolist(), [0.0, 0.0, 0.0, -1.57, 0.0]],
                times_s=[0.0, 0.5],
                predicted_position_error_m=0.0,
                predicted_orientation_error_rad=0.0,
                joint_margin_rad=[2.09] * 5,
                collision_free=collision_free,
                collision_details=[] if collision_free else ["mock floor collision"],
                created_backend_monotonic_s=now,
                expires_backend_monotonic_s=now + 5.0,
            )
            self._plans[plan.plan_id] = {"position": position, "plan": plan}
            self._plan = plan
            return self._response(
                request,
                CommandStatus.RECEIVED,
                "plan previewed",
                decision=self._approved(request, request.payload),
                plan=plan,
            )
        if command == CommandType.EXECUTE_TCP_TARGET:
            plan_id = request.payload["plan_id"]
            stored = self._plans.get(plan_id)
            if stored is None:
                return self._response(request, CommandStatus.REJECTED, "unknown plan_id")
            if not stored["plan"].collision_free:
                return self._response(request, CommandStatus.REJECTED, "plan failed collision precheck")
            self._tcp_target = stored["position"].copy()
            self._plan_status = "EXECUTING"
            return self._response(
                request,
                CommandStatus.DISPATCHED,
                decision=self._approved(request, request.payload),
                plan=stored["plan"].model_copy(update={"status": "EXECUTING"}),
            )
        if command == CommandType.CANCEL:
            self._requested_base[:] = 0.0
            self._tcp_target = None
            self._plan_status = "CANCELLED"
            return self._response(
                request,
                CommandStatus.CANCELLED,
                decision=self._approved(request, {"motion": "cancel_and_hold"}),
            )
        if command == CommandType.RESET:
            self.episode_id = new_id("episode")
            self.episode_started = time.monotonic()
            self._base_pose[:] = 0.0
            self._base_twist[:] = 0.0
            self._requested_base[:] = 0.0
            self._arm_q[:] = 0.0
            self._arm_target[:] = 0.0
            self._gripper = self._gripper_target = 0.025
            self._tcp[:] = [0.178, 0.002, 0.303]
            self._tcp_target = None
            self._plan = None
            self._latched = False
            self._owners = {"base": None, "arm": None}
            self.publisher.emit("episode_started", {"episode_id": self.episode_id})
            return self._response(
                request,
                CommandStatus.APPLIED,
                decision=self._approved(request, {"confirm": True}),
            )
        if command == CommandType.SET_TASK_PHASE:
            self._phase = TaskPhase(request.payload["phase"])
            return self._response(
                request,
                CommandStatus.APPLIED,
                decision=self._approved(request, request.payload),
            )
        return self._response(request, CommandStatus.REJECTED, "unsupported mock command")

    def _recording_required(self) -> bool:
        required = bool(self.safety_config.get("require_recording", False))
        return required and not self.publisher.recording_gate(self.episode_id)

    def _acquire(self, channel: str, request: CommandRequest):
        now = time.monotonic()
        owner = self._owners[channel]
        if owner not in (None, request.session_id) and now < self._leases[channel]:
            return False, f"{channel} is owned by another live session"
        self._owners[channel] = request.session_id
        self._owner_kind[channel] = OperatorKind(request.operator_kind)
        self._leases[channel] = now + float(self.safety_config["lease_timeout_s"])
        return True, ""

    def _loop(self) -> None:
        dt = 0.02
        next_time = time.monotonic()
        while not self.stop_event.is_set():
            with self.lock:
                self._tick += 1
                self._last_tick = f"mock_{self._tick:09d}"
                self._sim_time += dt
                now = time.monotonic()
                for channel in ("base", "arm"):
                    if self._owners[channel] and now >= self._leases[channel]:
                        if channel == "base":
                            self._requested_base[:] = 0.0
                        else:
                            self._tcp_target = None
                        self._owners[channel] = None
                self._base_twist += np.clip(
                    self._requested_base - self._base_twist,
                    -0.3 * dt,
                    0.3 * dt,
                )
                yaw = self._base_pose[2]
                rotation = self._yaw_matrix(yaw)
                self._base_pose[:2] += rotation[:2, :2] @ self._base_twist[:2] * dt
                self._base_pose[2] += self._base_twist[2] * dt
                self._arm_q += np.clip(self._arm_target - self._arm_q, -0.6 * dt, 0.6 * dt)
                self._gripper += float(np.clip(self._gripper_target - self._gripper, -0.06 * dt, 0.06 * dt))
                if self._tcp_target is not None:
                    delta = self._tcp_target - self._tcp
                    distance = float(np.linalg.norm(delta))
                    if distance <= 0.002:
                        self._tcp[:] = self._tcp_target
                        self._tcp_target = None
                        self._plan_status = "REACHED"
                    else:
                        self._tcp += delta * min(1.0, 0.15 * dt / distance)
                else:
                    self._tcp = np.array(
                        [
                            self._base_pose[0] + 0.178,
                            self._base_pose[1] + 0.002,
                            0.303 + 0.03 * math.sin(self._arm_q[1]),
                        ]
                    )
                slow = np.linalg.norm(self._base_twist[:2]) <= 0.005 and abs(self._base_twist[2]) <= 0.02
                if slow:
                    self._stopped_since = self._stopped_since or now
                    self._stopped = now - self._stopped_since >= 0.2
                else:
                    self._stopped = False
                    self._stopped_since = None
                for kind in SensorKind:
                    old = self._sensor_states[kind]
                    self._sensor_states[kind] = old.model_copy(
                        update={
                            "fresh": True,
                            "measurement_time_s": now,
                            "arrival_time_s": now,
                            "sample_id": f"mock_{kind.value}_{self._tick}",
                        }
                    )
                self._snapshot = self._build_snapshot()
            next_time += dt
            delay = next_time - time.monotonic()
            if delay > 0:
                self.stop_event.wait(delay)

    def _build_snapshot(self) -> RobotSnapshot:
        now = time.monotonic()
        plan = None
        if self._plan is not None:
            plan = self._plan.model_copy(update={"status": self._plan_status})
        return RobotSnapshot(
            run_id=self.run_id,
            backend_generation=self.backend_generation,
            episode_id=self.episode_id,
            episode_started_backend_monotonic_s=self.episode_started,
            backend_monotonic_s=now,
            sim_time_s=self._sim_time,
            feedback_time_s=now,
            wall_utc=utc_now(),
            control_tick_id=self._last_tick,
            task_phase=self._phase,
            base=BaseState(
                position_world_sim_m=[float(self._base_pose[0]), float(self._base_pose[1]), 0.055],
                orientation_world_sim_xyzw=quat_wxyz_to_xyzw(
                    [math.cos(self._base_pose[2] / 2), 0.0, 0.0, math.sin(self._base_pose[2] / 2)]
                ),
                vx_m_s=float(self._base_twist[0]),
                vy_m_s=float(self._base_twist[1]),
                wz_rad_s=float(self._base_twist[2]),
                requested_vx_m_s=float(self._requested_base[0]),
                requested_vy_m_s=float(self._requested_base[1]),
                requested_wz_rad_s=float(self._requested_base[2]),
                stopped=self._stopped,
                stop_latched=self._latched,
            ),
            arm=ArmState(
                user_joint_positions_rad=self._arm_q.tolist(),
                user_joint_targets_rad=self._arm_target.tolist(),
                model_joint_positions_rad=self._arm_q.tolist(),
                model_joint_targets_rad=self._arm_target.tolist(),
                joint_velocities_rad_s=[0.0] * 5,
                joint_torques_nm=[0.0] * 5,
                joint_margin_rad=(2.09 - np.abs(self._arm_q)).tolist(),
                model_limits_rad=[[-2.09, 2.09]] * 5,
                coordinates_status="synthetic_mock",
            ),
            gripper=GripperState(
                width_m=float(self._gripper),
                target_width_m=float(self._gripper_target),
                model_drive_position_rad=0.0,
                model_drive_target_rad=0.0,
                left_contact_force_n=0.5 if self._gripper < 0.021 else 0.0,
                right_contact_force_n=0.5 if self._gripper < 0.021 else 0.0,
                bilateral_contact=self._gripper < 0.021,
            ),
            tcp=TCPState(
                position_m=self._tcp.tolist(),
                orientation_xyzw=[0.0, 0.0, 0.0, 1.0],
                position_error_m=0.0 if self._tcp_target is None else float(np.linalg.norm(self._tcp_target - self._tcp)),
                orientation_error_rad=0.0,
                held=self._gripper < 0.021 and self._tcp[2] > 0.14,
            ),
            sensors=list(self._sensor_states.values()),
            plan=plan,
            active_session_id=self._owners["base"] or self._owners["arm"],
            active_operator_kind=self._owner_kind["base"] or self._owner_kind["arm"],
            base_owner_session_id=self._owners["base"],
            arm_owner_session_id=self._owners["arm"],
            safety_latched=self._latched,
            contact_summary={"unexpected_contact": False},
            diagnostics={"mode": "MOCK", "synthetic_fixture": True},
        )

    @staticmethod
    def _yaw_matrix(yaw: float) -> np.ndarray:
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
