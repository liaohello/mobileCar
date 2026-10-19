"""Single-owner MuJoCo backend for the G1 LanderPi scene."""

from __future__ import annotations

import copy
import json
import math
import os
import threading
import time
import xml.etree.ElementTree as ET
from collections import Counter, OrderedDict
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping
from uuid import uuid4

import mujoco
import numpy as np
import yaml
from PIL import Image

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
from ...contracts.navigation import (
    G3A_NAVIGATION_PROFILE_ID,
    EstimatedBaseState,
    ImuMeasurement,
    LaserReturnStatus,
    LaserScanObservation,
    NavigationClockV1,
    NavigationControlMessage,
    NavigationHealth,
    NavigationJointMeasurement,
    NavigationObservation,
    WheelMeasurement,
)
from ...contracts.g3b import (
    ControlReceipt,
    G3BControlMessage,
    G3BControlType,
)
from ...contracts.g4a1 import (
    G4A1ActionReceipt,
    G4A1ActionRequest,
    G4A1Observation,
    G4A1Status,
    NearfieldPermissionReceipt,
    NearfieldPermissionRequest,
)
from ...contracts.g4a2 import (
    G4A2ActionReceipt,
    G4A2ActionRequest,
    HumanIntentReceipt,
    HumanIntentRequest,
    HumanSessionReceipt,
    HumanSessionRequest,
)
from ...control.arm import ArmController
from ...control.wheel import WheelController
from ...planning.ik import sampled_path_collisions, solve_pose
from ...planning.trajectory import JointTrajectory, time_parameterize
from ...sensors.contacts import object_contacts
from ...sensors.scene import build_scene_manifest
from ..common import (
    EventPublisher,
    file_sha256,
    new_id,
    quat_angle,
    quat_wxyz_to_xyzw,
    utc_now,
)


REPO_ROOT = Path(__file__).resolve().parents[4]
ARM_JOINTS = [f"joint{i}" for i in range(1, 6)]


@dataclass
class _PreviewPlan:
    preview: PlanPreview
    trajectory: JointTrajectory
    endpoint_model_rad: np.ndarray
    start_model_rad: np.ndarray
    target_rotation_world: np.ndarray
    target_position_world: np.ndarray
    target_revision: int
    cancel_epoch: int
    base_position_world: np.ndarray
    base_rotation_world: np.ndarray
    created_monotonic_s: float
    expires_monotonic_s: float
    model_sha256: str
    profile_sha256: str
    backend_generation: int
    episode_id: str
    planning_metrics: dict[str, Any]


@dataclass
class _PlanningInput:
    target_position_world: np.ndarray
    target_rotation_world: np.ndarray
    qpos_snapshot: np.ndarray
    current_tcp_rotation: np.ndarray
    gripper_width_m: float
    grasp_contacts: dict[str, Any]
    task_phase: TaskPhase
    target_revision: int
    cancel_epoch: int
    base_position_world: np.ndarray
    base_rotation_world: np.ndarray
    base_request_body: np.ndarray
    control_tick_id: str
    backend_generation: int


class MujocoBackend:
    """A finite-force backend where only the physics thread calls ``mj_step``."""

    def __init__(
        self,
        backend_config: Mapping[str, Any],
        safety_config: Mapping[str, Any],
        acceptance_config: Mapping[str, Any],
        run_root: Path,
    ) -> None:
        self.backend_config = copy.deepcopy(dict(backend_config))
        self.safety_config = copy.deepcopy(dict(safety_config))
        self.acceptance_config = copy.deepcopy(dict(acceptance_config))
        self.run_root = Path(run_root).resolve()
        self.run_id = new_id("run")
        self.backend_generation = int(backend_config.get("generation", 1))
        self.episode_id = new_id("episode")
        self.episode_started = time.monotonic()
        profile_value = Path(
            backend_config.get(
                "observation_profile_path", "configs/profiles/g2a_rgbd_5hz.yaml"
            )
        )
        self.observation_profile_path = (
            profile_value if profile_value.is_absolute() else REPO_ROOT / profile_value
        ).resolve()
        self.observation_profile = yaml.safe_load(
            self.observation_profile_path.read_text(encoding="utf-8")
        )
        if self.observation_profile.get("interface_profile") not in {
            "g2a-1.0",
            "g4a1-1.0",
        }:
            raise ValueError("G2A/G4A-1 observation profile is required")
        self.observation_profile_sha256 = file_sha256(self.observation_profile_path)
        self.g4a1_config = copy.deepcopy(dict(backend_config.get("g4a1", {})))
        self.g4a1_enabled = bool(self.g4a1_config.get("enabled", False))
        self.action_profile: dict[str, Any] = {}
        self.action_profile_sha256: str | None = None
        if self.g4a1_enabled:
            action_profile_path = REPO_ROOT / str(
                self.g4a1_config["action_profile_path"]
            )
            self.action_profile = yaml.safe_load(
                action_profile_path.read_text(encoding="utf-8")
            )
            if self.action_profile.get("interface_profile") != "g4a1-1.0":
                raise ValueError("G4A-1 action profile is required")
            self.action_profile_sha256 = file_sha256(action_profile_path)
        navigation_profile_value = backend_config.get("navigation_profile_path")
        self.navigation_profile_path: Path | None = None
        self.navigation_profile: dict[str, Any] | None = None
        self.navigation_profile_sha256: str | None = None
        if navigation_profile_value:
            navigation_path = Path(str(navigation_profile_value))
            self.navigation_profile_path = (
                navigation_path
                if navigation_path.is_absolute()
                else REPO_ROOT / navigation_path
            ).resolve()
            self.navigation_profile = yaml.safe_load(
                self.navigation_profile_path.read_text(encoding="utf-8")
            )
            if self.navigation_profile.get("interface_profile") != "g3a-1.0":
                raise ValueError("G3A navigation profile is required")
            self.navigation_profile_sha256 = file_sha256(
                self.navigation_profile_path
            )
        self.navigation_enabled = self.navigation_profile is not None
        self._rgbd_period_s = 1.0 / float(
            self.observation_profile["image"]["nominal_capture_hz"]
        )
        self._rgbd_deadline = time.monotonic() + self._rgbd_period_s
        self._rgbd_schedule_index = 0
        self._rgbd_skip_reasons: Counter[str] = Counter()
        self._rgbd_skipped_slots = 0
        self._last_capture_metric: dict[str, Any] = {}
        self._last_planning_metric: dict[str, Any] = {}
        self.lock = threading.RLock()
        self._navigation_health_lock = threading.RLock()
        self._navigation_control_lock = threading.RLock()
        self._pending_g3b_action: dict[str, Any] | None = None
        self._g3b_control_sequence = 0
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None
        self.sensor_thread: threading.Thread | None = None
        self.closed = False
        self.publisher = EventPublisher(self.run_id, lambda: self.episode_id)
        self._command_cache: dict[str, CommandResponse] = {}
        self._session_last_seq: dict[str, int] = {}
        self._owners: dict[str, str | None] = {"base": None, "arm": None}
        self._owner_kind: dict[str, OperatorKind | None] = {"base": None, "arm": None}
        self._owner_operator: dict[str, str | None] = {"base": None, "arm": None}
        self._lease_deadline: dict[str, float] = {"base": 0.0, "arm": 0.0}
        self._base_request = np.zeros(3)
        self._stop_latched = False
        self._episode_timed_out = False
        self._stopped_since: float | None = None
        self._stopped = True
        self._task_phase = TaskPhase.OBSERVE
        self._last_unexpected: list[dict[str, Any]] = []
        self._last_tick_id = "tick_0"
        self._tick_count = 0
        self._feedback_time = time.monotonic()
        self._sensor_states: dict[SensorKind, SensorState] = {
            kind: SensorState(kind=kind, available=False, fresh=False)
            for kind in SensorKind
        }
        self._g4a1_epoch = 1
        self._g4a1_permission: dict[str, Any] | None = None
        self._g4a1_session_last_seq: dict[str, int] = {}
        self._g4a1_last_safe_target: list[float] | None = None
        self._g4a1_pending_reference: dict[str, Any] | None = None
        self._g4a1_faults: list[str] = []
        self._g4a1_observation_cache: dict[str, G4A1Observation] = {}
        self._g4a2_human_sessions: dict[str, dict[str, Any]] = {}
        self._g4a2_intents: dict[str, dict[str, Any]] = {}
        self._g4a2_intent_seq = 0
        self._latest_imu: dict[str, list[float]] = {
            "gyro_rad_s": [0.0, 0.0, 0.0],
            "acceleration_m_s2": [0.0, 0.0, 0.0],
        }
        self._latest_imu_valid = {"gyro": False, "acceleration": False}
        self._latest_laser: dict[str, Any] = {
            "angle_min_rad": -math.pi,
            "angle_max_rad": math.pi,
            "angle_increment_rad": math.pi / 180.0,
            "range_max_m": 4.0,
            "ranges_m": [float("nan")] * 361,
        }
        self._latest_rgbd_metadata: dict[str, Any] = {}
        self._previews: dict[str, _PreviewPlan] = {}
        self._active_plan: _PreviewPlan | None = None
        self._active_command: CommandRequest | None = None
        self._active_plan_started_monotonic = 0.0
        self._active_plan_started_sim = 0.0
        self._reached_since: float | None = None
        self._target_revision = 0
        self._cancel_epoch = 0
        self._navigation_epoch = int(
            (self.navigation_profile or {}).get("epochs", {}).get(
                "navigation_epoch_initial", 0
            )
        )
        self._navigation_sequence = 0
        self._navigation_next_publish_sim_s = 0.02
        self._navigation_next_scan_sim_s = 0.10
        self._navigation_session_id: str | None = None
        self._authorized_g3b_goal: dict[str, Any] | None = None
        self._last_g3b_goal_generation = 0
        self._navigation_revoked_sessions: set[str] = set()
        self._navigation_cancel_epoch = 0
        self._navigation_health_stop = False
        self._navigation_faults: set[str] = set()
        self._navigation_control_sequence: dict[str, int] = {}
        self._navigation_references: OrderedDict[str, dict[str, Any]] = OrderedDict()
        self._navigation_last_health: dict[str, Any] = {
            "status": "UNKNOWN",
            "stop_required": False,
        }
        self._renderer: mujoco.Renderer | None = None
        self._sensor_error: str | None = None
        self._sensor_commit_lock = threading.Lock()
        self._sensor_capture_enabled = threading.Event()
        self._sensor_capture_enabled.set()

        model_path = Path(self.backend_config["model_path"])
        self.model_path = model_path if model_path.is_absolute() else REPO_ROOT / model_path
        self.model_path = self.model_path.resolve()
        if not self.model_path.is_file():
            raise FileNotFoundError(f"MuJoCo model not found: {self.model_path}")
        from ..modelgen.parameters import load_parameters

        self.params = load_parameters()
        if self.params["simulation"]["fixed_arm"]:
            raise ValueError("G1 requires the dynamic-arm model; fixed_arm must be false")
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        self.arm_spec = json.loads((self.model_path.parent / "arm_spec.json").read_text(encoding="utf-8"))
        if not self.arm_spec.get("dynamic"):
            raise ValueError("G1 arm_spec must be dynamic")
        self.wheel = WheelController(self.model, self.params)
        self.arm = ArmController(self.model, self.params, self.arm_spec)
        self.arm_controller = self.arm
        self.base_id = self.model.body("base_link").id
        self.tcp_id = self.model.site("grasp_tcp").id
        self.camera_id = self.model.camera("rgbd").id
        try:
            self.target_body_id: int | None = self.model.body("target_cube").id
            self.target_joint_id: int | None = int(
                self.model.body_jntadr[self.target_body_id]
            )
            self.target_qpos: int | None = int(
                self.model.jnt_qposadr[self.target_joint_id]
            )
        except Exception:
            self.target_body_id = None
            self.target_joint_id = None
            self.target_qpos = None
        self.base_qpos = int(
            self.model.jnt_qposadr[self.model.joint("base_free").id]
        )
        self.link4_id = self.model.body("link4").id
        self.camera_body_id = int(self.model.cam_bodyid[self.camera_id])
        if not self._camera_is_link4_branch():
            raise ValueError("camera must be mounted on the link4 branch")
        self.model_sha256 = file_sha256(self.model_path)
        if self.model_sha256 != self.observation_profile.get("model_sha256"):
            raise ValueError("observation profile model_sha256 does not match compiled model")
        if (
            self.navigation_profile is not None
            and self.model_sha256 != self.navigation_profile.get("model_sha256")
        ):
            raise ValueError("navigation profile model_sha256 does not match compiled model")
        camera_config = self.params["simulation"]["camera"]
        profile_image = self.observation_profile["image"]
        if (
            int(camera_config["width"]),
            int(camera_config["height"]),
        ) != (
            int(profile_image["width"]),
            int(profile_image["height"]),
        ):
            raise ValueError("observation profile resolution differs from camera parameters")
        if not math.isclose(
            float(camera_config["vertical_fov_deg"]),
            float(profile_image["vertical_fov_deg"]),
        ):
            raise ValueError("observation profile vertical FOV differs from camera parameters")
        self.model_id = f"landerpi_mecanum_dynamic_5dof_{self.model_sha256[:12]}"
        self.scene_id = Path(self.backend_config.get("scene_path", "mobile_pick.xml")).stem
        self.scene = build_scene_manifest(
            self.model,
            self.model_path,
            model_id=self.model_id,
            scene_id=self.scene_id,
        )
        self._scene_assets = self._load_scene_assets()
        self._capabilities = self._build_capabilities()
        self._incoming_root = self.run_root / self.run_id / "_incoming" / self.episode_id
        self._incoming_root.mkdir(parents=True, exist_ok=True)
        self._reset_physics(new_episode=False)
        self._cached_snapshot = self._build_snapshot()

    def set_event_sink(self, sink) -> None:
        self.publisher.set_sink(sink)

    def capabilities(self) -> Capabilities:
        return self._capabilities.model_copy(deep=True)

    def scene_manifest(self) -> SceneManifest:
        return self.scene.model_copy(deep=True)

    def scene_asset(self, asset_id: str) -> bytes:
        path = self._scene_assets.get(asset_id)
        if path is None:
            raise KeyError(asset_id)
        return path.read_bytes()

    def confirm_human_session(
        self, request: HumanSessionRequest
    ) -> HumanSessionReceipt:
        received = time.monotonic()
        with self.lock:
            reason = ""
            if request.run_id != self.run_id:
                reason = "run_id does not match backend"
            elif request.episode_id != self.episode_id:
                reason = "human session belongs to another Episode"
            elif request.backend_generation != self.backend_generation:
                reason = "stale backend_generation"
            elif not request.explicit_human_confirmation:
                reason = "explicit human confirmation is required"
            if reason:
                return HumanSessionReceipt(
                    accepted=False,
                    reason=reason,
                    run_id=request.run_id,
                    episode_id=request.episode_id,
                    backend_generation=request.backend_generation,
                    session_id=request.session_id,
                    operator_id=request.operator_id,
                    received_backend_monotonic_s=received,
                )
            human_session_id = new_id("human_session")
            self._g4a2_human_sessions[human_session_id] = {
                "human_session_id": human_session_id,
                "run_id": request.run_id,
                "episode_id": request.episode_id,
                "backend_generation": request.backend_generation,
                "session_id": request.session_id,
                "operator_id": request.operator_id,
                "operator_kind": request.operator_kind,
                "source": request.source,
                "view_source": request.view_source,
                "input_device": request.input_device,
                "assist_mode": request.assist_mode,
                "confirmed_backend_monotonic_s": received,
                "active": True,
            }
            self.publisher.emit(
                "human_session_confirmed",
                self._g4a2_human_sessions[human_session_id],
                sim_time_s=float(self.data.time),
                control_tick_id=self._last_tick_id,
            )
            return HumanSessionReceipt(
                accepted=True,
                human_session_id=human_session_id,
                run_id=request.run_id,
                episode_id=request.episode_id,
                backend_generation=request.backend_generation,
                session_id=request.session_id,
                operator_id=request.operator_id,
                received_backend_monotonic_s=received,
            )

    def record_human_intent(
        self, request: HumanIntentRequest
    ) -> HumanIntentReceipt:
        received = time.monotonic()
        with self.lock:
            session = self._g4a2_human_sessions.get(request.human_session_id)
            reason = ""
            if session is None or not session.get("active", False):
                reason = "human session is not active"
            elif request.run_id != self.run_id:
                reason = "run_id does not match backend"
            elif request.episode_id != self.episode_id:
                reason = "human intent belongs to another Episode"
            elif request.backend_generation != self.backend_generation:
                reason = "stale backend_generation"
            elif request.session_id != session["session_id"]:
                reason = "human session mismatch"
            elif request.display_sample_id not in self._g4a1_observation_cache:
                reason = "display sample is not known"
            if reason:
                return HumanIntentReceipt(
                    accepted=False,
                    reason=reason,
                    human_session_id=request.human_session_id,
                    display_sample_id=request.display_sample_id,
                    server_receive_backend_monotonic_s=received,
                )
            self._g4a2_intent_seq += 1
            intent_id = new_id("human_intent")
            record = {
                "human_intent_id": intent_id,
                "human_event_seq": self._g4a2_intent_seq,
                "human_session_id": request.human_session_id,
                "run_id": request.run_id,
                "episode_id": request.episode_id,
                "backend_generation": request.backend_generation,
                "session_id": request.session_id,
                "display_sample_id": request.display_sample_id,
                "input_type": request.input_type,
                "assist_mode": request.assist_mode,
                "control_generator": request.control_generator,
                "client_display_monotonic_ns": request.client_display_monotonic_ns,
                "client_intent_monotonic_ns": request.client_intent_monotonic_ns,
                "server_receive_backend_monotonic_s": received,
                "raw_input": request.raw_input,
            }
            self._g4a2_intents[intent_id] = record
            self.publisher.emit(
                "human_intent",
                record,
                sim_time_s=float(self.data.time),
                control_tick_id=self._last_tick_id,
            )
            return HumanIntentReceipt(
                accepted=True,
                human_intent_id=intent_id,
                human_event_seq=self._g4a2_intent_seq,
                human_session_id=request.human_session_id,
                display_sample_id=request.display_sample_id,
                server_receive_backend_monotonic_s=received,
                client_display_monotonic_ns=request.client_display_monotonic_ns,
                client_intent_monotonic_ns=request.client_intent_monotonic_ns,
            )

    def submit_g4a2_action(self, request: G4A2ActionRequest) -> G4A2ActionReceipt:
        requested = time.monotonic()
        with self.lock:
            session = self._g4a2_human_sessions.get(request.human_session_id)
            intent = self._g4a2_intents.get(request.human_intent_id)
            reason = ""
            if session is None or not session.get("active", False):
                reason = "human session is not active"
            elif intent is None or intent.get("human_session_id") != request.human_session_id:
                reason = "human intent does not bind this session"
            elif request.display_sample_id != intent.get("display_sample_id"):
                reason = "display sample does not bind human intent"
            elif request.teacher_plan_id is not None:
                reason = "teacher_plan_id is forbidden on ordinary human actions"
            elif request.observation_sample_id not in self._g4a1_observation_cache:
                reason = "observation sample is not known"
            if reason:
                return G4A2ActionReceipt(
                    accepted=False,
                    reason=reason,
                    human_intent_id=request.human_intent_id,
                    action_frame_id=request.action_frame_id,
                    action_seq=request.action_seq,
                    human_session_id=request.human_session_id,
                    requested_backend_monotonic_s=requested,
                    decision_backend_monotonic_s=requested,
                )
            observation = self._g4a1_observation_cache[request.observation_sample_id]
            base = G4A1ActionRequest(
                run_id=request.run_id,
                episode_id=request.episode_id,
                backend_generation=request.backend_generation,
                nearfield_epoch=request.nearfield_epoch,
                permission_id=request.permission_id,
                session_id=request.session_id,
                action_frame_id=request.action_frame_id,
                action_seq=request.action_seq,
                observation_sample_id=request.observation_sample_id,
                observation_capture_control_tick_id=observation.capture_control_tick_id,
                observation_arrival_backend_monotonic_s=observation.arrival_backend_monotonic_s,
                decision_backend_monotonic_s=requested,
                issued_backend_monotonic_s=requested,
                expires_backend_monotonic_s=requested + 0.4,
                observation_profile_sha256=observation.observation_profile_sha256,
                action_profile_sha256=observation.action_profile_sha256,
                source="human_teleop",
                teacher_plan_id=None,
                joint_order=request.joint_order,
                units=request.units,
                arm_user_joint_positions_rad=request.arm_user_joint_positions_rad,
                gripper_width_m=request.gripper_width_m,
            )
            base_receipt = self.submit_g4a1_action(base)
            receipt = G4A2ActionReceipt(
                accepted=base_receipt.accepted,
                reason=base_receipt.reason,
                human_intent_id=request.human_intent_id,
                action_frame_id=request.action_frame_id,
                action_seq=request.action_seq,
                human_session_id=request.human_session_id,
                requested_backend_monotonic_s=requested,
                decision_backend_monotonic_s=requested,
                dispatch_backend_monotonic_s=base_receipt.dispatch_backend_monotonic_s,
                control_tick_id=base_receipt.control_tick_id,
                target=base_receipt.target,
            )
            self.publisher.emit(
                "g4a2_action_linked",
                {
                    "human_intent_id": request.human_intent_id,
                    "action_frame_id": request.action_frame_id,
                    "action_seq": request.action_seq,
                    "human_session_id": request.human_session_id,
                    "display_sample_id": request.display_sample_id,
                    "receipt": receipt.model_dump(mode="json"),
                },
                sim_time_s=float(self.data.time),
                control_tick_id=self._last_tick_id,
            )
            return receipt

    def g4a1_status(self) -> G4A1Status:
        permission = self._g4a1_permission
        return G4A1Status(
            enabled=self.g4a1_enabled,
            run_id=self.run_id,
            episode_id=self.episode_id,
            backend_generation=self.backend_generation,
            nearfield_epoch=self._g4a1_epoch,
            permission_active=permission is not None,
            permission_id=None if permission is None else permission["permission_id"],
            session_id=None if permission is None else permission["session_id"],
            stopped=bool(self._stopped),
            last_action_seq=(
                0
                if permission is None
                else int(self._g4a1_session_last_seq.get(permission["session_id"], 0))
            ),
            faults=list(self._g4a1_faults),
        )

    def authorize_nearfield(
        self, request: NearfieldPermissionRequest
    ) -> NearfieldPermissionReceipt:
        requested = time.monotonic()
        with self.lock:
            reason = ""
            if not self.g4a1_enabled:
                reason = "G4A-1 is disabled"
            elif request.run_id != self.run_id:
                reason = "run_id does not match backend"
            elif request.backend_generation != self.backend_generation:
                reason = "stale backend_generation"
            elif request.episode_id != self.episode_id:
                reason = "permission belongs to another Episode"
            elif request.observation_profile_sha256 != self.observation_profile_sha256:
                reason = "observation profile hash mismatch"
            elif request.action_profile_sha256 != self.action_profile_sha256:
                reason = "action profile hash mismatch"
            elif not self._stopped:
                reason = "base has not completed the prepared-pose stop gate"
            elif self._navigation_session_id is not None or self._authorized_g3b_goal is not None:
                reason = "navigation authority is active"
            elif self._recording_required():
                reason = "recording gate is not ready"
            elif self._stop_latched:
                reason = "software stop is latched"
            if reason:
                self.publisher.emit(
                    "g4a1_permission_rejected",
                    {"reason": reason, "request": request.model_dump(mode="json")},
                    sim_time_s=float(self.data.time),
                    control_tick_id=self._last_tick_id,
                )
                return NearfieldPermissionReceipt(
                    accepted=False,
                    reason=reason,
                    run_id=request.run_id,
                    episode_id=request.episode_id,
                    backend_generation=request.backend_generation,
                    nearfield_epoch=self._g4a1_epoch,
                    session_id=request.session_id,
                    requested_backend_monotonic_s=requested,
                )
            self._g4a1_epoch += 1
            permission_id = new_id("permission")
            expires = requested + float(
                self.action_profile.get("permission_ttl_s", 30.0)
            )
            self._g4a1_permission = {
                "permission_id": permission_id,
                "run_id": request.run_id,
                "episode_id": request.episode_id,
                "backend_generation": request.backend_generation,
                "nearfield_epoch": self._g4a1_epoch,
                "session_id": request.session_id,
                "operator_id": request.operator_id,
                "operator_kind": request.operator_kind,
                "source": request.source,
                "view_source": request.view_source,
                "observation_profile_sha256": request.observation_profile_sha256,
                "action_profile_sha256": request.action_profile_sha256,
                "requested_backend_monotonic_s": requested,
                "granted_backend_monotonic_s": requested,
                "expires_backend_monotonic_s": expires,
            }
            self._g4a1_faults.clear()
            self._owners["arm"] = request.session_id
            self._owner_kind["arm"] = OperatorKind(request.operator_kind)
            self._owner_operator["arm"] = request.operator_id
            self._lease_deadline["arm"] = expires
            self._g4a1_session_last_seq[request.session_id] = 0
            self.publisher.emit(
                "g4a1_permission_granted",
                self._g4a1_permission,
                sim_time_s=float(self.data.time),
                control_tick_id=self._last_tick_id,
            )
            return NearfieldPermissionReceipt(
                accepted=True,
                permission_id=permission_id,
                run_id=request.run_id,
                episode_id=request.episode_id,
                backend_generation=request.backend_generation,
                nearfield_epoch=self._g4a1_epoch,
                session_id=request.session_id,
                observation_profile_sha256=request.observation_profile_sha256,
                action_profile_sha256=request.action_profile_sha256,
                requested_backend_monotonic_s=requested,
                granted_backend_monotonic_s=requested,
                expires_backend_monotonic_s=expires,
            )

    def revoke_nearfield(self, reason: str, source: str) -> dict[str, Any]:
        with self.lock:
            self._revoke_nearfield(reason, source)
            return {
                "accepted": True,
                "reason": reason,
                "nearfield_epoch": self._g4a1_epoch,
                "sim_time_s": float(self.data.time),
                "control_tick_id": self._last_tick_id,
            }

    def set_g4a1_fault(self, fault: str, active: bool) -> dict[str, Any]:
        if not self.g4a1_enabled:
            raise ValueError("G4A-1 is disabled")
        with self.lock:
            if active and fault not in self._g4a1_faults:
                self._g4a1_faults.append(fault)
            if not active:
                self._g4a1_faults = [item for item in self._g4a1_faults if item != fault]
            if active:
                self._revoke_nearfield(f"g4a1_fault:{fault}", "test_injection")
            return {
                "fault": fault,
                "active": active,
                "nearfield_epoch": self._g4a1_epoch,
                "permission_active": self._g4a1_permission is not None,
            }

    def mark_g4a1_phase(self, phase: str) -> dict[str, Any]:
        allowed = {
            "OBSERVE",
            "READY",
            "APPROACH",
            "CLOSE",
            "LIFT",
            "HOLD",
            "DONE",
            "ABORTED",
        }
        if phase not in allowed:
            raise ValueError(f"unsupported G4A-1 phase: {phase}")
        with self.lock:
            if phase in {"APPROACH", "CLOSE", "LIFT", "HOLD"}:
                self._task_phase = TaskPhase.MANIPULATE
            elif phase == "DONE":
                self._task_phase = TaskPhase.VERIFY
            elif phase in {"OBSERVE", "READY"}:
                self._task_phase = TaskPhase.OBSERVE
            self.publisher.emit(
                "g4a1_phase",
                {"phase": phase, "nearfield_epoch": self._g4a1_epoch},
                sim_time_s=float(self.data.time),
                control_tick_id=self._last_tick_id,
            )
            return {
                "accepted": True,
                "phase": phase,
                "nearfield_epoch": self._g4a1_epoch,
                "sim_time_s": float(self.data.time),
            }

    def g4a1_observation(self) -> G4A1Observation:
        with self.lock:
            metadata = dict(self._latest_rgbd_metadata)
            timing = dict(metadata.get("timing") or {})
            sample_id = str(metadata.get("sample_id") or "")
            if not sample_id:
                state = self._sensor_states[SensorKind.RGB]
                sample_id = str(state.sample_id or "")
            cached = self._g4a1_observation_cache.get(sample_id)
            if cached is not None:
                return cached.model_copy(
                    deep=True,
                    update={"observation_backend_monotonic_s": time.monotonic()},
                )
            required = ("rgb_path", "depth_path", "valid_mask_path")
            if not sample_id or any(not metadata.get(key) for key in required):
                raise ValueError("recorded link4 RGB-D observation is not available")
            paths = {
                key: (self.run_root / str(metadata[key])).resolve()
                for key in required
            }
            if any(not path.is_file() or not path.is_relative_to(self.run_root) for path in paths.values()):
                raise ValueError("recorded RGB-D asset is missing")
            depth = np.load(paths["depth_path"])
            center_y, center_x = depth.shape[0] // 2, depth.shape[1] // 2
            center = float(depth[center_y, center_x])
            snapshot = self._build_snapshot()
            camera = self.params["simulation"]["camera"]
            fy = 0.5 * float(camera["height"]) / math.tan(
                math.radians(float(camera["vertical_fov_deg"])) / 2.0
            )
            observation = G4A1Observation(
                run_id=self.run_id,
                episode_id=self.episode_id,
                backend_generation=self.backend_generation,
                nearfield_epoch=self._g4a1_epoch,
                sample_id=sample_id,
                capture_control_tick_id=str(timing["capture_control_tick_id"]),
                capture_sim_time_s=float(timing["capture_sim_time_s"]),
                capture_backend_monotonic_s=float(timing["capture_backend_monotonic_s"]),
                arrival_backend_monotonic_s=float(timing["arrival_backend_monotonic_s"]),
                observation_backend_monotonic_s=time.monotonic(),
                rgb_uri=f"/api/v1/g4a1/rgbd/{sample_id}/rgb",
                depth_uri=f"/api/v1/g4a1/rgbd/{sample_id}/depth",
                valid_mask_uri=f"/api/v1/g4a1/rgbd/{sample_id}/mask",
                rgb_sha256=file_sha256(paths["rgb_path"]),
                depth_sha256=file_sha256(paths["depth_path"]),
                valid_mask_sha256=file_sha256(paths["valid_mask_path"]),
                valid_fraction=float(metadata["valid_fraction"]),
                center_depth_m=center if math.isfinite(center) else None,
                arm_user_joint_positions_rad=list(snapshot.arm.user_joint_positions_rad),
                arm_user_joint_velocities_rad_s=list(snapshot.arm.joint_velocities_rad_s),
                gripper_width_m=float(snapshot.gripper.width_m),
                camera_intrinsics={
                    "width": int(camera["width"]),
                    "height": int(camera["height"]),
                    "fx": fy,
                    "fy": fy,
                    "cx": float(camera["width"]) / 2.0,
                    "cy": float(camera["height"]) / 2.0,
                    "optical_convention": "X_right_Y_down_Z_forward",
                },
                T_base_camera=metadata["T_base_camera_render"],
                T_base_link4=metadata["T_base_link4"],
                T_base_tcp=metadata["T_base_tcp"],
                observation_profile_sha256=self.observation_profile_sha256,
                action_profile_sha256=str(self.action_profile_sha256),
                quality="simulated_nominal" if not self._g4a1_faults else "FAULT_INJECTED",
            )
            self._g4a1_observation_cache[sample_id] = observation
            if len(self._g4a1_observation_cache) > 16:
                oldest = min(
                    self._g4a1_observation_cache.values(),
                    key=lambda item: item.capture_backend_monotonic_s,
                )
                self._g4a1_observation_cache.pop(oldest.sample_id, None)
            return observation.model_copy(deep=True)

    def g4a1_asset(self, sample_id: str, kind: str) -> Path:
        mapping = {
            "rgb": ("rgb_path", "image/png"),
            "depth": ("depth_path", "application/octet-stream"),
            "mask": ("valid_mask_path", "image/png"),
        }
        if kind not in mapping:
            raise KeyError(kind)
        metadata = self._latest_rgbd_metadata
        if self._sensor_states[SensorKind.RGB].sample_id != sample_id:
            raise KeyError(sample_id)
        relative = metadata.get(mapping[kind][0])
        if not relative:
            raise KeyError(kind)
        path = (self.run_root / str(relative)).resolve()
        if not path.is_file() or not path.is_relative_to(self.run_root):
            raise KeyError(kind)
        return path

    def submit_g4a1_action(self, request: G4A1ActionRequest) -> G4A1ActionReceipt:
        requested = time.monotonic()
        with self.lock:
            reason = ""
            permission = self._g4a1_permission
            metadata = self._latest_rgbd_metadata
            if not self.g4a1_enabled:
                reason = "G4A-1 is disabled"
            elif permission is None:
                reason = "nearfield permission is not active"
            elif self._g4a1_faults:
                reason = "nearfield fault requires a new permission"
            elif request.run_id != self.run_id:
                reason = "run_id does not match backend"
            elif request.episode_id != self.episode_id:
                reason = "action belongs to another Episode"
            elif request.backend_generation != self.backend_generation:
                reason = "stale backend_generation"
            elif request.nearfield_epoch != permission["nearfield_epoch"]:
                reason = "stale nearfield_epoch"
            elif request.permission_id != permission["permission_id"]:
                reason = "permission_id mismatch"
            elif request.session_id != permission["session_id"]:
                reason = "session_id does not own nearfield permission"
            elif requested > float(permission["expires_backend_monotonic_s"]):
                reason = "nearfield permission expired"
            elif request.observation_profile_sha256 != self.observation_profile_sha256:
                reason = "observation profile hash mismatch"
            elif request.action_profile_sha256 != self.action_profile_sha256:
                reason = "action profile hash mismatch"
            elif request.action_seq <= self._g4a1_session_last_seq.get(request.session_id, 0):
                reason = "older or duplicate action_seq"
            elif request.observation_sample_id not in self._g4a1_observation_cache:
                reason = "action observation sample is not known"
            elif abs(
                request.observation_arrival_backend_monotonic_s
                - self._g4a1_observation_cache[
                    request.observation_sample_id
                ].arrival_backend_monotonic_s
            ) > 1e-6:
                reason = "action observation arrival clock mismatch"
            elif requested > request.expires_backend_monotonic_s:
                reason = "action expired before dispatch"
            elif request.issued_backend_monotonic_s > requested + 1e-3:
                reason = "action issue clock is in the future"
            elif request.decision_backend_monotonic_s < request.observation_arrival_backend_monotonic_s:
                reason = "decision precedes observation arrival"
            elif requested - request.decision_backend_monotonic_s > 0.4:
                reason = "decision too stale for dispatch"
            elif requested - float(
                self._g4a1_observation_cache[
                    request.observation_sample_id
                ].capture_backend_monotonic_s
            ) > 0.4:
                reason = "observation capture exceeded action age"
            if not reason:
                arm = np.asarray(request.arm_user_joint_positions_rad, dtype=float)
                gripper = self.arm_spec["gripper"]
                if np.any(arm < self.arm.user_limits[:, 0]) or np.any(arm > self.arm.user_limits[:, 1]):
                    reason = "joint target violates user limits"
                elif not float(gripper["min_width_m"]) <= request.gripper_width_m <= float(gripper["max_width_m"]):
                    reason = "gripper width outside limits"
            if reason:
                receipt = G4A1ActionReceipt(
                    accepted=False,
                    reason=reason,
                    action_frame_id=request.action_frame_id,
                    action_seq=request.action_seq,
                    permission_id=request.permission_id,
                    run_id=request.run_id,
                    episode_id=request.episode_id,
                    backend_generation=request.backend_generation,
                    nearfield_epoch=request.nearfield_epoch,
                    session_id=request.session_id,
                    observation_sample_id=request.observation_sample_id,
                    requested_backend_monotonic_s=requested,
                )
                self.publisher.emit(
                    "g4a1_action_rejected",
                    receipt.model_dump(mode="json"),
                    sim_time_s=float(self.data.time),
                    control_tick_id=self._last_tick_id,
                )
                return receipt
            self._cancel_arm_motion("g4a1_action_takeover")
            active_plan = None
            if request.teacher_plan_id:
                active_plan = self._previews.get(request.teacher_plan_id)
                if active_plan is None:
                    reason = "teacher_plan_id is unknown or expired"
                elif (
                    active_plan.episode_id != self.episode_id
                    or active_plan.backend_generation != self.backend_generation
                    or active_plan.model_sha256 != self.model_sha256
                    or active_plan.profile_sha256 != self.observation_profile_sha256
                ):
                    reason = "teacher plan identity is stale"
                elif (
                    active_plan.target_revision != self._target_revision
                    or active_plan.cancel_epoch != self._cancel_epoch
                ):
                    reason = "teacher plan revision or cancel epoch is stale"
                elif requested > active_plan.expires_monotonic_s:
                    reason = "teacher plan expired"
                else:
                    expected_user = self.arm.coordinates.to_user(
                        active_plan.endpoint_model_rad
                    )
                    if np.max(
                        np.abs(
                            expected_user
                            - np.asarray(request.arm_user_joint_positions_rad)
                        )
                    ) > 0.01:
                        reason = "six-dimensional target differs from teacher plan endpoint"
                if reason:
                    receipt = G4A1ActionReceipt(
                        accepted=False,
                        reason=reason,
                        action_frame_id=request.action_frame_id,
                        action_seq=request.action_seq,
                        permission_id=request.permission_id,
                        run_id=request.run_id,
                        episode_id=request.episode_id,
                        backend_generation=request.backend_generation,
                        nearfield_epoch=request.nearfield_epoch,
                        session_id=request.session_id,
                        observation_sample_id=request.observation_sample_id,
                        requested_backend_monotonic_s=requested,
                    )
                    self.publisher.emit(
                        "g4a1_action_rejected",
                        receipt.model_dump(mode="json"),
                        sim_time_s=float(self.data.time),
                        control_tick_id=self._last_tick_id,
                    )
                    return receipt
                self.arm.set_user_goal(
                    arm=np.asarray(request.arm_user_joint_positions_rad, dtype=float),
                    width=request.gripper_width_m,
                )
                self.arm.follow_trajectory(active_plan.trajectory, self.data)
                self._active_plan = active_plan
                self._active_command = None
                self._active_plan_started_monotonic = time.monotonic()
                self._active_plan_started_sim = float(self.data.time)
                self._reached_since = None
            else:
                self.arm.set_user_goal(
                    arm=np.asarray(request.arm_user_joint_positions_rad, dtype=float),
                    width=request.gripper_width_m,
                )
            self._g4a1_last_safe_target = request.target
            self._g4a1_session_last_seq[request.session_id] = request.action_seq
            self._lease_deadline["arm"] = requested + float(
                self.action_profile.get("permission_ttl_s", 120.0)
            )
            permission["expires_backend_monotonic_s"] = requested + float(
                self.action_profile.get("permission_ttl_s", 120.0)
            )
            approved = time.monotonic()
            receipt = G4A1ActionReceipt(
                accepted=True,
                action_frame_id=request.action_frame_id,
                action_seq=request.action_seq,
                permission_id=request.permission_id,
                run_id=request.run_id,
                episode_id=request.episode_id,
                backend_generation=request.backend_generation,
                nearfield_epoch=request.nearfield_epoch,
                session_id=request.session_id,
                observation_sample_id=request.observation_sample_id,
                requested_backend_monotonic_s=requested,
                approved_backend_monotonic_s=approved,
                dispatch_backend_monotonic_s=approved,
                control_tick_id=self._last_tick_id,
                target=request.target,
            )
            self.publisher.emit(
                "g4a1_action_approved",
                {"request": request.model_dump(mode="json"), "receipt": receipt.model_dump(mode="json")},
                sim_time_s=float(self.data.time),
                control_tick_id=self._last_tick_id,
            )
            self._g4a1_pending_reference = receipt.model_dump(mode="json")
            return receipt

    def _revoke_nearfield(self, reason: str, source: str) -> None:
        permission = self._g4a1_permission
        if permission is None:
            return
        self._cancel_arm_motion(reason)
        if self._g4a1_last_safe_target is None:
            self._g4a1_last_safe_target = [
                *self.arm.coordinates.to_user(self.data.qpos[self.arm.qpos[:5]]).tolist(),
                self._snapshot_gripper_width(),
            ]
        self._g4a1_epoch += 1
        self._g4a1_permission = None
        for human_session_id, human_session in self._g4a2_human_sessions.items():
            if human_session.get("active", False):
                human_session["active"] = False
                human_session["revoked_reason"] = reason
                self.publisher.emit(
                    "human_session_revoked",
                    {
                        "human_session_id": human_session_id,
                        "reason": reason,
                        "source": source,
                    },
                    sim_time_s=float(self.data.time),
                    control_tick_id=self._last_tick_id,
                )
        if self._owners["arm"] == permission["session_id"]:
            self._owners["arm"] = None
            self._owner_kind["arm"] = None
            self._owner_operator["arm"] = None
            self._lease_deadline["arm"] = 0.0
        self.publisher.emit(
            "g4a1_permission_revoked",
            {
                "reason": reason,
                "source": source,
                "old_permission": permission,
                "nearfield_epoch": self._g4a1_epoch,
                "held_target": self._g4a1_last_safe_target,
            },
            sim_time_s=float(self.data.time),
            control_tick_id=self._last_tick_id,
        )

    def start(self) -> None:
        if self.thread is not None and self.thread.is_alive():
            return
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._physics_loop, name="mobilecar-physics", daemon=True)
        self.thread.start()
        self.sensor_thread = threading.Thread(target=self._sensor_loop, name="mobilecar-sensors", daemon=True)
        self.sensor_thread.start()
        self.publisher.emit(
            "backend_started",
            {
                "model_id": self.model_id,
                "model_sha256": self.model_sha256,
                "backend_generation": self.backend_generation,
                "camera_branch": "link4",
                "tcp_body": "guide_rod_link",
            },
        )

    def snapshot(self) -> RobotSnapshot:
        with self.lock:
            return self._cached_snapshot.model_copy(deep=True)

    def reset(self, request: CommandRequest) -> CommandResponse:
        return self._handle(request, lambda: self._reset_command(request))

    def renew_lease(self, session_id: str) -> bool:
        with self.lock:
            renewed = False
            now = time.monotonic()
            for channel in ("base", "arm"):
                if self._owners[channel] == session_id:
                    self._lease_deadline[channel] = now + float(
                        self.safety_config["lease_timeout_s"]
                    )
                    renewed = True
            self.publisher.emit(
                "lease_heartbeat",
                {"renewed": renewed, "session_id": session_id},
                sim_time_s=float(self.data.time),
                control_tick_id=self._last_tick_id,
            )
            return renewed

    def submit_navigation_control(
        self, message: NavigationControlMessage
    ) -> CommandResponse:
        """Submit one ROS navigation control through the command boundary."""

        with self.lock:
            sequence = self._navigation_control_sequence.get(message.session_id, 0) + 1
            self._navigation_control_sequence[message.session_id] = sequence
            provenance = {
                "navigation_source": "ros_navigation_test",
                "navigation_epoch": int(message.navigation_epoch),
                "source_session_id": message.session_id,
            }
            if message.message_type == "cmd_vel":
                command_type = CommandType.SET_BASE_TWIST
                payload = {
                    "frame_id": "base_link",
                    "vx_m_s": float(message.vx_m_s),
                    "vy_m_s": float(message.vy_m_s),
                    "wz_rad_s": float(message.wz_rad_s),
                    **provenance,
                }
            elif message.message_type == "release":
                command_type = CommandType.ZERO_BASE
                payload = {
                    **provenance,
                    "reason": message.reason or "navigation_release",
                }
            else:
                command_type = CommandType.STOP
                payload = {
                    **provenance,
                    "latch": True,
                    "reason": message.reason or "navigation_stop",
                }
            request = CommandRequest(
                run_id=message.run_id,
                backend_generation=message.backend_generation,
                episode_id=message.episode_id,
                session_id=message.session_id,
                operator_id="ros_navigation_test",
                operator_kind=OperatorKind.AUTONOMOUS_POLICY,
                view_source="ros_navigation_test",
                command_id=f"navigation_{uuid4().hex}",
                command_seq=sequence,
                command_type=command_type,
                requested_ttl_s=float(message.lease_ttl_s),
                payload=payload,
            )
            return self._handle(request, lambda: self._dispatch_command(request))

    def submit_g3b_control(self, message: G3BControlMessage) -> ControlReceipt:
        """Validate goal-bound control and return an actual dispatch receipt."""

        return self._submit_g3b_control_queued(message)

        received = True

        def receipt(
            *,
            authorized: bool,
            accepted: bool,
            backend_status: str,
            dispatch_state: str,
            reason: str,
            response: CommandResponse | None = None,
        ) -> ControlReceipt:
            return ControlReceipt(
                request_id=message.request_id,
                goal_id=message.goal_id,
                goal_generation=message.goal_generation,
                map_id=message.map_id,
                session_id=message.session_id,
                run_id=message.run_id,
                episode_id=message.episode_id,
                backend_generation=message.backend_generation,
                navigation_epoch=message.navigation_epoch,
                received=received,
                authorized=authorized,
                accepted=accepted,
                backend_status=backend_status,
                reason=reason,
                control_tick_id=(
                    response.control_tick_id if response is not None else self._last_tick_id
                ),
                backend_monotonic_s=(
                    response.backend_monotonic_s
                    if response is not None
                    else time.monotonic()
                ),
                sim_time_s=(
                    response.sim_time_s
                    if response is not None
                    else float(self.data.time)
                ),
                dispatch_state=dispatch_state,
            )

        with self.lock:
            if message.run_id != self.run_id:
                return receipt(
                    authorized=False,
                    accepted=False,
                    backend_status="REJECTED",
                    dispatch_state="REJECTED",
                    reason="run_id does not match backend",
                )
            if message.episode_id != self.episode_id:
                return receipt(
                    authorized=False,
                    accepted=False,
                    backend_status="REJECTED",
                    dispatch_state="REJECTED",
                    reason="goal belongs to another Episode",
                )
            if message.backend_generation != self.backend_generation:
                return receipt(
                    authorized=False,
                    accepted=False,
                    backend_status="REJECTED",
                    dispatch_state="REJECTED",
                    reason="stale backend_generation",
                )
            if message.navigation_epoch != self._navigation_epoch:
                return receipt(
                    authorized=False,
                    accepted=False,
                    backend_status="REJECTED",
                    dispatch_state="REJECTED",
                    reason="stale navigation_epoch",
                )

            if message.message_type == G3BControlType.AUTHORIZE:
                if self._stop_latched:
                    return receipt(
                        authorized=False,
                        accepted=False,
                        backend_status="REJECTED",
                        dispatch_state="REJECTED",
                        reason="software stop is latched",
                    )
                if self._navigation_health_stop:
                    return receipt(
                        authorized=False,
                        accepted=False,
                        backend_status="REJECTED",
                        dispatch_state="REJECTED",
                        reason="navigation health currently requires a stop",
                    )
                if self._recording_required():
                    return receipt(
                        authorized=False,
                        accepted=False,
                        backend_status="REJECTED",
                        dispatch_state="REJECTED",
                        reason="formal recording is required and not ready",
                    )
                if message.goal_generation <= self._last_g3b_goal_generation:
                    return receipt(
                        authorized=False,
                        accepted=False,
                        backend_status="REJECTED",
                        dispatch_state="REJECTED",
                        reason="stale or duplicate goal_generation",
                    )
                if self._authorized_g3b_goal is not None:
                    self._revoke_navigation("new_goal_authorization", zero_base=True)
                self._last_g3b_goal_generation = message.goal_generation
                self._authorized_g3b_goal = {
                    "goal_id": message.goal_id,
                    "goal_generation": message.goal_generation,
                    "map_id": message.map_id,
                    "run_id": message.run_id,
                    "episode_id": message.episode_id,
                    "backend_generation": message.backend_generation,
                    "navigation_epoch": message.navigation_epoch,
                    "session_id": message.session_id,
                    "authorization_request_id": message.request_id,
                    "authorized_backend_monotonic_s": time.monotonic(),
                }
                self._navigation_session_id = message.session_id
                self.publisher.emit(
                    "g3b_goal_authorized",
                    dict(self._authorized_g3b_goal),
                    sim_time_s=float(self.data.time),
                    control_tick_id=self._last_tick_id,
                )
                return receipt(
                    authorized=True,
                    accepted=True,
                    backend_status="APPLIED",
                    dispatch_state="AUTHORIZED",
                    reason="goal authorization accepted",
                )

            authorized = self._authorized_g3b_goal
            matches_authorization = bool(
                authorized is not None
                and authorized.get("goal_id") == message.goal_id
                and authorized.get("goal_generation") == message.goal_generation
                and authorized.get("map_id") == message.map_id
                and authorized.get("session_id") == message.session_id
            )
            if message.message_type in {
                G3BControlType.REVOKE,
                G3BControlType.RELEASE,
            }:
                if not matches_authorization:
                    self._base_request[:] = 0.0
                    return receipt(
                        authorized=False,
                        accepted=False,
                        backend_status="REJECTED",
                        dispatch_state="REJECTED",
                        reason="stale or mismatched goal revocation",
                    )
                self._authorized_g3b_goal = None
                self._revoke_navigation(
                    (
                        "goal_revoke"
                        if message.message_type == G3BControlType.REVOKE
                        else "goal_release"
                    ),
                    zero_base=True,
                )
                return receipt(
                    authorized=False,
                    accepted=True,
                    backend_status="CANCELLED",
                    dispatch_state="CANCELLED",
                    reason="goal authority revoked and base zeroed",
                )

            if message.message_type == G3BControlType.STOP:
                sequence = (
                    self._navigation_control_sequence.get(message.session_id, 0) + 1
                )
                self._navigation_control_sequence[message.session_id] = sequence
                request = CommandRequest(
                    run_id=message.run_id,
                    backend_generation=message.backend_generation,
                    episode_id=message.episode_id,
                    session_id=message.session_id,
                    operator_id="nav2_goal_supervisor",
                    operator_kind=OperatorKind.AUTONOMOUS_POLICY,
                    view_source="g3b_nav2_goal",
                    command_id=f"g3b_{uuid4().hex}",
                    command_seq=sequence,
                    command_type=CommandType.STOP,
                    requested_ttl_s=float(message.lease_ttl_s),
                    payload={
                        "latch": True,
                        "reason": message.reason or "navigation_system_stop",
                        "navigation_source": "nav2_goal",
                        "navigation_epoch": message.navigation_epoch,
                    },
                )
                response = self._handle(
                    request, lambda: self._dispatch_command(request)
                )
                accepted = response.status in {
                    CommandStatus.DISPATCHED,
                    CommandStatus.APPLIED,
                    CommandStatus.RECEIVED,
                }
                return receipt(
                    authorized=False,
                    accepted=accepted,
                    backend_status=str(
                        getattr(response.status, "value", response.status)
                    ),
                    dispatch_state="DISPATCHED" if accepted else "REJECTED",
                    reason=response.reason or "software stop dispatched",
                    response=response,
                )

            if not matches_authorization:
                self._base_request[:] = 0.0
                return receipt(
                    authorized=False,
                    accepted=False,
                    backend_status="REJECTED",
                    dispatch_state="REJECTED",
                    reason="goal/session/map authorization mismatch",
                )
            if message.session_id in self._navigation_revoked_sessions:
                return receipt(
                    authorized=False,
                    accepted=False,
                    backend_status="REJECTED",
                    dispatch_state="REJECTED",
                    reason="goal session was revoked",
                )
            if self._navigation_health_stop:
                self._base_request[:] = 0.0
                return receipt(
                    authorized=True,
                    accepted=False,
                    backend_status="REJECTED",
                    dispatch_state="REJECTED",
                    reason="navigation health requires a stop",
                )
            sequence = (
                self._navigation_control_sequence.get(message.session_id, 0) + 1
            )
            self._navigation_control_sequence[message.session_id] = sequence
            request = CommandRequest(
                run_id=message.run_id,
                backend_generation=message.backend_generation,
                episode_id=message.episode_id,
                session_id=message.session_id,
                operator_id="nav2_goal_supervisor",
                operator_kind=OperatorKind.AUTONOMOUS_POLICY,
                view_source="g3b_nav2_goal",
                command_id=f"g3b_{uuid4().hex}",
                command_seq=sequence,
                command_type=CommandType.SET_BASE_TWIST,
                requested_ttl_s=float(message.lease_ttl_s),
                payload={
                    "frame_id": "base_link",
                    "vx_m_s": float(message.vx_m_s),
                    "vy_m_s": float(message.vy_m_s),
                    "wz_rad_s": float(message.wz_rad_s),
                    "navigation_source": "nav2_goal",
                    "navigation_epoch": message.navigation_epoch,
                    "goal_id": message.goal_id,
                    "goal_generation": message.goal_generation,
                    "map_id": message.map_id,
                    "request_id": message.request_id,
                },
            )
            response = self._handle(request, lambda: self._dispatch_command(request))
            accepted = response.status in {
                CommandStatus.DISPATCHED,
                CommandStatus.APPLIED,
                CommandStatus.RECEIVED,
            }
            dispatch_state = (
                "DISPATCHED"
                if response.status == CommandStatus.DISPATCHED
                else "AUTHORIZED"
                if accepted
                else "REJECTED"
            )
            return receipt(
                authorized=authorized is not None,
                accepted=accepted,
                backend_status=str(
                    getattr(response.status, "value", response.status)
                ),
                dispatch_state=dispatch_state,
                reason=response.reason,
                response=response,
            )

    def _submit_g3b_control_queued(
        self, message: G3BControlMessage
    ) -> ControlReceipt:
        """Validate without the physics lock and queue application for one tick."""

        def make_receipt(
            *,
            authorized: bool,
            accepted: bool,
            backend_status: str,
            dispatch_state: str,
            reason: str,
        ) -> ControlReceipt:
            return ControlReceipt(
                request_id=message.request_id,
                goal_id=message.goal_id,
                goal_generation=message.goal_generation,
                map_id=message.map_id,
                session_id=message.session_id,
                run_id=message.run_id,
                episode_id=message.episode_id,
                backend_generation=message.backend_generation,
                navigation_epoch=message.navigation_epoch,
                received=True,
                authorized=authorized,
                accepted=accepted,
                backend_status=backend_status,
                reason=reason,
                control_tick_id=self._last_tick_id,
                backend_monotonic_s=time.monotonic(),
                sim_time_s=float(self.data.time),
                dispatch_state=dispatch_state,
            )

        if message.run_id != self.run_id:
            return make_receipt(
                authorized=False,
                accepted=False,
                backend_status="REJECTED",
                dispatch_state="REJECTED",
                reason="run_id does not match backend",
            )
        if message.episode_id != self.episode_id:
            return make_receipt(
                authorized=False,
                accepted=False,
                backend_status="REJECTED",
                dispatch_state="REJECTED",
                reason="goal belongs to another Episode",
            )
        if message.backend_generation != self.backend_generation:
            return make_receipt(
                authorized=False,
                accepted=False,
                backend_status="REJECTED",
                dispatch_state="REJECTED",
                reason="stale backend_generation",
            )
        if message.navigation_epoch != self._navigation_epoch:
            return make_receipt(
                authorized=False,
                accepted=False,
                backend_status="REJECTED",
                dispatch_state="REJECTED",
                reason="stale navigation_epoch",
            )

        with self._navigation_control_lock:
            now = time.monotonic()
            if message.message_type == G3BControlType.AUTHORIZE:
                if self._stop_latched:
                    reason = "software stop is latched"
                elif self._navigation_health_stop:
                    reason = "navigation health currently requires a stop"
                elif self._recording_required():
                    reason = "formal recording is required and not ready"
                elif message.goal_generation <= self._last_g3b_goal_generation:
                    reason = "stale or duplicate goal_generation"
                else:
                    old = self._authorized_g3b_goal
                    if old is not None:
                        self._navigation_revoked_sessions.add(
                            str(old["session_id"])
                        )
                    self._last_g3b_goal_generation = message.goal_generation
                    self._authorized_g3b_goal = {
                        "goal_id": message.goal_id,
                        "goal_generation": message.goal_generation,
                        "map_id": message.map_id,
                        "run_id": message.run_id,
                        "episode_id": message.episode_id,
                        "backend_generation": message.backend_generation,
                        "navigation_epoch": message.navigation_epoch,
                        "session_id": message.session_id,
                        "authorization_request_id": message.request_id,
                        "authorized_backend_monotonic_s": now,
                    }
                    self._navigation_session_id = message.session_id
                    self._pending_g3b_action = {
                        "type": "zero",
                        "session_id": message.session_id,
                    }
                    reason = "goal authorization accepted"
                    self.publisher.emit(
                        "g3b_goal_authorized",
                        dict(self._authorized_g3b_goal),
                        sim_time_s=float(self.data.time),
                        control_tick_id=self._last_tick_id,
                    )
                    return make_receipt(
                        authorized=True,
                        accepted=True,
                        backend_status="APPLIED",
                        dispatch_state="AUTHORIZED",
                        reason=reason,
                    )
                return make_receipt(
                    authorized=False,
                    accepted=False,
                    backend_status="REJECTED",
                    dispatch_state="REJECTED",
                    reason=reason,
                )

            authorized = self._authorized_g3b_goal
            matches = bool(
                authorized is not None
                and authorized.get("goal_id") == message.goal_id
                and authorized.get("goal_generation")
                == message.goal_generation
                and authorized.get("map_id") == message.map_id
                and authorized.get("session_id") == message.session_id
            )
            if message.message_type in {
                G3BControlType.REVOKE,
                G3BControlType.RELEASE,
            }:
                if not matches:
                    self._pending_g3b_action = {
                        "type": "zero",
                        "session_id": message.session_id,
                    }
                    return make_receipt(
                        authorized=False,
                        accepted=False,
                        backend_status="REJECTED",
                        dispatch_state="REJECTED",
                        reason="stale or mismatched goal revocation",
                    )
                self._navigation_revoked_sessions.add(message.session_id)
                self._authorized_g3b_goal = None
                self._navigation_session_id = None
                self._navigation_cancel_epoch += 1
                self._pending_g3b_action = {
                    "type": "zero",
                    "session_id": message.session_id,
                }
                reason = "goal authority revoked and base zero queued"
                self.publisher.emit(
                    "navigation_authority_revoked",
                    {
                        "reason": (
                            "goal_revoke"
                            if message.message_type == G3BControlType.REVOKE
                            else "goal_release"
                        ),
                        "sessions": [message.session_id],
                        "navigation_cancel_epoch": self._navigation_cancel_epoch,
                        "navigation_epoch": self._navigation_epoch,
                    },
                    sim_time_s=float(self.data.time),
                    control_tick_id=self._last_tick_id,
                )
                return make_receipt(
                    authorized=False,
                    accepted=True,
                    backend_status="CANCELLED",
                    dispatch_state="CANCELLED",
                    reason=reason,
                )

            if message.message_type == G3BControlType.STOP:
                self._navigation_revoked_sessions.add(message.session_id)
                self._authorized_g3b_goal = None
                self._navigation_session_id = None
                self._pending_g3b_action = {"type": "stop"}
                return make_receipt(
                    authorized=False,
                    accepted=True,
                    backend_status="APPLIED",
                    dispatch_state="DISPATCHED",
                    reason="software stop queued",
                )

            if not matches:
                self._pending_g3b_action = {
                    "type": "zero",
                    "session_id": message.session_id,
                }
                return make_receipt(
                    authorized=False,
                    accepted=False,
                    backend_status="REJECTED",
                    dispatch_state="REJECTED",
                    reason="goal/session/map authorization mismatch",
                )
            if message.session_id in self._navigation_revoked_sessions:
                return make_receipt(
                    authorized=False,
                    accepted=False,
                    backend_status="REJECTED",
                    dispatch_state="REJECTED",
                    reason="goal session was revoked",
                )
            if self._navigation_health_stop:
                self._pending_g3b_action = {
                    "type": "zero",
                    "session_id": message.session_id,
                }
                return make_receipt(
                    authorized=True,
                    accepted=False,
                    backend_status="REJECTED",
                    dispatch_state="REJECTED",
                    reason="navigation health requires a stop",
                )
            owner = self._owners["base"]
            if (
                owner is not None
                and owner != message.session_id
                and now < self._lease_deadline["base"]
            ):
                return make_receipt(
                    authorized=True,
                    accepted=False,
                    backend_status="REJECTED",
                    dispatch_state="REJECTED",
                    reason="base is owned by another live session",
                )
            values = np.array(
                [message.vx_m_s, message.vy_m_s, message.wz_rad_s],
                dtype=float,
            )
            if not np.isfinite(values).all():
                return make_receipt(
                    authorized=True,
                    accepted=False,
                    backend_status="REJECTED",
                    dispatch_state="REJECTED",
                    reason="base twist must be finite",
                )
            self._g3b_control_sequence += 1
            sequence = self._g3b_control_sequence
            request = CommandRequest(
                run_id=message.run_id,
                backend_generation=message.backend_generation,
                episode_id=message.episode_id,
                session_id=message.session_id,
                operator_id="nav2_goal_supervisor",
                operator_kind=OperatorKind.AUTONOMOUS_POLICY,
                view_source="g3b_nav2_goal",
                command_id=f"g3b_{uuid4().hex}",
                command_seq=sequence,
                command_type=CommandType.SET_BASE_TWIST,
                requested_ttl_s=float(message.lease_ttl_s),
                payload={
                    "frame_id": "base_link",
                    "vx_m_s": float(values[0]),
                    "vy_m_s": float(values[1]),
                    "wz_rad_s": float(values[2]),
                    "navigation_source": "nav2_goal",
                    "navigation_epoch": message.navigation_epoch,
                    "goal_id": message.goal_id,
                    "goal_generation": message.goal_generation,
                    "map_id": message.map_id,
                    "request_id": message.request_id,
                },
            )
            self._pending_g3b_action = {
                "type": "velocity",
                "values": values.tolist(),
                "session_id": message.session_id,
            }
            self.publisher.emit(
                "requested",
                {"request": request.model_dump(mode="json")},
                command=request,
            )
            self.publisher.emit(
                "approved",
                {
                    "decision": SafetyDecision(
                        approved=True,
                        reason="approved",
                        effective_payload=request.payload,
                        allowed_contacts=["wheel-ground"],
                    ).model_dump(mode="json"),
                    "payload": request.payload,
                },
                command=request,
                sim_time_s=float(self.data.time),
            )
            self.publisher.emit(
                "dispatch",
                {"kind": "base_twist", "requested": values.tolist()},
                command=request,
                sim_time_s=float(self.data.time),
                control_tick_id=self._last_tick_id,
            )
            return make_receipt(
                authorized=True,
                accepted=True,
                backend_status="DISPATCHED",
                dispatch_state="DISPATCHED",
                reason="queued for next physics tick",
            )

    def _apply_pending_g3b_control(self) -> None:
        with self._navigation_control_lock:
            action = self._pending_g3b_action
            self._pending_g3b_action = None
        if action is None:
            return
        now = time.monotonic()
        kind = action.get("type")
        if kind == "velocity":
            session_id = str(action.get("session_id", ""))
            authorized = self._authorized_g3b_goal
            if (
                authorized is None
                or authorized.get("session_id") != session_id
                or session_id in self._navigation_revoked_sessions
                or self._navigation_health_stop
                or self._stop_latched
            ):
                self._base_request[:] = 0.0
                return
            owner = self._owners["base"]
            if owner is not None and owner != session_id and now < self._lease_deadline["base"]:
                return
            self._owners["base"] = session_id
            self._owner_kind["base"] = OperatorKind.AUTONOMOUS_POLICY
            self._owner_operator["base"] = "nav2_goal_supervisor"
            self._lease_deadline["base"] = now + float(
                self.safety_config["lease_timeout_s"]
            )
            self._navigation_session_id = session_id
            self._base_request[:] = np.asarray(action["values"], dtype=float)
        elif kind == "zero":
            session_id = action.get("session_id")
            owner = self._owners["base"]
            owner_kind = self._owner_kind["base"]
            authorized = self._authorized_g3b_goal
            affects_current_owner = (
                owner is None
                or (
                    session_id is not None
                    and owner == session_id
                )
                or (
                    owner_kind == OperatorKind.AUTONOMOUS_POLICY
                    and authorized is not None
                    and session_id is not None
                    and authorized.get("session_id") == session_id
                )
                or (
                    owner_kind == OperatorKind.AUTONOMOUS_POLICY
                    and session_id is None
                )
            )
            if not affects_current_owner:
                return
            self._base_request[:] = 0.0
            if owner is not None and (
                session_id is None or owner == session_id
            ):
                self._owners["base"] = None
                self._owner_kind["base"] = None
                self._owner_operator["base"] = None
                self._lease_deadline["base"] = 0.0
        elif kind == "stop":
            self._base_request[:] = 0.0
            self._cancel_arm_motion("software_stop")
            self._stop_latched = True
            self._owners = {"base": None, "arm": None}
            self._owner_kind = {"base": None, "arm": None}
            self._owner_operator = {"base": None, "arm": None}
            self._lease_deadline = {"base": 0.0, "arm": 0.0}

    def apply_navigation_health(self, health: NavigationHealth) -> None:
        """Apply estimator stop authority without taking over the arm group."""

        with self._navigation_health_lock:
            if health.clock.episode_id != self.episode_id:
                return
            if health.clock.backend_generation != self.backend_generation:
                return
            if health.clock.navigation_epoch != self._navigation_epoch:
                return
            self._navigation_last_health = health.model_dump(mode="json")
            if not health.stop_required:
                self._navigation_health_stop = False
                return
            already_stopped = self._navigation_health_stop
            self._navigation_health_stop = True
        if already_stopped:
            return
        with self.lock:
            if health.clock.navigation_epoch != self._navigation_epoch:
                return
            self._base_request[:] = 0.0
            self._revoke_navigation("navigation_health_stop", zero_base=True)
            self.publisher.emit(
                "navigation_health_stop",
                {
                    "status": health.status,
                    "reasons": health.rejection_reasons,
                    "navigation_epoch": self._navigation_epoch,
                },
                sim_time_s=float(self.data.time),
                control_tick_id=self._last_tick_id,
            )

    def set_navigation_fault(self, fault: str, active: bool) -> dict[str, Any]:
        normalized = str(fault).lower()
        if normalized not in {"wheel", "imu", "scan"}:
            raise ValueError("fault must be wheel, imu, or scan")
        with self.lock:
            if active:
                self._navigation_faults.add(normalized)
            else:
                self._navigation_faults.discard(normalized)
            self.publisher.emit(
                "navigation_fault_injection",
                {
                    "fault": normalized,
                    "active": bool(active),
                    "active_faults": sorted(self._navigation_faults),
                    "scope": "subsequent_capture_only",
                },
                sim_time_s=float(self.data.time),
                control_tick_id=self._last_tick_id,
            )
            return {
                "fault": normalized,
                "active": bool(active),
                "active_faults": sorted(self._navigation_faults),
                "navigation_epoch": self._navigation_epoch,
            }

    def navigation_reference(self, control_tick_id: str) -> dict[str, Any] | None:
        with self.lock:
            value = self._navigation_references.get(control_tick_id)
            return copy.deepcopy(value) if value is not None else None

    def navigation_status(self) -> dict[str, Any]:
        with self.lock:
            return {
                "interface_profile": "g3a-1.0",
                "profile_id": (
                    self.navigation_profile or {}
                ).get("profile_id"),
                "profile_sha256": self.navigation_profile_sha256,
                "enabled": self.navigation_enabled,
                "backend_generation": self.backend_generation,
                "episode_id": self.episode_id,
                "navigation_epoch": self._navigation_epoch,
                "navigation_session_id": self._navigation_session_id,
                "authorized_g3b_goal": copy.deepcopy(self._authorized_g3b_goal),
                "last_g3b_goal_generation": self._last_g3b_goal_generation,
                "navigation_cancel_epoch": self._navigation_cancel_epoch,
                "revoked_session_count": len(self._navigation_revoked_sessions),
                "health_stop_latched_until_new_session": self._navigation_health_stop,
                "active_faults": sorted(self._navigation_faults),
                "last_health": copy.deepcopy(self._navigation_last_health),
                "reference_ring_size": len(self._navigation_references),
            }

    def submit(self, request: CommandRequest) -> CommandResponse:
        if CommandType(request.command_type) == CommandType.PREVIEW_TCP_TARGET:
            return self._preview_tcp_offlock(request)
        return self._handle(request, lambda: self._dispatch_command(request))

    def close(self, timeout_s: float = 5.0) -> None:
        if self.closed:
            return
        self.closed = True
        with self.lock:
            self._base_request[:] = 0.0
            self.wheel.update(self.data, self._base_request, self.params["simulation"]["controller_period_s"])
            self.arm.stop_trajectory(self.data)
        self.stop_event.set()
        for thread in (self.thread, self.sensor_thread):
            if thread is not None:
                thread.join(timeout=max(0.1, timeout_s))

    def pause_sensor_capture(self, timeout_s: float = 2.0) -> None:
        """Wait for an in-flight RGB/Depth pair before recorder shutdown."""

        self._sensor_capture_enabled.clear()
        acquired = self._sensor_commit_lock.acquire(timeout=max(0.01, timeout_s))
        if acquired:
            self._sensor_commit_lock.release()

    def resume_sensor_capture(self) -> None:
        self._rgbd_deadline = time.monotonic() + self._rgbd_period_s
        self._sensor_capture_enabled.set()

    def _build_capabilities(self) -> Capabilities:
        user_limits = self.arm.user_limits
        gripper = self.arm_spec["gripper"]
        return Capabilities(
            backend_kind="mujoco",
            arm_joint_names=ARM_JOINTS,
            arm_user_limits_rad=user_limits.tolist(),
            arm_velocity_limits_rad_s=self.arm.velocity_limits[:5].tolist(),
            gripper_min_width_m=float(gripper["min_width_m"]),
            gripper_max_width_m=float(gripper["max_width_m"]),
            base_linear_limit_m_s=float(self.safety_config["translation_norm_m_s"]),
            base_yaw_limit_rad_s=float(self.safety_config["yaw_rate_rad_s"]),
            tcp_position_tolerance_m=float(self.safety_config["tcp_position_tolerance_m"]),
            tcp_orientation_is_soft=True,
            supported_commands=list(CommandType),
            sensor_streams=list(SensorKind),
            model_id=self.model_id,
            model_sha256=self.model_sha256,
            scene_id=self.scene_id,
            camera_parent_body="link4",
            tcp_parent_body="guide_rod_link",
            ground_truth_policy_visible=False,
        )

    def _load_scene_assets(self) -> dict[str, Path]:
        root = ET.parse(self.model_path.parent / "assets.xml").getroot()
        result: dict[str, Path] = {}
        for node in root.findall(".//mesh"):
            name = node.get("name")
            file_name = node.get("file")
            if not name or not file_name:
                continue
            result[name.removesuffix("_mesh")] = (self.model_path.parent / file_name).resolve()
        return result

    def _camera_is_link4_branch(self) -> bool:
        current = self.camera_body_id
        while current > 0:
            if current == self.link4_id:
                return True
            if current in (self.model.body("link5").id, self.model.body("guide_rod_link").id):
                return False
            current = int(self.model.body_parentid[current])
        return False

    def _handle(self, request: CommandRequest, action) -> CommandResponse:
        received_at = time.monotonic()
        delay = float(self.backend_config.get("test_dispatch_delay_s", 0.0))
        if delay > 0:
            time.sleep(delay)
        with self.lock:
            cached = self._command_cache.get(request.command_id)
            if cached is not None:
                return cached.model_copy(deep=True)
            self.publisher.emit(
                "requested",
                {"request": request.model_dump(mode="json")},
                command=request,
            )
            rejected = self._validate_envelope(request, received_at)
            if rejected is not None:
                response, reason, decision = rejected
                response = response.model_copy(update={"reason": reason, "decision": decision})
                self._command_cache[request.command_id] = response
                self._session_last_seq[request.session_id] = max(
                    request.command_seq, self._session_last_seq.get(request.session_id, 0)
                )
                self.publisher.emit(
                    "rejected",
                    {"reason": reason, "decision": decision.model_dump(mode="json")},
                    command=request,
                    sim_time_s=float(self.data.time),
                )
                self._cached_snapshot = self._build_snapshot()
                return response.model_copy(deep=True)
            response = action()
            self._command_cache[request.command_id] = response
            self._session_last_seq[request.session_id] = max(
                request.command_seq, self._session_last_seq.get(request.session_id, 0)
            )
            self._cached_snapshot = self._build_snapshot()
            return response.model_copy(deep=True)

    def _validate_envelope(
        self, request: CommandRequest, received_at: float | None = None
    ) -> tuple[CommandResponse, str, SafetyDecision] | None:
        now = time.monotonic()
        received_at = now if received_at is None else received_at
        reason = ""
        if request.run_id != self.run_id:
            reason = "run_id does not match backend"
        elif request.backend_generation != self.backend_generation:
            reason = "stale backend_generation"
        elif request.episode_id != self.episode_id:
            reason = "command belongs to another Episode"
        elif request.command_seq <= self._session_last_seq.get(request.session_id, 0):
            reason = "older or duplicate command_seq"
        elif now > received_at + request.requested_ttl_s:
            reason = "command expired before dispatch"
        if not reason:
            return None
        status = (
            CommandStatus.EXPIRED_BEFORE_DISPATCH
            if "expired" in reason
            else CommandStatus.REJECTED
        )
        response = self._response(request, status, reason)
        decision = SafetyDecision(
            approved=False,
            reason=reason,
            effective_payload={},
            allowed_contacts=["wheel-ground", "finger-target-during-grasp"],
        )
        return response, reason, decision

    def _response(
        self,
        request: CommandRequest,
        status: CommandStatus,
        reason: str = "",
        *,
        decision: SafetyDecision | None = None,
        plan: PlanPreview | None = None,
        tick_id: str | None = None,
    ) -> CommandResponse:
        return CommandResponse(
            command_id=request.command_id,
            command_seq=request.command_seq,
            status=status,
            reason=reason,
            backend_monotonic_s=time.monotonic(),
            sim_time_s=float(self.data.time),
            decision=decision,
            plan=plan,
            control_tick_id=tick_id or self._last_tick_id,
        )

    def _approved(self, request: CommandRequest, payload: dict[str, Any]) -> SafetyDecision:
        decision = SafetyDecision(
            approved=True,
            safety_revision="g1-safety-1",
            reason="approved",
            effective_payload=payload,
            allowed_contacts=["wheel-ground", "finger-target-during-grasp"],
        )
        self.publisher.emit(
            "approved",
            {"decision": decision.model_dump(mode="json"), "payload": payload},
            command=request,
            sim_time_s=float(self.data.time),
        )
        return decision

    def _dispatch_command(self, request: CommandRequest) -> CommandResponse:
        command = CommandType(request.command_type)
        if command == CommandType.SET_BASE_TWIST:
            return self._set_base_twist(request)
        if command in (CommandType.ZERO_BASE, CommandType.HEARTBEAT, CommandType.STOP):
            return self._base_control(request, command)
        if command == CommandType.SET_ARM_JOINT_GOAL:
            return self._set_arm_joint(request)
        if command == CommandType.SET_GRIPPER_WIDTH_GOAL:
            return self._set_gripper(request)
        if command == CommandType.PREVIEW_TCP_TARGET:
            return self._preview_tcp(request)
        if command == CommandType.EXECUTE_TCP_TARGET:
            return self._execute_tcp(request)
        if command == CommandType.CANCEL:
            return self._cancel(request)
        if command == CommandType.RESET:
            return self._reset_command(request)
        if command == CommandType.SET_TASK_PHASE:
            return self._set_task_phase(request)
        if command in (CommandType.START_RECORDING, CommandType.STOP_RECORDING):
            return self._response(request, CommandStatus.REJECTED, "recording lifecycle is owned by the application")
        return self._response(request, CommandStatus.REJECTED, f"unsupported command {command.value}")

    def _acquire(self, channel: str, request: CommandRequest) -> tuple[bool, str]:
        now = time.monotonic()
        owner = self._owners[channel]
        if owner is not None and owner != request.session_id and now < self._lease_deadline[channel]:
            return False, f"{channel} is owned by another live session"
        self._owners[channel] = request.session_id
        self._owner_kind[channel] = OperatorKind(request.operator_kind)
        self._owner_operator[channel] = request.operator_id
        self._lease_deadline[channel] = now + float(self.safety_config["lease_timeout_s"])
        return True, ""

    def _set_base_twist(self, request: CommandRequest) -> CommandResponse:
        if self._stop_latched:
            return self._response(request, CommandStatus.REJECTED, "software stop is latched")
        if self._recording_required():
            return self._response(
                request,
                CommandStatus.REJECTED,
                "formal recording is required and command capture is not ready",
            )
        payload = request.payload
        if self._g4a1_permission is not None and any(
            abs(float(payload.get(key, 0.0))) > 1e-9
            for key in ("vx_m_s", "vy_m_s", "wz_rad_s")
        ):
            return self._response(
                request,
                CommandStatus.REJECTED,
                "nearfield permission requires explicit cancellation before base motion",
            )
        navigation_source = payload.get("navigation_source")
        navigation_command = (
            navigation_source in {"ros_navigation_test", "nav2_goal"}
            and OperatorKind(request.operator_kind)
            == OperatorKind.AUTONOMOUS_POLICY
        )
        if navigation_command:
            if not self.navigation_enabled:
                return self._response(
                    request,
                    CommandStatus.REJECTED,
                    "navigation profile is disabled",
                )
            if int(payload.get("navigation_epoch", -1)) != self._navigation_epoch:
                return self._response(
                    request,
                    CommandStatus.REJECTED,
                    "stale navigation_epoch",
                )
            if request.session_id in self._navigation_revoked_sessions:
                return self._response(
                    request,
                    CommandStatus.REJECTED,
                    "navigation session was revoked by takeover or health stop",
                )
            if self._navigation_health_stop:
                return self._response(
                    request,
                    CommandStatus.REJECTED,
                    "navigation health requires a stop and a new session",
                )
            if navigation_source == "nav2_goal":
                authorized = self._authorized_g3b_goal
                if not (
                    authorized is not None
                    and authorized.get("goal_id") == payload.get("goal_id")
                    and authorized.get("goal_generation")
                    == int(payload.get("goal_generation", -1))
                    and authorized.get("map_id") == payload.get("map_id")
                    and authorized.get("session_id") == request.session_id
                ):
                    return self._response(
                        request,
                        CommandStatus.REJECTED,
                        "goal/session/map authorization mismatch",
                    )
        elif (
            self._owners["base"] is not None
            and self._owner_kind["base"] == OperatorKind.AUTONOMOUS_POLICY
        ):
            self._revoke_navigation("manual_base_takeover", zero_base=True)
        ok, reason = self._acquire("base", request)
        if not ok:
            return self._response(request, CommandStatus.REJECTED, reason)
        values = np.array(
            [payload["vx_m_s"], payload["vy_m_s"], payload["wz_rad_s"]], dtype=float
        )
        if not np.isfinite(values).all():
            return self._response(request, CommandStatus.REJECTED, "base twist must be finite")
        self._base_request[:] = values
        if navigation_command:
            self._navigation_session_id = request.session_id
        elif not bool(
            self.safety_config.get("preserve_arm_on_manual_base", False)
        ):
            self._cancel_arm_motion("base_manual_takeover")
            self._owners["arm"] = None
            self._owner_kind["arm"] = None
            self._owner_operator["arm"] = None
            self._lease_deadline["arm"] = 0.0
        decision = self._approved(
            request,
            {
                "frame_id": "base_link",
                "vx_m_s": float(values[0]),
                "vy_m_s": float(values[1]),
                "wz_rad_s": float(values[2]),
            },
        )
        self.publisher.emit(
            "dispatch",
            {"kind": "base_twist", "requested": values.tolist()},
            command=request,
            sim_time_s=float(self.data.time),
            control_tick_id=self._last_tick_id,
        )
        return self._response(request, CommandStatus.DISPATCHED, decision=decision)

    def _base_control(self, request: CommandRequest, command: CommandType) -> CommandResponse:
        if command == CommandType.HEARTBEAT:
            now = time.monotonic()
            renewed = False
            for channel in ("base", "arm"):
                if self._owners[channel] == request.session_id:
                    self._lease_deadline[channel] = now + float(self.safety_config["lease_timeout_s"])
                    renewed = True
            reason = "lease renewed" if renewed else "session owns no channel"
            self.publisher.emit("lease_heartbeat", {"renewed": renewed}, command=request)
            return self._response(request, CommandStatus.APPLIED, reason)
        if command == CommandType.ZERO_BASE:
            navigation_command = (
                request.payload.get("navigation_source") == "ros_navigation_test"
                and OperatorKind(request.operator_kind)
                == OperatorKind.AUTONOMOUS_POLICY
            )
            if navigation_command:
                if int(
                    request.payload.get("navigation_epoch", -1)
                ) != self._navigation_epoch:
                    return self._response(
                        request,
                        CommandStatus.REJECTED,
                        "stale navigation_epoch",
                    )
                self._navigation_revoked_sessions.add(request.session_id)
            elif (
                self._owners["base"] is not None
                and self._owner_kind["base"] == OperatorKind.AUTONOMOUS_POLICY
            ):
                self._revoke_navigation("manual_zero_base_takeover", zero_base=True)
            self._base_request[:] = 0.0
            if self._owners["base"] == request.session_id:
                self._owners["base"] = None
                self._owner_kind["base"] = None
                self._owner_operator["base"] = None
                self._lease_deadline["base"] = 0.0
            if navigation_command:
                self._navigation_session_id = None
            decision = self._approved(request, {"vx_m_s": 0.0, "vy_m_s": 0.0, "wz_rad_s": 0.0})
            self.publisher.emit(
                "dispatch",
                {"kind": "zero_base"},
                command=request,
                sim_time_s=float(self.data.time),
                control_tick_id=self._last_tick_id,
            )
            return self._response(request, CommandStatus.DISPATCHED, decision=decision)
        latch = bool(request.payload.get("latch", True))
        self._revoke_nearfield("software_stop", request.operator_kind)
        self._base_request[:] = 0.0
        self._revoke_navigation("software_stop", zero_base=False)
        self._cancel_arm_motion("software_stop")
        self._stop_latched = latch
        self._owners = {"base": None, "arm": None}
        self._owner_kind = {"base": None, "arm": None}
        self._owner_operator = {"base": None, "arm": None}
        self._lease_deadline = {"base": 0.0, "arm": 0.0}
        decision = self._approved(
            request,
            {
                "latch": latch,
                "base": "zero_and_brake",
                "arm": "cancel_and_hold",
                "gripper": "hold_current_width",
            },
        )
        self.publisher.emit(
            "dispatch",
            {"kind": "software_stop", "latch": latch},
            command=request,
            sim_time_s=float(self.data.time),
            control_tick_id=self._last_tick_id,
        )
        return self._response(request, CommandStatus.DISPATCHED, decision=decision)

    def _set_arm_joint(self, request: CommandRequest) -> CommandResponse:
        if self._stop_latched:
            return self._response(request, CommandStatus.REJECTED, "software stop is latched")
        if self._recording_required():
            return self._response(
                request,
                CommandStatus.REJECTED,
                "formal recording is required and command capture is not ready",
            )
        ok, reason = self._acquire("arm", request)
        if not ok:
            return self._response(request, CommandStatus.REJECTED, reason)
        user_values = np.asarray(request.payload["joint_positions_rad"], dtype=float)
        if np.any(user_values < self.arm.user_limits[:, 0]) or np.any(
            user_values > self.arm.user_limits[:, 1]
        ):
            return self._response(request, CommandStatus.REJECTED, "joint target violates user limits")
        self._cancel_active_plan("joint_takeover")
        self.arm.set_user_goal(arm=user_values)
        decision = self._approved(
            request,
            {"joint_names": ARM_JOINTS, "joint_positions_rad": user_values.tolist()},
        )
        self.publisher.emit(
            "dispatch",
            {"kind": "arm_joint_goal", "user_joint_positions_rad": user_values.tolist()},
            command=request,
            sim_time_s=float(self.data.time),
            control_tick_id=self._last_tick_id,
        )
        return self._response(request, CommandStatus.DISPATCHED, decision=decision)

    def _set_gripper(self, request: CommandRequest) -> CommandResponse:
        if self._stop_latched:
            return self._response(request, CommandStatus.REJECTED, "software stop is latched")
        if self._recording_required():
            return self._response(
                request,
                CommandStatus.REJECTED,
                "formal recording is required and command capture is not ready",
            )
        ok, reason = self._acquire("arm", request)
        if not ok:
            return self._response(request, CommandStatus.REJECTED, reason)
        width = float(request.payload["width_m"])
        gripper = self.arm_spec["gripper"]
        if not gripper["min_width_m"] <= width <= gripper["max_width_m"]:
            return self._response(
                request,
                CommandStatus.REJECTED,
                f"gripper width outside [{gripper['min_width_m']:.5f}, {gripper['max_width_m']:.5f}] m",
            )
        self._cancel_arm_motion("gripper_takeover")
        self.arm.set_user_goal(width=width)
        decision = self._approved(request, {"width_m": width})
        self.publisher.emit(
            "dispatch",
            {"kind": "gripper_width_goal", "width_m": width},
            command=request,
            sim_time_s=float(self.data.time),
            control_tick_id=self._last_tick_id,
        )
        return self._response(request, CommandStatus.DISPATCHED, decision=decision)

    def _preview_tcp_offlock(self, request: CommandRequest) -> CommandResponse:
        """Copy state under a short lock, solve outside it, then validate versions."""

        received_at = time.monotonic()
        lock_started = time.monotonic()
        with self.lock:
            lock_acquired = time.monotonic()
            cached = self._command_cache.get(request.command_id)
            if cached is not None:
                return cached.model_copy(deep=True)
            self.publisher.emit(
                "requested",
                {"request": request.model_dump(mode="json")},
                command=request,
            )
            rejected = self._validate_envelope(request, received_at)
            if rejected is not None:
                response, reason, decision = rejected
                response = response.model_copy(
                    update={"reason": reason, "decision": decision}
                )
                self._command_cache[request.command_id] = response
                self._session_last_seq[request.session_id] = max(
                    request.command_seq,
                    self._session_last_seq.get(request.session_id, 0),
                )
                self.publisher.emit(
                    "rejected",
                    {"reason": reason, "decision": decision.model_dump(mode="json")},
                    command=request,
                    sim_time_s=float(self.data.time),
                )
                return response.model_copy(deep=True)
            if self._stop_latched:
                reason = "software stop is latched"
                decision = SafetyDecision(approved=False, reason=reason)
                response = self._response(
                    request, CommandStatus.REJECTED, reason, decision=decision
                )
                self._command_cache[request.command_id] = response
                self._session_last_seq[request.session_id] = max(
                    request.command_seq,
                    self._session_last_seq.get(request.session_id, 0),
                )
                return response.model_copy(deep=True)
            ok, reason = self._acquire("arm", request)
            if not ok:
                decision = SafetyDecision(approved=False, reason=reason)
                response = self._response(
                    request, CommandStatus.REJECTED, reason, decision=decision
                )
                self._command_cache[request.command_id] = response
                self._session_last_seq[request.session_id] = max(
                    request.command_seq,
                    self._session_last_seq.get(request.session_id, 0),
                )
                return response.model_copy(deep=True)
            target = self._world_target(request)
            current_rotation = self.data.site_xmat[self.tcp_id].reshape(3, 3).copy()
            orientation = request.payload.get("orientation_xyzw")
            target_rotation = current_rotation
            if orientation is not None:
                quat = np.asarray(orientation, dtype=float)
                matrix = np.empty(9)
                mujoco.mju_quat2Mat(
                    matrix, np.asarray([quat[3], quat[0], quat[1], quat[2]])
                )
                local_rotation = matrix.reshape(3, 3)
                if request.payload["frame_id"] == "world_sim":
                    target_rotation = local_rotation
                else:
                    target_rotation = (
                        self._target_frame_rotation(request.payload["frame_id"])
                        @ local_rotation
                    )
            revision = self._target_revision + 1
            self._target_revision = revision
            qpos_snapshot = self.data.qpos.copy()
            base_position = self.data.xpos[self.base_id].copy()
            base_rotation = self.data.xmat[self.base_id].reshape(3, 3).copy()
            planning_input = _PlanningInput(
                target_position_world=target,
                target_rotation_world=target_rotation,
                qpos_snapshot=qpos_snapshot,
                current_tcp_rotation=current_rotation,
                gripper_width_m=float(self._snapshot_gripper_width()),
                grasp_contacts=object_contacts(self),
                task_phase=self._task_phase,
                target_revision=revision,
                cancel_epoch=self._cancel_epoch,
                base_position_world=base_position,
                base_rotation_world=base_rotation,
                base_request_body=self._base_request.copy(),
                control_tick_id=self._last_tick_id,
                backend_generation=self.backend_generation,
            )
            episode_id = self.episode_id
            model_sha256 = self.model_sha256
            profile_sha256 = self.observation_profile_sha256
            self._session_last_seq[request.session_id] = max(
                request.command_seq,
                self._session_last_seq.get(request.session_id, 0),
            )
        copy_complete = time.monotonic()
        lock_wait_ms = (lock_acquired - lock_started) * 1000.0
        copy_ms = (copy_complete - lock_acquired) * 1000.0

        solve_started = time.monotonic()
        try:
            plan = self._make_plan_from_input(
                request,
                planning_input,
                episode_id=episode_id,
                model_sha256=model_sha256,
                profile_sha256=profile_sha256,
                lock_wait_ms=lock_wait_ms,
                copy_ms=copy_ms,
            )
        except ValueError as exc:
            with self.lock:
                decision = SafetyDecision(
                    approved=False, reason=str(exc), effective_payload={}
                )
                response = self._response(
                    request, CommandStatus.REJECTED, str(exc), decision=decision
                )
                self._command_cache[request.command_id] = response
                self.publisher.emit(
                    "rejected", {"reason": str(exc)}, command=request
                )
                self._cached_snapshot = self._build_snapshot()
                return response.model_copy(deep=True)

        with self.lock:
            stale_reason = ""
            if self.episode_id != episode_id:
                stale_reason = "Episode changed while planning"
            elif self.model_sha256 != model_sha256:
                stale_reason = "model changed while planning"
            elif self._target_revision != planning_input.target_revision:
                stale_reason = "target revision changed while planning"
            elif self._cancel_epoch != planning_input.cancel_epoch:
                stale_reason = "plan was cancelled while solving"
            elif self.backend_generation != planning_input.backend_generation:
                stale_reason = "backend generation changed while planning"
            elif self._owners["arm"] != request.session_id:
                stale_reason = "arm lease expired while planning"
            else:
                current_base_position = self.data.xpos[self.base_id].copy()
                current_base_rotation = self.data.xmat[self.base_id].reshape(3, 3).copy()
                if (
                    np.linalg.norm(current_base_position - planning_input.base_position_world)
                    > 1e-3
                    or np.linalg.norm(
                        self._base_request - planning_input.base_request_body
                    )
                    > 1e-6
                    or np.max(
                        np.abs(current_base_rotation - planning_input.base_rotation_world)
                    )
                    > 2e-3
                ):
                    stale_reason = "base state changed while planning"
                elif np.max(
                    np.abs(
                        self.data.qpos[self.arm.qpos[:5]]
                        - planning_input.qpos_snapshot[self.arm.qpos[:5]]
                    )
                ) > 5e-3:
                    stale_reason = "arm state changed while planning"
                elif abs(
                    self._snapshot_gripper_width() - planning_input.gripper_width_m
                ) > 2e-3:
                    stale_reason = "gripper state changed while planning"
            if stale_reason:
                plan.planning_metrics = {
                    **plan.planning_metrics,
                    "stale": True,
                    "stale_reason": stale_reason,
                }
                self._last_planning_metric = plan.planning_metrics
                decision = SafetyDecision(approved=False, reason=stale_reason)
                response = self._response(
                    request, CommandStatus.REJECTED, stale_reason, decision=decision
                )
                self._command_cache[request.command_id] = response
                self.publisher.emit(
                    "plan_discarded",
                    plan.planning_metrics,
                    command=request,
                    plan_id=plan.preview.plan_id,
                )
                self._cached_snapshot = self._build_snapshot()
                return response.model_copy(deep=True)
            self._previews[plan.preview.plan_id] = plan
            decision = self._approved(request, request.payload)
            self.publisher.emit(
                "preview_plan",
                plan.preview.model_dump(mode="json"),
                command=request,
                plan_id=plan.preview.plan_id,
                sim_time_s=float(self.data.time),
            )
            self.publisher.emit(
                "planning_metrics",
                plan.planning_metrics,
                command=request,
                plan_id=plan.preview.plan_id,
                sim_time_s=float(self.data.time),
            )
            response = self._response(
                request,
                CommandStatus.RECEIVED,
                (
                    plan.preview.collision_details[0]
                    if plan.preview.collision_details
                    else "plan previewed"
                ),
                decision=decision,
                plan=plan.preview,
            )
            self._command_cache[request.command_id] = response
            self._last_planning_metric = plan.planning_metrics
            self._cached_snapshot = self._build_snapshot()
            return response.model_copy(deep=True)

    def _preview_tcp(self, request: CommandRequest) -> CommandResponse:
        # Retained as an internal compatibility entry; submit routes previews
        # through _preview_tcp_offlock so this path must never hold the lock.
        raise RuntimeError("TCP preview must use the off-lock planner path")

    def _execute_tcp(self, request: CommandRequest) -> CommandResponse:
        if self._stop_latched:
            return self._response(request, CommandStatus.REJECTED, "software stop is latched")
        if self._recording_required():
            return self._response(
                request,
                CommandStatus.REJECTED,
                "formal recording is required and command capture is not ready",
            )
        plan_id = str(request.payload["plan_id"])
        plan = self._previews.get(plan_id)
        if plan is None:
            return self._response(request, CommandStatus.REJECTED, "unknown or expired plan_id")
        if plan.episode_id != self.episode_id or plan.backend_generation != self.backend_generation:
            return self._response(request, CommandStatus.REJECTED, "plan identity is stale")
        if plan.model_sha256 != self.model_sha256 or plan.profile_sha256 != self.observation_profile_sha256:
            return self._response(request, CommandStatus.REJECTED, "plan model/profile identity mismatch")
        if plan.target_revision != self._target_revision or plan.cancel_epoch != self._cancel_epoch:
            return self._response(request, CommandStatus.REJECTED, "plan revision or cancel epoch is stale")
        now = time.monotonic()
        if now > plan.expires_monotonic_s:
            return self._response(request, CommandStatus.EXPIRED_BEFORE_DISPATCH, "plan preview expired")
        if not plan.preview.collision_free:
            return self._response(request, CommandStatus.REJECTED, "plan failed collision precheck")
        target = self._world_target(request)
        if np.linalg.norm(target - plan.target_position_world) > 1e-4:
            return self._response(request, CommandStatus.REJECTED, "execute target differs from preview")
        base_pose = self._base_pose_world()
        if np.linalg.norm(base_pose[0] - plan.base_position_world) > 0.01:
            return self._response(request, CommandStatus.REJECTED, "base moved after preview; preview again")
        ok, reason = self._acquire("arm", request)
        if not ok:
            return self._response(request, CommandStatus.REJECTED, reason)
        start = self.data.qpos[self.arm.qpos[:5]].copy()
        if np.max(np.abs(start - plan.start_model_rad)) > 0.06:
            return self._response(request, CommandStatus.REJECTED, "arm moved after preview; preview again")
        try:
            self.arm.follow_trajectory(plan.trajectory, self.data)
        except ValueError as exc:
            return self._response(request, CommandStatus.REJECTED, str(exc))
        self._active_plan = plan
        self._active_command = request
        self._active_plan_started_monotonic = now
        self._active_plan_started_sim = float(self.data.time)
        self._reached_since = None
        decision = self._approved(request, request.payload)
        self.publisher.emit(
            "dispatch",
            {"kind": "tcp_trajectory", "plan_id": plan_id},
            command=request,
            plan_id=plan_id,
            sim_time_s=float(self.data.time),
            control_tick_id=self._last_tick_id,
        )
        return self._response(
            request,
            CommandStatus.DISPATCHED,
            decision=decision,
            plan=plan.preview.model_copy(update={"status": "EXECUTING"}),
        )

    def _make_plan_from_input(
        self,
        request: CommandRequest,
        planning: _PlanningInput,
        *,
        episode_id: str,
        model_sha256: str,
        profile_sha256: str,
        lock_wait_ms: float,
        copy_ms: float,
    ) -> _PreviewPlan:
        target = planning.target_position_world
        target_rotation = planning.target_rotation_world
        start = planning.qpos_snapshot[self.arm.qpos[:5]].copy()
        solve_started = time.monotonic()
        result = solve_pose(
            self,
            target,
            target_rotation,
            seed=start,
            max_iterations=400,
            qpos_snapshot=planning.qpos_snapshot,
            position_tolerance_m=float(self.safety_config["tcp_position_tolerance_m"]),
        )
        solve_ms = (time.monotonic() - solve_started) * 1000.0
        tolerance = float(self.safety_config["tcp_position_tolerance_m"])
        if result.position_error_m > tolerance:
            raise ValueError(
                f"TCP target unreachable: predicted position residual {result.position_error_m:.5f} m"
            )
        opening = planning.gripper_width_m
        approved_grasp_phase = planning.task_phase in (
            TaskPhase.APPROACH,
            TaskPhase.MANIPULATE,
            TaskPhase.VERIFY,
        )
        # Bilateral finger contact means the cube participates in the planned
        # payload even before it has cleared the support. The support-release
        # tolerance below is bounded to 1 mm and actual dynamics remain free.
        carried_object = bool(
            approved_grasp_phase and planning.grasp_contacts["bilateral_contact"]
        )
        collision_started = time.monotonic()
        collision_details = sampled_path_collisions(
            self,
            np.vstack([start, result.model_angles_rad]),
            opening,
            qpos_snapshot=planning.qpos_snapshot,
            allow_finger_object=approved_grasp_phase,
            carried_object=carried_object,
            initial_support_tolerance_m=0.001 if carried_object else 0.0,
        )
        collision_ms = (time.monotonic() - collision_started) * 1000.0
        trajectory_started = time.monotonic()
        trajectory = time_parameterize(
            np.vstack([start, result.model_angles_rad]),
            self.arm.velocity_limits[:5],
            self.arm.acceleration_limits[:5],
            speed_scale=float(self.safety_config.get("tcp_speed_scale", 0.5)),
            minimum_segment_s=0.2,
        )
        trajectory_ms = (time.monotonic() - trajectory_started) * 1000.0
        revision = planning.target_revision
        now = time.monotonic()
        expiry = min(now + 30.0, now + request.requested_ttl_s + 5.0)
        orientation = request.payload.get("orientation_xyzw")
        preview = PlanPreview(
            plan_id=new_id("plan"),
            command_id=request.command_id,
            target_revision=revision,
            frame_id=request.payload["frame_id"],
            position_m=target.tolist(),
            orientation_xyzw=(
                None
                if orientation is None
                else muJoCo_mat_to_xyzw(target_rotation)
            ),
            waypoints_model_rad=trajectory.waypoints.tolist(),
            times_s=[0.0, trajectory.duration_s],
            predicted_position_error_m=float(result.position_error_m),
            predicted_orientation_error_rad=float(result.axis_error_rad),
            joint_margin_rad=np.minimum(
                result.model_angles_rad - self.arm.limits[:5, 0],
                self.arm.limits[:5, 1] - result.model_angles_rad,
            ).tolist(),
            collision_free=not collision_details,
            collision_details=["; ".join(pair) for pair in collision_details],
            created_backend_monotonic_s=now,
            expires_backend_monotonic_s=expiry,
            status="PREVIEWED",
        )
        total_ms = (
            (now - solve_started) * 1000.0
            + lock_wait_ms
            + copy_ms
        )
        metrics = {
            "interface_profile": "g2a-1.0",
            "lock_wait_ms": lock_wait_ms,
            "copy_ms": copy_ms,
            "solve_ms": solve_ms,
            "collision_check_ms": collision_ms,
            "trajectory_ms": trajectory_ms,
            "total_ms": total_ms,
            "target_revision": revision,
            "cancel_epoch": planning.cancel_epoch,
            "control_tick_id": planning.control_tick_id,
            "model_sha256": model_sha256,
            "profile_sha256": profile_sha256,
            "stale": False,
        }
        return _PreviewPlan(
            preview=preview,
            trajectory=trajectory,
            endpoint_model_rad=result.model_angles_rad.copy(),
            start_model_rad=start,
            target_rotation_world=target_rotation,
            target_position_world=target,
            target_revision=revision,
            cancel_epoch=planning.cancel_epoch,
            base_position_world=planning.base_position_world,
            base_rotation_world=planning.base_rotation_world,
            created_monotonic_s=now,
            expires_monotonic_s=expiry,
            model_sha256=model_sha256,
            profile_sha256=profile_sha256,
            backend_generation=self.backend_generation,
            episode_id=episode_id,
            planning_metrics=metrics,
        )

    def _world_target(self, request: CommandRequest) -> np.ndarray:
        local = np.asarray(request.payload["position_m"], dtype=float)
        return self._target_frame_position(request.payload["frame_id"]) + self._target_frame_rotation(
            request.payload["frame_id"]
        ) @ local

    def _target_frame_position(self, frame_id: str) -> np.ndarray:
        if frame_id == "world_sim":
            return np.zeros(3)
        if frame_id == "base_link":
            return self.data.xpos[self.base_id].copy()
        if frame_id == "arm_base":
            return self.data.site_xpos[self.model.site("arm_base").id].copy()
        raise ValueError(f"unsupported frame: {frame_id}")

    def _target_frame_rotation(self, frame_id: str) -> np.ndarray:
        if frame_id == "world_sim":
            return np.eye(3)
        if frame_id == "base_link":
            return self.data.xmat[self.base_id].reshape(3, 3).copy()
        if frame_id == "arm_base":
            return self.data.site_xmat[self.model.site("arm_base").id].reshape(3, 3).copy()
        raise ValueError(f"unsupported frame: {frame_id}")

    def _recording_required(self) -> bool:
        required = bool(self.safety_config.get("require_recording", False))
        return required and not self.publisher.recording_gate(self.episode_id)

    def _base_pose_world(self) -> tuple[np.ndarray, np.ndarray]:
        return self.data.xpos[self.base_id].copy(), self.data.xmat[self.base_id].reshape(3, 3).copy()

    def _cancel_arm_motion(self, reason: str) -> None:
        if self._active_plan is not None:
            self._cancel_active_plan(reason)
        else:
            self.arm.stop_trajectory(self.data)

    def _revoke_navigation(self, reason: str, *, zero_base: bool) -> None:
        """Revoke only ROS navigation authority; arm ownership is independent."""

        revoked_goal = copy.deepcopy(self._authorized_g3b_goal)
        self._authorized_g3b_goal = None
        owner = self._owners["base"]
        sessions = {
            value
            for value in (owner if owner is not None else None, self._navigation_session_id)
            if value is not None
        }
        if zero_base:
            self._base_request[:] = 0.0
        if owner is not None and self._owner_kind["base"] == OperatorKind.AUTONOMOUS_POLICY:
            self._owners["base"] = None
            self._owner_kind["base"] = None
            self._owner_operator["base"] = None
            self._lease_deadline["base"] = 0.0
        if self._navigation_session_id is not None:
            sessions.add(self._navigation_session_id)
        self._navigation_revoked_sessions.update(sessions)
        self._navigation_session_id = None
        self._navigation_cancel_epoch += 1
        self.publisher.emit(
            "navigation_authority_revoked",
            {
                "reason": reason,
                "sessions": sorted(sessions),
                "navigation_cancel_epoch": self._navigation_cancel_epoch,
                "navigation_epoch": self._navigation_epoch,
                "goal": revoked_goal,
            },
            sim_time_s=float(self.data.time),
            control_tick_id=self._last_tick_id,
        )

    def _cancel_active_plan(self, reason: str) -> None:
        plan = self._active_plan
        if plan is not None:
            self.arm.stop_trajectory(self.data)
            self.publisher.emit(
                "cancelled",
                {"reason": reason, "plan_id": plan.preview.plan_id},
                command=self._active_command,
                plan_id=plan.preview.plan_id,
                sim_time_s=float(self.data.time),
            )
        self._active_plan = None
        self._active_command = None
        self._reached_since = None

    def _cancel(self, request: CommandRequest) -> CommandResponse:
        self._revoke_nearfield("operator_cancel", request.operator_kind)
        self._base_request[:] = 0.0
        self._revoke_navigation("operator_cancel", zero_base=False)
        self._cancel_arm_motion("operator_cancel")
        self._cancel_epoch += 1
        self._owners = {"base": None, "arm": None}
        self._owner_kind = {"base": None, "arm": None}
        self._owner_operator = {"base": None, "arm": None}
        self._lease_deadline = {"base": 0.0, "arm": 0.0}
        decision = self._approved(
            request,
            {"base": "zero", "arm": "cancel_and_hold", "gripper": "hold_current"},
        )
        return self._response(request, CommandStatus.CANCELLED, "motion cancelled and held", decision=decision)

    def _reset_command(self, request: CommandRequest) -> CommandResponse:
        self._revoke_nearfield("episode_reset", request.operator_kind)
        self._base_request[:] = 0.0
        self._revoke_navigation("episode_reset", zero_base=False)
        self._active_plan = None
        self._active_command = None
        self._previews.clear()
        self._target_revision = 0
        self._cancel_epoch += 1
        self._stop_latched = False
        self._episode_timed_out = False
        self._owners = {"base": None, "arm": None}
        self._owner_kind = {"base": None, "arm": None}
        self._owner_operator = {"base": None, "arm": None}
        self._lease_deadline = {"base": 0.0, "arm": 0.0}
        old_episode = self.episode_id
        self._reset_physics(new_episode=True)
        self.publisher.emit(
            "episode_started",
            {"previous_episode_id": old_episode, "episode_id": self.episode_id, "boundary": "RESET"},
            command=request,
            sim_time_s=0.0,
        )
        decision = self._approved(request, {"confirm": True, "episode_id": self.episode_id})
        return self._response(request, CommandStatus.APPLIED, "reset to new Episode", decision=decision)

    def _set_task_phase(self, request: CommandRequest) -> CommandResponse:
        self._task_phase = TaskPhase(request.payload["phase"])
        decision = self._approved(request, {"phase": self._task_phase.value})
        self.publisher.emit(
            "task_phase",
            {"phase": self._task_phase.value},
            command=request,
            task_phase=self._task_phase.value,
        )
        return self._response(request, CommandStatus.APPLIED, decision=decision)

    def _reset_physics(self, *, new_episode: bool) -> None:
        mujoco.mj_resetData(self.model, self.data)
        for name, value in self.arm_spec["initial_positions"].items():
            joint_id = self.model.joint(name).id
            self.data.qpos[self.model.jnt_qposadr[joint_id]] = value
        if self.g4a1_enabled:
            prepared = self.g4a1_config.get("prepared_base_qpos")
            if isinstance(prepared, list) and len(prepared) == 7:
                self.data.qpos[self.base_qpos : self.base_qpos + 7] = np.asarray(
                    prepared, dtype=float
                )
            if self.target_qpos is not None:
                cube = self.g4a1_config.get(
                    "initial_cube_position_m", [0.4, 0.0, 0.112]
                )
                self.data.qpos[
                    self.target_qpos : self.target_qpos + 3
                ] = np.asarray(cube, dtype=float)
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self.wheel.reset()
        self.arm.reset(self.data)
        self._base_request[:] = 0.0
        self._stop_latched = False
        self._stopped = True
        self._stopped_since = float(self.data.time)
        self._feedback_time = time.monotonic()
        self._task_phase = TaskPhase.OBSERVE
        self._last_unexpected = []
        if new_episode:
            self._g4a1_observation_cache.clear()
            self._g4a1_permission = None
            self._g4a1_epoch += 1
            self._g4a2_human_sessions.clear()
            self._g4a2_intents.clear()
            self._sensor_capture_enabled.set()
            self._rgbd_deadline = time.monotonic() + self._rgbd_period_s
            self._rgbd_schedule_index = 0
            self._rgbd_skip_reasons = Counter()
            self._rgbd_skipped_slots = 0
            self._last_capture_metric = {}
            self.episode_id = new_id("episode")
            self._navigation_epoch += 1
            self._navigation_sequence = 0
            self._navigation_session_id = None
            self._authorized_g3b_goal = None
            self._last_g3b_goal_generation = 0
            self._g3b_control_sequence = 0
            with self._navigation_control_lock:
                self._pending_g3b_action = None
            self._navigation_revoked_sessions.clear()
            self._navigation_control_sequence.clear()
            self._navigation_faults.clear()
            self._navigation_health_stop = False
            self._navigation_last_health = {
                "status": "UNKNOWN",
                "stop_required": False,
            }
            self.episode_started = time.monotonic()
            self._incoming_root = self.run_root / self.run_id / "_incoming" / self.episode_id
            self._incoming_root.mkdir(parents=True, exist_ok=True)
            self._sensor_states = {
                kind: SensorState(kind=kind, available=False, fresh=False)
                for kind in SensorKind
            }
        self._navigation_references.clear()
        self._navigation_next_publish_sim_s = float(self.data.time) + 0.02
        self._navigation_next_scan_sim_s = float(self.data.time) + 0.10

    def _physics_loop(self) -> None:
        dt = float(self.params["simulation"]["controller_period_s"])
        substeps = max(1, round(dt / float(self.model.opt.timestep)))
        next_time = time.monotonic()
        realtime = bool(self.backend_config.get("realtime", True))
        while not self.stop_event.is_set():
            with self.lock:
                self._control_tick(dt, substeps)
                self._cached_snapshot = self._build_snapshot()
            next_time += dt
            if realtime:
                delay = next_time - time.monotonic()
                if delay > 0:
                    self.stop_event.wait(min(delay, dt))
                elif delay < -5 * dt:
                    next_time = time.monotonic()

    def _control_tick(self, dt: float, substeps: int) -> None:
        now = time.monotonic()
        self._tick_count += 1
        self._last_tick_id = f"tick_{self._tick_count:09d}"
        self._apply_pending_g3b_control()
        if (
            not self._episode_timed_out
            and now - self.episode_started
            > float(self.safety_config["episode_timeout_s"])
        ):
            self._episode_timed_out = True
            self._base_request[:] = 0.0
            self._revoke_navigation("episode_timeout", zero_base=False)
            self._cancel_arm_motion("episode_timeout")
            self._stop_latched = True
            self._owners = {"base": None, "arm": None}
            self._owner_kind = {"base": None, "arm": None}
            self._owner_operator = {"base": None, "arm": None}
            self._lease_deadline = {"base": 0.0, "arm": 0.0}
            self.publisher.emit(
                "episode_timeout",
                {
                    "episode_id": self.episode_id,
                    "limit_s": float(self.safety_config["episode_timeout_s"]),
                    "termination_reason": "EPISODE_TIMEOUT",
                },
                sim_time_s=float(self.data.time),
                control_tick_id=self._last_tick_id,
            )
        for channel in ("base", "arm"):
            owner = self._owners[channel]
            if owner is not None and now >= self._lease_deadline[channel]:
                was_navigation = (
                    channel == "base"
                    and self._owner_kind["base"]
                    == OperatorKind.AUTONOMOUS_POLICY
                )
                self._owners[channel] = None
                self._owner_kind[channel] = None
                self._owner_operator[channel] = None
                if channel == "base":
                    self._base_request[:] = 0.0
                    if was_navigation:
                        self._navigation_revoked_sessions.add(owner)
                        self._navigation_session_id = None
                        self._navigation_cancel_epoch += 1
                else:
                    self._cancel_arm_motion("lease_expired")
                self.publisher.emit(
                    "lease_expired",
                    {"channel": channel, "owner_session_id": owner},
                    sim_time_s=float(self.data.time),
                    control_tick_id=self._last_tick_id,
                )
        if self._stop_latched:
            self._base_request[:] = 0.0
        if (
            self._active_plan is not None
            and now - self._active_plan_started_monotonic
            > float(self.safety_config["single_action_timeout_s"])
        ):
            self._fail_active_plan("action_timeout")

        target_wheel = self.wheel.update(self.data, self._base_request, dt)
        self.arm.update(self.data, dt)
        if self._g4a1_pending_reference is not None:
            receipt = self._g4a1_pending_reference
            target = self.arm.state(self.data)["gripper_target_width_m"]
            receipt["actuator_reference_backend_monotonic_s"] = time.monotonic()
            receipt["control_tick_id"] = self._last_tick_id
            self.publisher.emit(
                "g4a1_actuator_reference",
                {
                    "receipt": receipt,
                    "arm_model_reference_rad": self.arm.target[:5].tolist(),
                    "gripper_width_target_m": target,
                },
                sim_time_s=float(self.data.time),
                control_tick_id=self._last_tick_id,
            )
            self._g4a1_pending_reference = None
        for _ in range(substeps):
            mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self._feedback_time = time.monotonic()
        self._sample_fast_sensors()
        self._update_stop_state(float(self.data.time))
        contacts = self._contact_monitor()
        if contacts and not self._stop_latched:
            self._last_unexpected = contacts
            self._base_request[:] = 0.0
            self._cancel_arm_motion("unexpected_contact")
            self._stop_latched = True
            self.publisher.emit(
                "contact_violation",
                {"contacts": contacts},
                sim_time_s=float(self.data.time),
                control_tick_id=self._last_tick_id,
            )
        self._check_plan_completion(now)
        if self._tick_count % 5 == 0:
            gripper_contacts = object_contacts(self)
            if self.target_body_id is None or self.target_qpos is None:
                payload = {
                    "sim_time_s": float(self.data.time),
                    "tcp_position_world_m": self.data.site_xpos[self.tcp_id].tolist(),
                    "left_contact_force_n": float(gripper_contacts["left_force_n"]),
                    "right_contact_force_n": float(gripper_contacts["right_force_n"]),
                    "bilateral_contact": False,
                    "held": False,
                    "oracle_status": "NOT_APPLICABLE",
                    "reason": "target_cube_not_present",
                    "oracle_access": "GT_DEBUG_ONLY",
                }
            else:
                object_z = float(self.data.xpos[self.target_body_id, 2])
                initial_z = float(self.model.qpos0[self.target_qpos + 2])
                payload = {
                    "sim_time_s": float(self.data.time),
                    "cube_position_world_sim_m": self.data.xpos[
                        self.target_body_id
                    ].tolist(),
                    "cube_orientation_world_sim_xyzw": quat_wxyz_to_xyzw(
                        self.data.xquat[self.target_body_id]
                    ),
                    "tcp_position_world_m": self.data.site_xpos[self.tcp_id].tolist(),
                    "left_contact_force_n": float(gripper_contacts["left_force_n"]),
                    "right_contact_force_n": float(gripper_contacts["right_force_n"]),
                    "bilateral_contact": bool(
                        gripper_contacts["bilateral_contact"]
                    ),
                    "held": bool(
                        gripper_contacts["bilateral_contact"]
                        and object_z > initial_z + 0.02
                    ),
                    "oracle_status": "AVAILABLE",
                    "oracle_access": "GT_DEBUG_ONLY",
                }
            self.publisher.emit(
                "oracle_sample",
                payload,
                sim_time_s=float(self.data.time),
                control_tick_id=self._last_tick_id,
            )
        self.publisher.emit(
            "actuator_reference",
            {
                "base_twist": self.wheel.command.tolist(),
                "wheel_target_rad_s": target_wheel.tolist(),
                "wheel_torque_nm": self.wheel.torque.tolist(),
                "arm_model_reference_rad": self.arm.target.tolist(),
                "arm_torque_limit_nm": self.arm.torque_limits.tolist(),
                "gripper_width_target_m": self.arm.state(self.data)["gripper_target_width_m"],
            },
            sim_time_s=float(self.data.time),
            control_tick_id=self._last_tick_id,
        )
        self.publisher.emit(
            "feedback",
            {
                "feedback_time_monotonic": self._feedback_time,
                "sim_time_s": float(self.data.time),
                "base_position_world_sim_m": self.data.xpos[self.base_id].tolist(),
                "base_quaternion_xyzw": quat_wxyz_to_xyzw(self.data.xquat[self.base_id]),
                "base_twist_body_m_s": self._base_velocity().tolist(),
                "wheel_velocity_rad_s": self.data.qvel[self.wheel.dofs].tolist(),
                "arm_model_qpos_rad": self.data.qpos[self.arm.qpos[:5]].tolist(),
                "gripper_width_m": self._snapshot_gripper_width(),
                "tcp_position_world_m": self.data.site_xpos[self.tcp_id].tolist(),
            },
            sim_time_s=float(self.data.time),
            control_tick_id=self._last_tick_id,
        )

    def _sample_fast_sensors(self) -> None:
        now = time.monotonic()
        imu = self._sensor_vectors()
        self._latest_imu = imu
        self._latest_imu_valid = {
            "gyro": bool(imu.pop("gyro_valid")),
            "acceleration": bool(imu.pop("acceleration_valid")),
        }
        for kind in (SensorKind.IMU, SensorKind.ENCODERS, SensorKind.JOINT_FEEDBACK):
            old = self._sensor_states[kind]
            if kind == SensorKind.IMU:
                available = bool(
                    self._latest_imu_valid["gyro"]
                    or self._latest_imu_valid["acceleration"]
                )
                quality = (
                    "simulated_nominal"
                    if available
                    else "sensor_sample_failed"
                )
            else:
                available = kind != SensorKind.ENCODERS or "wheel" not in self._navigation_faults
                quality = (
                    "simulated_nominal"
                    if available
                    else "injected_invalid_wheel_fault"
                )
            self._sensor_states[kind] = old.model_copy(
                update={
                    "available": available,
                    "fresh": available,
                    "measurement_time_s": now,
                    "arrival_time_s": now,
                    "sample_id": f"{kind.value}_{self._tick_count:09d}",
                    "stream_id": kind.value,
                    "quality": quality,
                    "capture_sim_time_s": float(self.data.time),
                    "capture_backend_monotonic_s": now,
                    "capture_control_tick_id": self._last_tick_id,
                    "observation_age_s": 0.0,
                    "profile_id": self.observation_profile["profile_id"],
                }
            )
        scan_current: LaserScanObservation | None = None
        sim_time_s = float(self.data.time)
        scan_due = (
            self.navigation_enabled
            and sim_time_s >= self._navigation_next_scan_sim_s
        )
        if scan_due:
            navigation_clock = self._navigation_clock(now)
            scan_current = self._scan_navigation(navigation_clock)
            self._latest_laser = {
                **scan_current.model_dump(mode="json"),
                "sim_time_s": float(self.data.time),
            }
            self._sensor_states[SensorKind.LIDAR] = self._sensor_states[SensorKind.LIDAR].model_copy(
                update={
                    "available": "scan" not in self._navigation_faults,
                    "fresh": True,
                    "measurement_time_s": now,
                    "arrival_time_s": now,
                    "sample_id": f"laser_{self._tick_count:09d}",
                    "stream_id": "lidar",
                    "quality": (
                        "injected_invalid_scan_fault"
                        if "scan" in self._navigation_faults
                        else "simulated_raycast"
                    ),
                    "capture_sim_time_s": float(self.data.time),
                    "capture_backend_monotonic_s": now,
                    "capture_control_tick_id": self._last_tick_id,
                    "observation_age_s": 0.0,
                    "profile_id": (
                        self.navigation_profile or {}
                    ).get("profile_id", self.observation_profile["profile_id"]),
                }
            )
            sink = self.publisher.sink
            if sink is not None and hasattr(sink, "publish_observation"):
                observation = SensorObservation(
                    run_id=self.run_id,
                    episode_id=self.episode_id,
                    stream_id="lidar",
                    sample_id=f"laser_{self._tick_count:09d}",
                    sensor_kind=SensorKind.LIDAR,
                    measurement_time_s=now,
                    arrival_time_s=now,
                    clock_id="backend_monotonic",
                    frame_id="laser_frame",
                    source="mujoco_mj_ray",
                    quality="simulated_raycast",
                    value=self._latest_laser,
                    capture_sim_time_s=float(self.data.time),
                    capture_backend_monotonic_s=now,
                    capture_control_tick_id=self._last_tick_id,
                    metadata={
                        "interface_profile": "g3a-1.0",
                        "profile_id": scan_current.profile_id,
                        "profile_sha256": scan_current.profile_sha256,
                        "source_sample_period_s": 0.01,
                        "publish_period_s": 0.1,
                        "angular_frame_note": "ranges and angles are represented in laser_frame",
                        "time_increment_s": 0.0,
                        "status_counts": scan_current.status_counts,
                    },
                )
                sink.publish_observation(observation)
            self._navigation_next_scan_sim_s += 0.10
            if sim_time_s - self._navigation_next_scan_sim_s > 0.10:
                self._navigation_next_scan_sim_s = sim_time_s + 0.10
        elif not self.navigation_enabled and self._tick_count % 10 == 0:
            self._publish_legacy_lidar(now)
        if self._tick_count % 10 == 0:
            self._publish_fast_sensor_observations(now)
            self._publish_oracle_sample(now)
        # Deadline scheduling follows the frozen 50 Hz/10 Hz simulation-time
        # profile even when a laser capture makes one control tick overrun.
        navigation_due = (
            self.navigation_enabled
            and (
                scan_due
                or sim_time_s >= self._navigation_next_publish_sim_s
            )
        )
        if navigation_due:
            self._publish_navigation_observation(now, scan_current)
            self._navigation_next_publish_sim_s += 0.02
            if sim_time_s - self._navigation_next_publish_sim_s > 0.04:
                self._navigation_next_publish_sim_s = sim_time_s + 0.02

    def _publish_fast_sensor_observations(self, now: float) -> None:
        """Publish G2A-compatible 10 Hz scalar/vector summary streams."""

        sink = self.publisher.sink
        if sink is None or not hasattr(sink, "publish_observation"):
            return
        base = dict(
            run_id=self.run_id,
            episode_id=self.episode_id,
            measurement_time_s=now,
            arrival_time_s=now,
            clock_id="backend_monotonic",
            source="mujoco_sensor",
            capture_sim_time_s=float(self.data.time),
            capture_backend_monotonic_s=now,
            capture_control_tick_id=self._last_tick_id,
        )
        streams = (
            (
                SensorKind.IMU,
                "imu",
                "imu_link" if self.navigation_enabled else "base_link",
                {
                    **self._latest_imu,
                    "gyro_valid": self._latest_imu_valid["gyro"],
                    "acceleration_valid": self._latest_imu_valid["acceleration"],
                    "measurement_time_s": now,
                },
            ),
            (
                SensorKind.ENCODERS,
                "encoders",
                "base_link",
                {
                    "wheel_joint_positions_rad": self.data.qpos[
                        [int(self.model.joint(name + "_joint").qposadr) for name in
                         ("front_left", "rear_left", "front_right", "rear_right")]
                    ].tolist(),
                    "wheel_velocity_rad_s": self.data.qvel[self.wheel.dofs].tolist(),
                    "base_twist_body_m_s": self._base_velocity().tolist(),
                    "valid": "wheel" not in self._navigation_faults,
                },
            ),
            (
                SensorKind.JOINT_FEEDBACK,
                "joint_feedback",
                "arm_base",
                {
                    "model_joint_positions_rad": self.data.qpos[self.arm.qpos[:5]].tolist(),
                    "user_joint_positions_rad": self.arm.coordinates.to_user(
                        self.data.qpos[self.arm.qpos[:5]]
                    ).tolist(),
                    "joint_velocities_rad_s": self.data.qvel[self.arm.dofs[:5]].tolist(),
                    "joint_torques_nm": self.data.actuator_force[
                        self.arm.actuators[:5]
                    ].tolist(),
                    "gripper_width_m": self._snapshot_gripper_width(),
                },
            ),
        )
        for kind, stream_id, frame_id, value in streams:
            sample_id = str(self._sensor_states[kind].sample_id)
            quality = str(self._sensor_states[kind].quality)
            sink.publish_observation(
                SensorObservation(
                    **base,
                    stream_id=stream_id,
                    sample_id=sample_id,
                    sensor_kind=kind,
                    frame_id=frame_id,
                    quality=quality,
                    value=value,
                    metadata={
                        "interface_profile": "g2a-1.0",
                        "source_sample_period_s": 0.01,
                        "publish_period_s": 0.1,
                        "control_tick_id": self._last_tick_id,
                    },
                )
            )

    def _publish_oracle_sample(self, now: float) -> None:
        """Emit explicit GT_DEBUG samples into the Recorder's isolated oracle path."""

        contacts = object_contacts(self)
        if self.target_body_id is None or self.target_qpos is None:
            payload = {
                "measurement_time_s": now,
                "tcp_position_world_sim_m": self.data.site_xpos[self.tcp_id].tolist(),
                "left_contact_force_n": 0.0,
                "right_contact_force_n": 0.0,
                "bilateral_contact": False,
                "support_contact": False,
                "held": False,
                "oracle_status": "NOT_APPLICABLE",
                "reason": "target_cube_not_present",
                "oracle_access": "GT_DEBUG_ONLY",
            }
        else:
            initial_object_z = float(self.model.qpos0[self.target_qpos + 2])
            object_z = float(self.data.xpos[self.target_body_id, 2])
            payload = {
                "measurement_time_s": now,
                "cube_position_world_sim_m": self.data.xpos[
                    self.target_body_id
                ].tolist(),
                "cube_orientation_world_sim_xyzw": quat_wxyz_to_xyzw(
                    self.data.xquat[self.target_body_id]
                ),
                "left_contact_force_n": float(contacts["left_force_n"]),
                "right_contact_force_n": float(contacts["right_force_n"]),
                "bilateral_contact": bool(contacts["bilateral_contact"]),
                "support_contact": bool(contacts["support_force_n"] > 0.0),
                "tcp_position_world_sim_m": self.data.site_xpos[self.tcp_id].tolist(),
                "held": bool(
                    contacts["bilateral_contact"]
                    and object_z > initial_object_z + 0.02
                ),
                "object_lift_m": object_z - initial_object_z,
                "oracle_status": "AVAILABLE",
                "oracle_access": "GT_DEBUG_ONLY",
            }
        self.publisher.emit(
            "oracle_sample",
            payload,
            sim_time_s=float(self.data.time),
            control_tick_id=self._last_tick_id,
            task_phase=self._task_phase.value,
        )

    def _sensor_vectors(self) -> dict[str, list[float]]:
        result: dict[str, Any] = {}
        for names, key in (
            (("navigation_imu_gyro", "imu_gyro"), "gyro_rad_s"),
            (("navigation_imu_accel", "imu_accel"), "acceleration_m_s2"),
        ):
            value = None
            for name in names:
                try:
                    candidate = self.data.sensor(name).data.astype(float)
                    if candidate.shape == (3,) and np.isfinite(candidate).all():
                        value = candidate.tolist()
                        break
                except Exception:
                    continue
            result[key] = value
        result["gyro_valid"] = result["gyro_rad_s"] is not None and "imu" not in self._navigation_faults
        result["acceleration_valid"] = (
            result["acceleration_m_s2"] is not None
            and "imu" not in self._navigation_faults
        )
        if not result["gyro_valid"]:
            result["gyro_rad_s"] = None
        if not result["acceleration_valid"]:
            result["acceleration_m_s2"] = None
        return result

    def _navigation_clock(self, now: float) -> NavigationClockV1:
        return NavigationClockV1(
            run_id=self.run_id,
            episode_id=self.episode_id,
            backend_generation=self.backend_generation,
            navigation_epoch=self._navigation_epoch,
            control_tick_id=self._last_tick_id,
            capture_sim_time_s=float(self.data.time),
            capture_backend_monotonic_s=float(now),
        )

    def _is_robot_geom(self, geom_id: int) -> bool:
        if geom_id < 0:
            return False
        body_id = int(self.model.geom_bodyid[geom_id])
        while body_id > 0:
            if body_id == self.base_id:
                return True
            body_id = int(self.model.body_parentid[body_id])
        return False

    def _scan_navigation(
        self, clock: NavigationClockV1
    ) -> LaserScanObservation:
        if self.navigation_profile is None or self.navigation_profile_sha256 is None:
            raise RuntimeError("navigation scan requested without profile")
        scan_config = self.navigation_profile["sensors"]["scan"]
        count = int(scan_config["beam_count"])
        angle_min = float(scan_config["angle_min_rad"])
        increment = float(scan_config["angle_increment_rad"])
        range_min = float(scan_config["range_min_m"])
        range_max = float(scan_config["range_max_m"])
        ranges: list[float | None] = []
        statuses: list[LaserReturnStatus] = []
        faulted = "scan" in self._navigation_faults
        mount = self.navigation_profile["frames"]["t_base_lidar"]
        base_rotation = self.data.xmat[self.base_id].reshape(3, 3)
        mount_rotation = _rotation_from_quaternion_xyzw(
            mount["rotation_xyzw"]
        )
        origin = (
            self.data.xpos[self.base_id]
            + base_rotation @ np.asarray(mount["translation_m"], dtype=float)
        )
        # Include robot collision and visual mesh groups so a ray cannot pass
        # through the modeled chassis/arm merely because a visual mesh has no
        # collision affinity.
        geomgroup = np.array([1, 1, 0, 1, 0, 0], dtype=np.uint8)
        hit_id = np.array([-1], dtype=np.int32)
        for index in range(count):
            if faulted:
                ranges.append(None)
                statuses.append(LaserReturnStatus.INVALID)
                continue
            angle = angle_min + index * increment
            local_direction = np.array(
                [math.cos(angle), math.sin(angle), 0.0], dtype=float
            )
            direction = base_rotation @ mount_rotation @ local_direction
            hit_id[0] = -1
            distance = float(
                mujoco.mj_ray(
                    self.model,
                    self.data,
                    origin,
                    direction,
                    geomgroup,
                    1,
                    -1,
                    hit_id,
                )
            )
            robot_hit = self._is_robot_geom(int(hit_id[0]))
            if robot_hit and distance >= 0.0 and distance <= range_max:
                ranges.append(None)
                statuses.append(LaserReturnStatus.SELF_OCCLUDED)
            elif distance < 0.0 or distance > range_max:
                ranges.append(None)
                statuses.append(LaserReturnStatus.NO_RETURN)
            elif distance < range_min:
                ranges.append(None)
                statuses.append(LaserReturnStatus.INVALID)
            else:
                ranges.append(distance)
                statuses.append(LaserReturnStatus.HIT)
        counts = {
            status.value: sum(1 for item in statuses if item == status)
            for status in LaserReturnStatus
        }
        angle_max = angle_min + (count - 1) * increment
        return LaserScanObservation(
            profile_id=str(self.navigation_profile["profile_id"]),
            profile_version=str(self.navigation_profile["profile_version"]),
            profile_sha256=self.navigation_profile_sha256,
            clock=clock,
            scan_id=f"scan_{self._tick_count:09d}",
            angle_min_rad=angle_min,
            angle_max_rad=angle_max,
            angle_increment_rad=increment,
            range_min_m=range_min,
            range_max_m=range_max,
            ranges_m=ranges,
            statuses=statuses,
            beam_count=count,
            valid_count=counts[LaserReturnStatus.HIT.value],
            status_counts=counts,
        )

    def _publish_navigation_observation(
        self,
        now: float,
        scan: LaserScanObservation | None,
    ) -> None:
        if self.navigation_profile is None or self.navigation_profile_sha256 is None:
            return
        profile = self.navigation_profile
        clock = self._navigation_clock(now)
        wheel_names = [
            f"{name}_wheel"
            for name in ("front_left", "rear_left", "front_right", "rear_right")
        ]
        wheel_joint_qpos = [
            int(self.model.jnt_qposadr[self.model.joint(name + "_joint").id])
            for name in ("front_left", "rear_left", "front_right", "rear_right")
        ]
        wheel_fault = "wheel" in self._navigation_faults
        wheels = WheelMeasurement(
            sample_id=f"wheel_{self._tick_count:09d}",
            clock=clock,
            wheel_names=wheel_names,
            wheel_angles_rad=[
                float(self.data.qpos[index]) for index in wheel_joint_qpos
            ],
            wheel_velocities_rad_s=self.data.qvel[self.wheel.dofs].astype(float).tolist(),
            wheel_radius_m=[
                float(value)
                for value in profile["sensors"]["wheel"]["radius_m"]
            ],
            wheel_rotation_sign=[
                float(value)
                for value in profile["sensors"]["wheel"]["rotation_sign"]
            ],
            valid=not wheel_fault,
            quality=(
                "injected_invalid_wheel_fault"
                if wheel_fault
                else "simulated_nominal"
            ),
            source=str(profile["sensors"]["wheel"]["source"]),
        )
        gyro_valid = bool(self._latest_imu_valid["gyro"])
        acceleration_valid = bool(self._latest_imu_valid["acceleration"])
        imu = ImuMeasurement(
            sample_id=f"imu_{self._tick_count:09d}",
            clock=clock,
            rotation_imu_to_base_wxyz=[
                float(value)
                for value in profile["sensors"]["imu"][
                    "rotation_imu_to_base_wxyz"
                ]
            ],
            angular_velocity_rad_s=(
                list(self._latest_imu["gyro_rad_s"]) if gyro_valid else None
            ),
            linear_acceleration_m_s2=(
                list(self._latest_imu["acceleration_m_s2"])
                if acceleration_valid
                else None
            ),
            gyro_valid=gyro_valid,
            acceleration_valid=acceleration_valid,
            quality=(
                "simulated_nominal"
                if gyro_valid
                else "imu_gyro_invalid"
            ),
            source=str(profile["sensors"]["imu"]["source"]),
        )
        joints = NavigationJointMeasurement(
            sample_id=f"joint_{self._tick_count:09d}",
            clock=clock,
            joint_names=ARM_JOINTS,
            model_positions_rad=self.data.qpos[self.arm.qpos[:5]].astype(float).tolist(),
            available=True,
            quality="simulated_nominal",
        )
        self._navigation_sequence += 1
        observation = NavigationObservation(
            profile_id=str(profile["profile_id"]),
            profile_version=str(profile["profile_version"]),
            profile_sha256=self.navigation_profile_sha256,
            sequence=self._navigation_sequence,
            clock=clock,
            wheels=wheels,
            imu=imu,
            joints=joints,
            scan=scan,
        )
        self.publisher.publish_navigation(observation)
        self.publisher.emit(
            "navigation_observation",
            {"navigation_observation": observation.model_dump(mode="json")},
            sim_time_s=float(self.data.time),
            control_tick_id=self._last_tick_id,
        )
        self._navigation_references[self._last_tick_id] = {
            "truth_label": "GT_DEBUG_ONLY",
            "control_tick_id": self._last_tick_id,
            "capture_sim_time_s": float(self.data.time),
            "base_position_world_sim_m": self.data.xpos[self.base_id].tolist(),
            "base_quaternion_world_xyzw": quat_wxyz_to_xyzw(
                self.data.xquat[self.base_id]
            ),
            "arm_model_joint_positions_rad": self.data.qpos[
                self.arm.qpos[:5]
            ].astype(float).tolist(),
        }
        while len(self._navigation_references) > 2000:
            self._navigation_references.popitem(last=False)

    def _publish_legacy_lidar(self, now: float) -> None:
        """Preserve the G2A lidar stream when navigation is disabled."""

        self._latest_laser = self._scan_laser()
        self._sensor_states[SensorKind.LIDAR] = self._sensor_states[
            SensorKind.LIDAR
        ].model_copy(
            update={
                "available": True,
                "fresh": True,
                "measurement_time_s": now,
                "arrival_time_s": now,
                "sample_id": f"laser_{self._tick_count:09d}",
                "stream_id": "lidar",
                "quality": "simulated_raycast",
                "capture_sim_time_s": float(self.data.time),
                "capture_backend_monotonic_s": now,
                "capture_control_tick_id": self._last_tick_id,
                "observation_age_s": 0.0,
                "profile_id": self.observation_profile["profile_id"],
            }
        )
        sink = self.publisher.sink
        if sink is not None and hasattr(sink, "publish_observation"):
            sink.publish_observation(
                SensorObservation(
                    run_id=self.run_id,
                    episode_id=self.episode_id,
                    stream_id="lidar",
                    sample_id=f"laser_{self._tick_count:09d}",
                    sensor_kind=SensorKind.LIDAR,
                    measurement_time_s=now,
                    arrival_time_s=now,
                    clock_id="backend_monotonic",
                    frame_id="base_link",
                    source="mujoco_mj_ray",
                    quality="simulated_raycast",
                    value=self._latest_laser,
                    capture_sim_time_s=float(self.data.time),
                    capture_backend_monotonic_s=now,
                    capture_control_tick_id=self._last_tick_id,
                    metadata={
                        "interface_profile": "g2a-1.0",
                        "source_sample_period_s": 0.01,
                        "publish_period_s": 0.1,
                        "angular_frame_note": "legacy G2A base-aligned scan",
                    },
                )
            )

    def _scan_laser(self) -> dict[str, Any]:
        count = 361
        ranges: list[float | None] = [None] * count
        origin = self.data.xpos[self.base_id] + np.array([0.0, 0.0, 0.08])
        geomgroup = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
        hit_id = np.array([-1], dtype=np.int32)
        for index in range(count):
            angle = -math.pi + index * (2 * math.pi / (count - 1))
            direction = np.array([math.cos(angle), math.sin(angle), 0.0])
            distance = float(
                mujoco.mj_ray(
                    self.model,
                    self.data,
                    origin,
                    direction,
                    geomgroup,
                    1,
                    -1,
                    hit_id,
                )
            )
            if 0.03 <= distance <= 4.0:
                ranges[index] = distance
        return {
            "angle_min_rad": -math.pi,
            "angle_max_rad": math.pi,
            "angle_increment_rad": 2 * math.pi / (count - 1),
            "range_max_m": 4.0,
            "ranges_m": ranges,
            "sim_time_s": float(self.data.time),
        }

    def _sensor_loop(self) -> None:
        try:
            while not self.stop_event.is_set():
                if not self._sensor_capture_enabled.is_set():
                    self.stop_event.wait(0.05)
                    self._rgbd_deadline = time.monotonic() + self._rgbd_period_s
                    continue
                if not bool(self.backend_config.get("render_rgbd", True)):
                    self.stop_event.wait(self._rgbd_period_s)
                    continue
                delay = self._rgbd_deadline - time.monotonic()
                if delay > 0 and self.stop_event.wait(delay):
                    break
                if self.stop_event.is_set():
                    break
                now = time.monotonic()
                late_s = max(0.0, now - self._rgbd_deadline)
                missed = int(late_s // self._rgbd_period_s)
                if missed:
                    self._rgbd_skipped_slots += missed
                    self._rgbd_skip_reasons["deadline_overrun"] += missed
                self._rgbd_schedule_index += missed + 1
                schedule = {
                    "schedule_index": self._rgbd_schedule_index,
                    "deadline_lateness_ms": late_s * 1000.0,
                    "missed_slots": missed,
                }
                self._rgbd_deadline += (missed + 1) * self._rgbd_period_s
                try:
                    self._capture_rgbd(schedule=schedule)
                except Exception as exc:
                    self._sensor_error = f"{type(exc).__name__}: {exc}"
                    for kind in (SensorKind.RGB, SensorKind.DEPTH, SensorKind.IMAGE_TRANSFORMS):
                        self._sensor_states[kind] = self._sensor_states[kind].model_copy(
                            update={"available": False, "fresh": False, "quality": self._sensor_error}
                        )
        finally:
            if self._renderer is not None:
                self._renderer.close()
                self._renderer = None

    def _capture_rgbd(self, *, schedule: dict[str, Any] | None = None) -> None:
        schedule = dict(schedule or {})
        capture_requested = time.monotonic()
        snapshot = mujoco.MjData(self.model)
        lock_started = time.monotonic()
        with self.lock:
            capture_backend_monotonic = time.monotonic()
            capture_sim_time = float(self.data.time)
            capture_control_tick = self._last_tick_id
            feedback_state = self._sensor_states[SensorKind.JOINT_FEEDBACK]
            feedback_sample_id = feedback_state.sample_id
            feedback_time_s = feedback_state.measurement_time_s
            snapshot.time = capture_sim_time
            snapshot.qpos[:] = self.data.qpos
            snapshot.qvel[:] = self.data.qvel
            snapshot.act[:] = self.data.act
            snapshot.ctrl[:] = self.data.ctrl
        copy_complete = time.monotonic()
        mujoco.mj_forward(self.model, snapshot)
        forward_complete = time.monotonic()
        render_started = time.monotonic()
        if self._renderer is None:
            self._renderer = mujoco.Renderer(
                self.model,
                height=int(self.params["simulation"]["camera"]["height"]),
                width=int(self.params["simulation"]["camera"]["width"]),
            )
        self._renderer.disable_depth_rendering()
        self._renderer.update_scene(snapshot, camera="rgbd")
        rgb = self._renderer.render().copy()
        self._renderer.enable_depth_rendering()
        self._renderer.update_scene(snapshot, camera="rgbd")
        depth = self._renderer.render().copy()
        self._renderer.disable_depth_rendering()
        render_complete = time.monotonic()
        camera_cfg = self.params["simulation"]["camera"]
        valid = np.isfinite(depth) & (depth >= camera_cfg["depth_min_m"]) & (
            depth <= camera_cfg["depth_max_m"]
        )
        depth[~valid] = np.nan
        sample_id = new_id("rgbd")
        encode_started = time.monotonic()
        encode_complete = encode_started
        with self._sensor_commit_lock:
            if not self._sensor_capture_enabled.is_set():
                return
            recording = self.publisher.recording_gate(self.episode_id)
            relative_rgb = relative_depth = relative_mask = None
            metadata = self._image_transforms(snapshot)
            metadata.update(
                {
                    "sample_id": sample_id,
                    "width": int(camera_cfg["width"]),
                    "height": int(camera_cfg["height"]),
                    "vertical_fov_deg": float(camera_cfg["vertical_fov_deg"]),
                    "depth_convention": "optical_axis_z_metres_invalid_nan",
                    "valid_fraction": float(valid.mean()),
                    "recording_gate": recording,
                }
            )
            if recording:
                folder = self._incoming_root / "rgbd"
                folder.mkdir(parents=True, exist_ok=True)
                rgb_path = folder / f"{sample_id}_rgb.png"
                depth_path = folder / f"{sample_id}_depth_m.npy"
                mask_path = folder / f"{sample_id}_valid.png"
                Image.fromarray(rgb).save(rgb_path)
                np.save(depth_path, depth)
                Image.fromarray((valid.astype(np.uint8) * 255)).save(mask_path)
                encode_complete = time.monotonic()
                relative_rgb = rgb_path.relative_to(self.run_root).as_posix()
                relative_depth = depth_path.relative_to(self.run_root).as_posix()
                relative_mask = mask_path.relative_to(self.run_root).as_posix()
                metadata.update(
                    {
                        "rgb_path": relative_rgb,
                        "depth_path": relative_depth,
                        "valid_mask_path": relative_mask,
                    }
                )
            publish_started = time.monotonic()
            arrival = publish_started
            timing = {
                "interface_profile": self.observation_profile.get(
                    "interface_profile", "g2a-1.0"
                ),
                "capture_clocks": {
                    "sim": "sim",
                    "backend_monotonic": "backend_monotonic",
                    "wall_utc": "wall_utc",
                },
                "capture_sim_time_s": capture_sim_time,
                "capture_backend_monotonic_s": capture_backend_monotonic,
                "capture_control_tick_id": capture_control_tick,
                "feedback_sample_id": feedback_sample_id,
                "feedback_time_s": feedback_time_s,
                "feedback_clock": "backend_monotonic",
                "render_start_backend_monotonic_s": render_started,
                "render_complete_backend_monotonic_s": render_complete,
                "publish_backend_monotonic_s": publish_started,
                "arrival_backend_monotonic_s": arrival,
                "capture_to_copy_ms": (copy_complete - capture_requested) * 1000.0,
                "lock_acquire_ms": (
                    (capture_backend_monotonic - lock_started) * 1000.0
                ),
                "state_copy_ms": (
                    (copy_complete - capture_backend_monotonic) * 1000.0
                ),
                "capture_to_forward_ms": (forward_complete - capture_backend_monotonic)
                * 1000.0,
                "render_ms": (render_complete - render_started) * 1000.0,
                "encode_ms": (encode_complete - encode_started) * 1000.0
                if recording
                else 0.0,
                "capture_to_render_complete_ms": (
                    render_complete - capture_backend_monotonic
                )
                * 1000.0,
                "render_complete_to_publish_ms": (
                    publish_started - render_complete
                )
                * 1000.0,
                "observation_age_ms": (
                    arrival - capture_backend_monotonic
                )
                * 1000.0,
                "lock_wait_ms": (
                    (capture_backend_monotonic - lock_started) * 1000.0
                ),
                "schedule": schedule,
            }
            metadata.update(
                {
                    "profile_id": self.observation_profile["profile_id"],
                    "profile_version": self.observation_profile["profile_version"],
                    "profile_sha256": self.observation_profile_sha256,
                    "interface_profile": self.observation_profile.get(
                        "interface_profile", "g2a-1.0"
                    ),
                    "timing": timing,
                }
            )
            self._latest_rgbd_metadata = metadata
            self._last_capture_metric = timing
            for kind in (SensorKind.RGB, SensorKind.DEPTH, SensorKind.IMAGE_TRANSFORMS):
                self._sensor_states[kind] = SensorState(
                    kind=kind,
                    available=True,
                    fresh=True,
                    sample_id=sample_id,
                    stream_id="rgbd",
                    measurement_time_s=capture_backend_monotonic,
                    arrival_time_s=arrival,
                    clock_id="backend_monotonic",
                    valid_fraction=float(valid.mean()),
                    asset_uri=relative_rgb
                    if kind == SensorKind.RGB
                    else (relative_depth if kind == SensorKind.DEPTH else None),
                    quality="simulated_nominal",
                    capture_sim_time_s=capture_sim_time,
                    capture_backend_monotonic_s=capture_backend_monotonic,
                    capture_control_tick_id=capture_control_tick,
                    feedback_sample_id=feedback_sample_id,
                    observation_age_s=arrival - capture_backend_monotonic,
                    profile_id=self.observation_profile["profile_id"],
                )
            sink = self.publisher.sink
            if recording and sink is not None and hasattr(sink, "publish_observation"):
                base = dict(
                    run_id=self.run_id,
                    episode_id=self.episode_id,
                    stream_id="rgbd",
                    sample_id=sample_id,
                    measurement_time_s=capture_backend_monotonic,
                    arrival_time_s=arrival,
                    clock_id="backend_monotonic",
                    source="mujoco_renderer",
                    quality="simulated_nominal",
                )
                sink.publish_observation(
                    SensorObservation(
                        **base,
                        sensor_kind=SensorKind.RGB,
                        frame_id="rgbd_optical",
                        value={"width": camera_cfg["width"], "height": camera_cfg["height"]},
                        asset_uri=relative_rgb,
                        metadata=metadata,
                        capture_sim_time_s=capture_sim_time,
                        capture_backend_monotonic_s=capture_backend_monotonic,
                        capture_control_tick_id=capture_control_tick,
                        feedback_sample_id=feedback_sample_id,
                        timing=timing,
                    )
                )
                sink.publish_observation(
                    SensorObservation(
                        **base,
                        sensor_kind=SensorKind.DEPTH,
                        frame_id="rgbd_optical",
                        value=metadata,
                        asset_uri=relative_depth,
                        valid_mask_uri=relative_mask,
                        metadata=metadata,
                        capture_sim_time_s=capture_sim_time,
                        capture_backend_monotonic_s=capture_backend_monotonic,
                        capture_control_tick_id=capture_control_tick,
                        feedback_sample_id=feedback_sample_id,
                        timing=timing,
                    )
                )
            publish_complete = time.monotonic()
            timing["publish_ms"] = (publish_complete - publish_started) * 1000.0
            timing["capture_to_publish_complete_ms"] = (
                publish_complete - capture_backend_monotonic
            ) * 1000.0
            timing["wall_utc"] = utc_now()
            self._last_capture_metric = timing
            self.publisher.emit(
                "rgbd_capture_timing",
                {"sample_id": sample_id, **timing},
                sim_time_s=capture_sim_time,
                control_tick_id=capture_control_tick,
            )

    def _image_transforms(self, snapshot: mujoco.MjData) -> dict[str, Any]:
        base_id = self.base_id
        tcp_id = self.tcp_id
        camera_id = self.camera_id
        t_world_base = transform_from_world(
            snapshot.xpos[base_id], snapshot.xmat[base_id]
        )
        t_world_link4 = transform_from_world(
            snapshot.xpos[self.link4_id], snapshot.xmat[self.link4_id]
        )
        t_world_camera = transform_from_world(
            snapshot.cam_xpos[camera_id], snapshot.cam_xmat[camera_id]
        )
        t_world_tcp = transform_from_world(
            snapshot.site_xpos[tcp_id], snapshot.site_xmat[tcp_id]
        )
        t_base_link4 = np.linalg.inv(t_world_base) @ t_world_link4
        t_base_camera = np.linalg.inv(t_world_base) @ t_world_camera
        t_base_tcp = np.linalg.inv(t_world_base) @ t_world_tcp
        return {
            "transform_version": "g2a-transform-1",
            "matrix_storage": "row_major",
            "T_base_link4": t_base_link4.tolist(),
            "T_base_camera_render": t_base_camera.tolist(),
            "T_base_tcp": t_base_tcp.tolist(),
            "T_link4_camera": (np.linalg.inv(t_base_link4) @ t_base_camera).tolist(),
            "T_TCP_camera": (np.linalg.inv(t_base_tcp) @ t_base_camera).tolist(),
            "joint_model_qpos_rad": snapshot.qpos[self.arm.qpos[:5]].tolist(),
            "joint_user_qpos_rad": self.arm.coordinates.to_user(
                snapshot.qpos[self.arm.qpos[:5]]
            ).tolist(),
            "calibration_id": "user_defined_photo_pose_not_hardware_500",
            "rgbd_optical_convention": "X_right_Y_down_Z_forward",
        }

    def _base_velocity(self) -> np.ndarray:
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(
            self.model,
            self.data,
            mujoco.mjtObj.mjOBJ_BODY,
            self.base_id,
            velocity,
            1,
        )
        return np.array([velocity[3], velocity[4], velocity[2]])

    def _update_stop_state(self, now: float) -> None:
        """Use simulation time so no-realtime probes preserve the 0.5 s rule."""

        velocity = self._base_velocity()
        slow = (
            np.linalg.norm(velocity[:2]) <= float(self.safety_config["stop_settle_linear_m_s"])
            and abs(velocity[2]) <= float(self.safety_config["stop_settle_yaw_rad_s"])
        )
        if slow:
            if self._stopped_since is None:
                self._stopped_since = now
            duration = now - self._stopped_since
            self._stopped = duration >= float(self.safety_config["stop_settle_duration_s"])
        else:
            self._stopped_since = None
            self._stopped = False

    def _contact_monitor(self) -> list[dict[str, Any]]:
        unexpected: list[dict[str, Any]] = []
        approved_grasp = self._task_phase in (
            TaskPhase.APPROACH,
            TaskPhase.MANIPULATE,
            TaskPhase.VERIFY,
        )
        object_state = object_contacts(self)
        if self.target_body_id is None or self.target_qpos is None:
            carried_object = False
        else:
            initial_object_z = float(self.model.qpos0[self.target_qpos + 2])
            carried_object = bool(
                object_state["bilateral_contact"]
                and self.data.xpos[self.target_body_id, 2]
                > initial_object_z + 0.02
            )
        for contact in self.data.contact:
            names = tuple(
                self.model.geom(index).name or f"geom_{index}"
                for index in (contact.geom1, contact.geom2)
            )
            if contact.dist >= 0:
                continue
            if any(_is_wheel(name) for name in names) and any(
                name in ("floor", "pick_platform") for name in names
            ):
                continue
            if "target_cube_contact" in names and (
                "pick_platform" in names or "floor" in names
            ):
                if not carried_object:
                    continue
                unexpected.append(
                    {
                        "geoms": list(names),
                        "distance_m": float(contact.dist),
                        "approved": False,
                        "reason": "carried_object_environment_contact",
                    }
                )
                continue
            fingers = any(
                name.startswith(("left_jaw_link_", "right_jaw_link_"))
                for name in names
            )
            if approved_grasp and "target_cube_contact" in names and fingers:
                continue
            robot_contact = any(
                name.startswith(
                    (
                        "chassis_",
                        "link",
                        "gripper",
                        "guide_rod",
                        "left_jaw",
                        "right_jaw",
                        "front_",
                        "rear_",
                    )
                )
                for name in names
            )
            if any(_is_wheel(name) for name in names) and all(
                _is_wheel(name) or name.startswith(("floor", "pick_platform"))
                for name in names
            ):
                continue
            if robot_contact:
                unexpected.append(
                    {
                        "geoms": list(names),
                        "distance_m": float(contact.dist),
                        "approved": False,
                    }
                )
        return unexpected

    def _check_plan_completion(self, now: float) -> None:
        plan = self._active_plan
        if plan is None:
            return
        elapsed = float(self.data.time) - self._active_plan_started_sim
        if elapsed < plan.trajectory.duration_s:
            return
        tcp_position = self.data.site_xpos[self.tcp_id]
        position_error = float(np.linalg.norm(tcp_position - plan.target_position_world))
        tolerance = float(self.safety_config["tcp_position_tolerance_m"])
        if position_error > tolerance:
            if elapsed - plan.trajectory.duration_s > 0.75:
                self._fail_active_plan(
                    f"tracking residual {position_error:.5f} m exceeds {tolerance:.5f} m"
                )
            return
        if self._reached_since is None:
            self._reached_since = now
        if now - self._reached_since >= float(self.safety_config["tcp_hold_duration_s"]):
            self.arm.stop_trajectory(self.data)
            plan.preview = plan.preview.model_copy(update={"status": "REACHED"})
            self.publisher.emit(
                "reached",
                {
                    "plan_id": plan.preview.plan_id,
                    "position_error_m": position_error,
                    "orientation_error_rad": self._tcp_orientation_error(
                        plan.target_rotation_world
                    ),
                },
                command=self._active_command,
                plan_id=plan.preview.plan_id,
                sim_time_s=float(self.data.time),
            )
            self._active_plan = None
            self._active_command = None
            self._reached_since = None

    def _fail_active_plan(self, reason: str) -> None:
        plan = self._active_plan
        if plan is None:
            return
        self.arm.stop_trajectory(self.data)
        self.publisher.emit(
            "failed",
            {"plan_id": plan.preview.plan_id, "reason": reason},
            command=self._active_command,
            plan_id=plan.preview.plan_id,
            sim_time_s=float(self.data.time),
        )
        self._active_plan = None
        self._active_command = None
        self._reached_since = None

    def _snapshot_gripper_width(self) -> float:
        return float(
            np.linalg.norm(
                self.data.site_xpos[self.arm.left_id] - self.data.site_xpos[self.arm.right_id]
            )
        )

    def _tcp_orientation_error(self, target_rotation: np.ndarray) -> float:
        current = self.data.site_xmat[self.tcp_id].reshape(3, 3)
        cosine = np.clip((np.trace(current.T @ target_rotation) - 1.0) / 2.0, -1.0, 1.0)
        return float(math.acos(cosine))

    def _build_snapshot(self) -> RobotSnapshot:
        now = time.monotonic()
        base_velocity = self._base_velocity()
        user_q = self.arm.coordinates.to_user(self.data.qpos[self.arm.qpos[:5]])
        user_target = self.arm.coordinates.to_user(self.arm.target[:5])
        torques = self.data.actuator_force[self.arm.actuators[:5]].copy()
        margins = np.minimum(
            self.data.qpos[self.arm.qpos[:5]] - self.arm.limits[:5, 0],
            self.arm.limits[:5, 1] - self.data.qpos[self.arm.qpos[:5]],
        )
        contacts = object_contacts(self)
        tcp_position = self.data.site_xpos[self.tcp_id].copy()
        tcp_quat = muJoCo_mat_to_xyzw(self.data.site_xmat[self.tcp_id])
        plan_preview = None
        position_error = None
        orientation_error = None
        if self._active_plan is not None:
            plan_preview = self._active_plan.preview.model_copy(update={"status": "EXECUTING"})
            position_error = float(
                np.linalg.norm(tcp_position - self._active_plan.target_position_world)
            )
            orientation_error = self._tcp_orientation_error(
                self._active_plan.target_rotation_world
            )
        elif self._previews:
            latest_plan = max(
                self._previews.values(), key=lambda item: item.created_monotonic_s
            )
            plan_preview = latest_plan.preview
            position_error = float(
                np.linalg.norm(tcp_position - latest_plan.target_position_world)
            )
            orientation_error = self._tcp_orientation_error(
                latest_plan.target_rotation_world
            )
        if self.target_body_id is None or self.target_qpos is None:
            held = False
            object_diagnostic: dict[str, Any] = {
                "status": "NOT_APPLICABLE",
                "reason": "target_cube_not_present",
            }
        else:
            initial_object_z = float(self.model.qpos0[self.target_qpos + 2])
            held = bool(
                contacts["bilateral_contact"]
                and self.data.xpos[self.target_body_id, 2]
                > initial_object_z + 0.02
            )
            object_diagnostic = {
                "status": "AVAILABLE",
                "position_m": self.data.xpos[self.target_body_id].tolist(),
                "orientation_xyzw": quat_wxyz_to_xyzw(
                    self.data.xquat[self.target_body_id]
                ),
            }
        return RobotSnapshot(
            run_id=self.run_id,
            backend_generation=self.backend_generation,
            episode_id=self.episode_id,
            episode_started_backend_monotonic_s=self.episode_started,
            backend_monotonic_s=now,
            sim_time_s=float(self.data.time),
            feedback_time_s=self._feedback_time,
            wall_utc=utc_now(),
            clock_id="backend_monotonic",
            control_tick_id=self._last_tick_id,
            task_phase=self._task_phase,
            base=BaseState(
                position_world_sim_m=self.data.xpos[self.base_id].tolist(),
                orientation_world_sim_xyzw=quat_wxyz_to_xyzw(self.data.xquat[self.base_id]),
                vx_m_s=float(base_velocity[0]),
                vy_m_s=float(base_velocity[1]),
                wz_rad_s=float(base_velocity[2]),
                requested_vx_m_s=float(self._base_request[0]),
                requested_vy_m_s=float(self._base_request[1]),
                requested_wz_rad_s=float(self._base_request[2]),
                stopped=bool(self._stopped),
                stop_latched=bool(self._stop_latched),
            ),
            arm=ArmState(
                user_joint_positions_rad=user_q.tolist(),
                user_joint_targets_rad=user_target.tolist(),
                model_joint_positions_rad=self.data.qpos[self.arm.qpos[:5]].tolist(),
                model_joint_targets_rad=self.arm.target[:5].tolist(),
                joint_velocities_rad_s=self.data.qvel[self.arm.dofs[:5]].tolist(),
                joint_torques_nm=torques.tolist(),
                joint_margin_rad=margins.tolist(),
                model_limits_rad=self.arm.limits[:5].tolist(),
                coordinates_status=self.arm_spec["user_coordinates"]["status"],
            ),
            gripper=GripperState(
                width_m=self._snapshot_gripper_width(),
                target_width_m=float(
                    self.arm_spec["gripper"]["max_width_m"]
                    + self.arm_spec["gripper"]["width_per_drive_rad_m"]
                    * self.arm.goal[5]
                ),
                model_drive_position_rad=float(self.data.qpos[self.arm.qpos[5]]),
                model_drive_target_rad=float(self.arm.target[5]),
                left_contact_force_n=float(contacts["left_force_n"]),
                right_contact_force_n=float(contacts["right_force_n"]),
                bilateral_contact=bool(contacts["bilateral_contact"]),
            ),
            tcp=TCPState(
                position_m=tcp_position.tolist(),
                orientation_xyzw=tcp_quat,
                position_error_m=position_error,
                orientation_error_rad=orientation_error,
                held=held,
            ),
            sensors=list(self._sensor_states.values()),
            plan=plan_preview,
            active_session_id=self._owners["base"] or self._owners["arm"],
            active_operator_id=self._owner_operator["base"] or self._owner_operator["arm"],
            active_operator_kind=self._owner_kind["base"] or self._owner_kind["arm"],
            base_owner_session_id=self._owners["base"],
            arm_owner_session_id=self._owners["arm"],
            safety_latched=self._stop_latched,
            contact_summary={
                "unexpected_contact": bool(self._last_unexpected),
                "unexpected": self._last_unexpected,
                "bilateral_grasp": bool(contacts["bilateral_contact"]),
                "left_force_n": float(contacts["left_force_n"]),
                "right_force_n": float(contacts["right_force_n"]),
            },
            diagnostics={
                "mode": "GT_DEBUG",
                "interface_profile": "g2a-1.0",
                "observation_profile_id": self.observation_profile["profile_id"],
                "observation_profile_version": self.observation_profile["profile_version"],
                "observation_profile_sha256": self.observation_profile_sha256,
                "transform_version": "g2a-transform-1",
                "rgbd_schedule": {
                    "period_s": self._rgbd_period_s,
                    "schedule_index": self._rgbd_schedule_index,
                    "skipped_slots": self._rgbd_skipped_slots,
                    "skip_reasons": dict(self._rgbd_skip_reasons),
                    "last_capture": self._last_capture_metric,
                },
                "planning_last": self._last_planning_metric,
                "model_id": self.model_id,
                "model_sha256": self.model_sha256,
                "scene_id": self.scene_id,
                "sensor_error": self._sensor_error,
                "object_pose_world_sim": object_diagnostic,
                "navigation": self.navigation_status(),
                "camera_body": "depth_cam_link",
                "camera_branch_root": "link4",
            },
        )


def muJoCo_mat_to_xyzw(rotation: np.ndarray) -> list[float]:
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, np.asarray(rotation, dtype=float).reshape(9))
    return quat_wxyz_to_xyzw(quat)


def transform_from_world(position, rotation_flat) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = np.asarray(rotation_flat, dtype=float).reshape(3, 3)
    transform[:3, 3] = np.asarray(position, dtype=float)
    return transform


def _rotation_from_quaternion_xyzw(values) -> np.ndarray:
    x, y, z, w = (float(value) for value in values)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("quaternion must be finite and nonzero")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _is_wheel(name: str) -> bool:
    return name.startswith(("front_left_", "rear_left_", "front_right_", "rear_right_"))
