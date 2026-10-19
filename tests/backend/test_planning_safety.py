import time
import threading
from pathlib import Path
from uuid import uuid4

import mujoco
import numpy as np
import pytest

from mobilecar.backends.mujoco.backend import MujocoBackend
from mobilecar.config import load_config
from mobilecar.contracts.models import CommandRequest, CommandStatus, OperatorKind


class Sink:
    def __init__(self):
        self.events = []

    def publish(self, event):
        self.events.append(event)

    def publish_observation(self, observation):
        pass

    def recording_gate(self, episode_id):
        return True


class Commands:
    def __init__(self, session="planning"):
        self.session = session
        self.sequence = 0

    def make(self, backend, command, payload, ttl=3.0):
        self.sequence += 1
        snapshot = backend.snapshot()
        return CommandRequest(
            run_id=snapshot.run_id,
            backend_generation=snapshot.backend_generation,
            episode_id=snapshot.episode_id,
            session_id=self.session,
            operator_id=self.session,
            operator_kind=OperatorKind.AGENT_UI,
            view_source="third_person_3d",
            command_id=f"planning_{uuid4().hex}",
            command_seq=self.sequence,
            command_type=command,
            requested_ttl_s=ttl,
            payload=payload,
        )


@pytest.fixture
def backend(tmp_path):
    config = load_config()
    backend_config = dict(config.backend)
    backend_config["render_rgbd"] = False
    backend_config["realtime"] = False
    instance = MujocoBackend(backend_config, config.safety, config.data["acceptance"], tmp_path)
    try:
        yield instance
    finally:
        instance.close()


def ticks(backend, count):
    for _ in range(count):
        with backend.lock:
            backend._control_tick(0.01, 10)
            backend._cached_snapshot = backend._build_snapshot()


def set_base_x(backend, value):
    with backend.lock:
        joint = backend.model.joint("base_free").id
        address = backend.model.jnt_qposadr[joint]
        backend.data.qpos[address] = value
        backend.data.qvel[:] = 0.0
        mujoco.mj_forward(backend.model, backend.data)
        backend._cached_snapshot = backend._build_snapshot()


def test_unreachable_tcp_joint_limit_and_collision_are_rejected(backend):
    commands = Commands()
    unreachable = commands.make(
        backend,
        "preview_tcp_target",
        {"frame_id": "world_sim", "position_m": [0.8, 0.0, 0.2]},
    )
    response = backend.submit(unreachable)
    assert response.status == CommandStatus.REJECTED
    assert "unreachable" in response.reason

    bad_joint = commands.make(
        backend,
        "set_arm_joint_goal",
        {
            "joint_names": [f"joint{i}" for i in range(1, 6)],
            "joint_positions_rad": [3.0, 0.0, 0.0, 0.0, 0.0],
        },
    )
    assert backend.submit(bad_joint).status == CommandStatus.REJECTED

    set_base_x(backend, 0.11)
    above = commands.make(
        backend,
        "preview_tcp_target",
        {"frame_id": "world_sim", "position_m": [0.4, 0.0, 0.2]},
    )
    above_response = backend.submit(above)
    assert above_response.status == CommandStatus.RECEIVED
    assert above_response.plan and above_response.plan.collision_free
    with backend.lock:
        backend.data.qpos[backend.arm.qpos[:5]] = above_response.plan.waypoints_model_rad[-1]
        mujoco.mj_forward(backend.model, backend.data)
        backend._cached_snapshot = backend._build_snapshot()
    colliding = commands.make(
        backend,
        "preview_tcp_target",
        {"frame_id": "world_sim", "position_m": [0.4, 0.0, 0.12]},
    )
    collision_response = backend.submit(colliding)
    assert collision_response.status == CommandStatus.RECEIVED
    assert collision_response.plan is not None
    assert collision_response.plan.collision_free is False
    assert collision_response.plan.collision_details


def test_cancel_takeover_and_expired_plan(backend):
    sink = Sink()
    backend.set_event_sink(sink)
    commands = Commands("owner_a")
    preview = commands.make(
        backend,
        "preview_tcp_target",
        {"frame_id": "world_sim", "position_m": [0.18, 0.0, 0.32]},
    )
    preview_response = backend.submit(preview)
    assert preview_response.status == CommandStatus.RECEIVED
    assert preview_response.plan and preview_response.plan.collision_free
    execute = commands.make(
        backend,
        "execute_tcp_target",
        {
            "frame_id": "world_sim",
            "position_m": [0.18, 0.0, 0.32],
            "plan_id": preview_response.plan.plan_id,
        },
    )
    assert backend.submit(execute).status == CommandStatus.DISPATCHED
    ticks(backend, 12)
    cancel = commands.make(backend, "cancel", {"reason": "test_cancel"})
    assert backend.submit(cancel).status == CommandStatus.CANCELLED
    assert backend.snapshot().safety_latched is False
    assert any(event.event_type == "cancelled" for event in sink.events)

    takeover_commands = Commands("owner_b")
    preview = takeover_commands.make(
        backend,
        "preview_tcp_target",
        {"frame_id": "world_sim", "position_m": [0.18, 0.0, 0.32]},
    )
    preview_response = backend.submit(preview)
    execute = takeover_commands.make(
        backend,
        "execute_tcp_target",
        {
            "frame_id": "world_sim",
            "position_m": [0.18, 0.0, 0.32],
            "plan_id": preview_response.plan.plan_id,
        },
    )
    assert backend.submit(execute).status == CommandStatus.DISPATCHED
    ticks(backend, 10)
    manual = Commands("manual_driver")
    takeover = manual.make(
        backend,
        "set_base_twist",
        {"frame_id": "base_link", "vx_m_s": 0.01, "vy_m_s": 0.0, "wz_rad_s": 0.0},
    )
    assert backend.submit(takeover).status == CommandStatus.DISPATCHED
    assert backend.snapshot().arm_owner_session_id is None
    assert any(
        event.event_type == "cancelled"
        and event.payload.get("reason") == "base_manual_takeover"
        for event in sink.events
    )

    expired_commands = Commands("expired")
    expired_preview = expired_commands.make(
        backend,
        "preview_tcp_target",
        {"frame_id": "world_sim", "position_m": [0.18, 0.0, 0.32]},
        ttl=0.01,
    )
    expired_response = backend.submit(expired_preview)
    plan_id = expired_response.plan.plan_id
    with backend.lock:
        backend._previews[plan_id].expires_monotonic_s = time.monotonic() - 1.0
    execute_expired = expired_commands.make(
        backend,
        "execute_tcp_target",
        {
            "frame_id": "world_sim",
            "position_m": [0.18, 0.0, 0.32],
            "plan_id": plan_id,
        },
    )
    assert backend.submit(execute_expired).status == CommandStatus.EXPIRED_BEFORE_DISPATCH


def test_preview_plans_outside_lock_and_discards_changed_state(backend):
    entered = threading.Event()
    release = threading.Event()
    original = backend._make_plan_from_input

    def slow_plan(request, planning, **kwargs):
        entered.set()
        assert release.wait(timeout=5.0)
        return original(request, planning, **kwargs)

    backend._make_plan_from_input = slow_plan
    commands = Commands("offlock")
    request = commands.make(
        backend,
        "preview_tcp_target",
        {"frame_id": "world_sim", "position_m": [0.18, 0.0, 0.32]},
    )
    result = {}

    def submit():
        result["response"] = backend.submit(request)

    thread = threading.Thread(target=submit)
    thread.start()
    assert entered.wait(timeout=5.0)
    lock_started = time.monotonic()
    with backend.lock:
        backend._control_tick(0.01, 10)
    assert time.monotonic() - lock_started < 0.2
    with backend.lock:
        base_joint = backend.model.joint("base_free").id
        address = backend.model.jnt_qposadr[base_joint]
        backend.data.qpos[address] += 0.02
        backend.data.qvel[:] = 0.0
        mujoco.mj_forward(backend.model, backend.data)
    release.set()
    thread.join(timeout=10.0)
    assert not thread.is_alive()
    response = result["response"]
    assert response.status == CommandStatus.REJECTED
    assert "base state changed" in response.reason
    planning = backend.snapshot().diagnostics["planning_last"]
    assert planning["stale"] is True
    assert planning["lock_wait_ms"] >= 0.0
    assert planning["copy_ms"] >= 0.0
