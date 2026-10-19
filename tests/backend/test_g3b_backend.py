from pathlib import Path
from uuid import uuid4

import pytest

from mobilecar.backends.mujoco.backend import MujocoBackend
from mobilecar.config import load_config
from mobilecar.contracts import (
    CommandRequest,
    G3BControlMessage,
    OperatorKind,
)


ROOT = Path(__file__).resolve().parents[2]


class Sink:
    def __init__(self):
        self.events = []
        self.observations = []
        self.navigation = []

    def publish(self, event):
        self.events.append(event)

    def publish_observation(self, observation):
        self.observations.append(observation)

    def publish_navigation(self, observation):
        self.navigation.append(observation)

    def recording_gate(self, episode_id):
        return True


@pytest.fixture
def backend(tmp_path):
    config = load_config("configs/g3b_server_room_a.yaml")
    backend_config = dict(config.backend)
    backend_config["render_rgbd"] = False
    backend_config["realtime"] = False
    instance = MujocoBackend(
        backend_config,
        config.safety,
        config.data["acceptance"],
        tmp_path,
    )
    instance.set_event_sink(Sink())
    try:
        yield instance
    finally:
        instance.close()


def ticks(instance, count):
    for _ in range(count):
        with instance.lock:
            instance._control_tick(0.01, 10)
            instance._cached_snapshot = instance._build_snapshot()


def control(
    instance, message_type, generation, *, session=None, goal_id=None, **values
):
    snapshot = instance.snapshot()
    payload = {
        "message_type": message_type,
        "request_id": f"req_{uuid4().hex}",
        "goal_id": goal_id or f"goal_{generation}",
        "goal_generation": generation,
        "map_id": "map_room_a",
        "run_id": snapshot.run_id,
        "episode_id": snapshot.episode_id,
        "backend_generation": snapshot.backend_generation,
        "navigation_epoch": instance.navigation_status()["navigation_epoch"],
        "session_id": session or f"nav2_goal_{generation}",
        "lease_ttl_s": 0.5,
        **values,
    }
    return G3BControlMessage.model_validate(payload)


def human(instance, command_type, payload, *, session="human", sequence=1):
    snapshot = instance.snapshot()
    return CommandRequest(
        run_id=snapshot.run_id,
        backend_generation=snapshot.backend_generation,
        episode_id=snapshot.episode_id,
        session_id=session,
        operator_id="g3b_human",
        operator_kind=OperatorKind.AGENT_UI,
        view_source="g3b_test",
        command_id=f"human_{uuid4().hex}",
        command_seq=sequence,
        command_type=command_type,
        requested_ttl_s=2.0,
        payload=payload,
    )


def test_goal_authorization_dispatch_receipt_and_stale_rejection(backend):
    ticks(backend, 2)
    authorize = control(backend, "authorize", 1)
    response = backend.submit_g3b_control(authorize)
    assert response.received is True
    assert response.authorized is True
    assert response.accepted is True
    assert response.backend_status == "APPLIED"
    assert response.dispatch_state == "AUTHORIZED"

    command = control(
        backend,
        "cmd_vel",
        1,
        vx_m_s=0.02,
        vy_m_s=0.01,
        wz_rad_s=0.05,
    )
    dispatched = backend.submit_g3b_control(command)
    assert dispatched.accepted is True
    assert dispatched.backend_status == "DISPATCHED"
    ticks(backend, 5)

    stale = control(
        backend,
        "cmd_vel",
        1,
        session="old_goal_session",
        goal_id="stale_goal",
        vx_m_s=0.02,
    )
    rejected = backend.submit_g3b_control(stale)
    assert rejected.received is True
    assert rejected.accepted is False
    assert rejected.dispatch_state == "REJECTED"
    assert "stale" in rejected.reason or "mismatch" in rejected.reason


def test_manual_nonzero_takeover_preserves_arm_and_new_goal_reauthorizes(backend):
    ticks(backend, 2)
    arm = human(
        backend,
        "set_arm_joint_goal",
        {
            "joint_names": [f"joint{index}" for index in range(1, 6)],
            "joint_positions_rad": [0.05, -0.03, 0.02, 0.04, -0.03],
        },
        session="arm_owner",
    )
    assert backend.submit(arm).status == "DISPATCHED"
    assert backend.submit_g3b_control(
        control(backend, "authorize", 1)
    ).accepted is True
    assert backend.submit_g3b_control(
        control(backend, "cmd_vel", 1, vx_m_s=0.02)
    ).accepted is True
    ticks(backend, 3)
    assert backend.snapshot().arm_owner_session_id == "arm_owner"

    takeover = human(
        backend,
        "set_base_twist",
        {
            "frame_id": "base_link",
            "vx_m_s": 0.01,
            "vy_m_s": 0.0,
            "wz_rad_s": 0.0,
        },
        session="manual_owner",
    )
    assert backend.submit(takeover).status == "DISPATCHED"
    assert backend.snapshot().arm_owner_session_id == "arm_owner"

    old = backend.submit_g3b_control(
        control(backend, "cmd_vel", 1, vx_m_s=0.02)
    )
    assert old.accepted is False
    release = human(
        backend,
        "zero_base",
        {"reason": "manual_takeover_release"},
        session="manual_owner",
        sequence=2,
    )
    assert backend.submit(release).status == "DISPATCHED"
    assert backend.submit_g3b_control(
        control(backend, "authorize", 2, session="nav2_goal_2")
    ).accepted is True
    assert backend.submit_g3b_control(
        control(
            backend,
            "cmd_vel",
            2,
            session="nav2_goal_2",
            vx_m_s=0.02,
        )
    ).accepted is True


def test_late_nav_revocation_does_not_override_manual_takeover(backend):
    ticks(backend, 2)
    first = control(backend, "authorize", 1)
    assert backend.submit_g3b_control(first).accepted is True
    assert backend.submit_g3b_control(
        control(backend, "cmd_vel", 1, vx_m_s=0.02)
    ).accepted is True
    ticks(backend, 3)
    takeover = human(
        backend,
        "set_base_twist",
        {
            "frame_id": "base_link",
            "vx_m_s": 0.01,
            "vy_m_s": 0.0,
            "wz_rad_s": 0.0,
        },
        session="manual_after_cancel",
    )
    assert backend.submit(takeover).status == "DISPATCHED"
    ticks(backend, 2)
    stale_revoke = control(backend, "revoke", 1, session="nav2_goal_1")
    receipt = backend.submit_g3b_control(stale_revoke)
    assert receipt.accepted is False
    ticks(backend, 3)
    snapshot = backend.snapshot()
    assert snapshot.base_owner_session_id == "manual_after_cancel"
    assert snapshot.base.requested_vx_m_s == pytest.approx(0.01)
