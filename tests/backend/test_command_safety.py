import time
from pathlib import Path
from uuid import uuid4

from mobilecar.backends.factory import create_backend
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
        return False


def request(backend, session, seq, command="set_base_twist", payload=None):
    snapshot = backend.snapshot()
    return CommandRequest(
        run_id=snapshot.run_id,
        backend_generation=snapshot.backend_generation,
        episode_id=snapshot.episode_id,
        session_id=session,
        operator_id=session,
        operator_kind=OperatorKind.AGENT_UI,
        view_source="third_person_3d",
        command_id=f"{session}_{seq}_{uuid4().hex[:6]}",
        command_seq=seq,
        command_type=command,
        requested_ttl_s=0.5,
        payload=payload
        or {"frame_id": "base_link", "vx_m_s": 0.01, "vy_m_s": 0.0, "wz_rad_s": 0.0},
    )


def test_command_identity_sequence_ownership_and_stop_latch(tmp_path):
    config = load_config()
    backend = create_backend(
        {"kind": "mock", "model_path": config.backend["model_path"], "generation": 1},
        config.safety,
        config.data["acceptance"],
        tmp_path,
    )
    sink = Sink()
    backend.set_event_sink(sink)
    try:
        first = request(backend, "session_a", 1)
        response = backend.submit(first)
        assert response.status == CommandStatus.DISPATCHED
        assert backend.submit(first).command_id == first.command_id

        stale = request(backend, "session_a", 1)
        assert backend.submit(stale).status == CommandStatus.REJECTED

        contended = request(backend, "session_b", 1)
        assert backend.submit(contended).status == CommandStatus.REJECTED

        stop = request(backend, "session_b", 2, "stop", {"latch": True})
        assert backend.submit(stop).status == CommandStatus.DISPATCHED
        assert backend.snapshot().safety_latched
        assert backend.submit(request(backend, "session_a", 2)).status == CommandStatus.REJECTED

        earlier = request(backend, "old_session", 1)
        earlier = earlier.model_copy(update={"episode_id": "wrong_episode"})
        assert backend.submit(earlier).status == CommandStatus.REJECTED
        assert any(event.event_type == "requested" for event in sink.events)
        assert any(event.event_type == "rejected" for event in sink.events)
    finally:
        backend.close()


def test_mujoco_ttl_expires_before_dispatch(tmp_path, monkeypatch):
    from mobilecar.backends.mujoco.backend import MujocoBackend

    config = load_config()
    backend_config = dict(config.backend)
    backend_config["render_rgbd"] = False
    backend_config["realtime"] = False
    backend_config["test_dispatch_delay_s"] = 0.08
    backend = MujocoBackend(
        backend_config,
        config.safety,
        config.data["acceptance"],
        tmp_path,
    )
    try:
        expired = request(backend, "ttl", 1)
        expired = expired.model_copy(update={"requested_ttl_s": 0.01})
        assert backend.submit(expired).status == CommandStatus.EXPIRED_BEFORE_DISPATCH
    finally:
        backend.close()


def test_mujoco_lease_expiry_releases_base_and_is_recorded(tmp_path):
    from mobilecar.backends.mujoco.backend import MujocoBackend

    config = load_config()
    backend_config = dict(config.backend)
    backend_config["render_rgbd"] = False
    backend_config["realtime"] = False
    backend = MujocoBackend(
        backend_config,
        config.safety,
        config.data["acceptance"],
        tmp_path,
    )
    sink = Sink()
    backend.set_event_sink(sink)
    try:
        command = request(backend, "lease_session", 1)
        assert backend.submit(command).status == CommandStatus.DISPATCHED
        assert backend.snapshot().base_owner_session_id == "lease_session"
        with backend.lock:
            backend._lease_deadline["base"] = time.monotonic() - 0.1
            backend._control_tick(0.01, 10)
            backend._cached_snapshot = backend._build_snapshot()
        snapshot = backend.snapshot()
        assert snapshot.base_owner_session_id is None
        assert any(
            event.event_type == "lease_expired"
            and event.payload.get("channel") == "base"
            for event in sink.events
        )
    finally:
        backend.close()


def test_episode_timeout_latches_and_publishes_terminal_reason(tmp_path):
    from mobilecar.backends.mujoco.backend import MujocoBackend

    config = load_config()
    backend_config = dict(config.backend)
    backend_config["render_rgbd"] = False
    backend_config["realtime"] = False
    safety = dict(config.safety)
    safety["episode_timeout_s"] = 0.01
    backend = MujocoBackend(backend_config, safety, config.data["acceptance"], tmp_path)
    sink = Sink()
    backend.set_event_sink(sink)
    try:
        with backend.lock:
            backend.episode_started = time.monotonic() - 1.0
            backend._control_tick(0.01, 10)
            backend._cached_snapshot = backend._build_snapshot()
        assert backend.snapshot().safety_latched is True
        timeout = [event for event in sink.events if event.event_type == "episode_timeout"]
        assert len(timeout) == 1
        assert timeout[0].payload["termination_reason"] == "EPISODE_TIMEOUT"
    finally:
        backend.close()
