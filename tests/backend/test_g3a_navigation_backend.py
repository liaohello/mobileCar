from pathlib import Path
from uuid import uuid4

import pytest

from mobilecar.backends.mujoco.backend import MujocoBackend
from mobilecar.config import load_config
from mobilecar.contracts import (
    CommandRequest,
    NavigationControlMessage,
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
    config = load_config("configs/g3a_server.yaml")
    backend_config = dict(config.backend)
    backend_config["render_rgbd"] = False
    backend_config["realtime"] = False
    instance = MujocoBackend(
        backend_config,
        config.safety,
        config.data["acceptance"],
        tmp_path,
    )
    sink = Sink()
    instance.set_event_sink(sink)
    try:
        yield instance, sink
    finally:
        instance.close()


def ticks(backend, count):
    for _ in range(count):
        with backend.lock:
            backend._control_tick(0.01, 10)
            backend._cached_snapshot = backend._build_snapshot()


def human_request(backend, command_type, payload, session_id="human"):
    snapshot = backend.snapshot()
    return CommandRequest(
        run_id=snapshot.run_id,
        backend_generation=snapshot.backend_generation,
        episode_id=snapshot.episode_id,
        session_id=session_id,
        operator_id="g3a_backend_test",
        operator_kind=OperatorKind.AGENT_UI,
        view_source="g3a_backend_test",
        command_id=f"test_{uuid4().hex}",
        command_seq=1,
        command_type=command_type,
        requested_ttl_s=2.0,
        payload=payload,
    )


def test_diagnostic_scene_nav_capture_and_no_target_oracle(backend):
    instance, sink = backend
    assert instance.target_body_id is None
    assert instance.scene_id == "navigation_diagnostic"
    ticks(instance, 12)
    assert len(sink.navigation) >= 5
    observation = sink.navigation[-1]
    assert observation.profile_id == "g3a-nav-50hz-laser10-v1"
    assert observation.wheels.valid is True
    assert observation.imu.gyro_valid is True
    scan_observation = next(
        item for item in reversed(sink.navigation) if item.scan is not None
    )
    scan = scan_observation.scan
    assert scan is not None
    assert scan.frame_id == "laser_frame"
    assert scan.mounting_body == "lidar_link"
    assert scan.beam_count == 360
    assert scan.time_increment_s == 0.0
    assert sum(scan.status_counts.values()) == 360
    assert scan.status_counts.get("SELF_OCCLUDED", 0) > 0
    assert all(
        value is None
        for value, status in zip(scan.ranges_m, scan.statuses)
        if status != "HIT"
    )
    oracle_events = [
        event for event in sink.events if event.event_type == "oracle_sample"
    ]
    assert oracle_events
    assert oracle_events[-1].payload["oracle_status"] == "NOT_APPLICABLE"
    assert instance.snapshot().tcp.held is False


def test_navigation_base_preserves_arm_and_manual_takeover_revokes_nav(backend):
    instance, sink = backend
    ticks(instance, 11)
    clock = sink.navigation[-1].clock.model_dump(mode="json")
    arm = human_request(
        instance,
        "set_arm_joint_goal",
        {
            "joint_names": [f"joint{index}" for index in range(1, 6)],
            "joint_positions_rad": [0.05, -0.03, 0.02, 0.04, -0.03],
        },
        session_id="arm_owner",
    )
    assert instance.submit(arm).status == "DISPATCHED"
    nav = NavigationControlMessage(
        message_type="cmd_vel",
        run_id=clock["run_id"],
        episode_id=clock["episode_id"],
        backend_generation=clock["backend_generation"],
        navigation_epoch=clock["navigation_epoch"],
        session_id="nav_owner",
        vx_m_s=0.02,
        vy_m_s=0.0,
        wz_rad_s=0.05,
    )
    assert instance.submit_navigation_control(nav).status == "DISPATCHED"
    ticks(instance, 5)
    current = instance.snapshot()
    assert current.arm_owner_session_id == "arm_owner"
    assert current.base_owner_session_id == "nav_owner"

    takeover = human_request(
        instance,
        "zero_base",
        {"reason": "manual_takeover"},
        session_id="human_takeover",
    )
    assert instance.submit(takeover).status == "DISPATCHED"
    rejected = instance.submit_navigation_control(nav)
    assert rejected.status == "REJECTED"
    assert "revoked" in rejected.reason
    ticks(instance, 5)
    assert instance.snapshot().arm_owner_session_id == "arm_owner"


def test_faults_invalidate_measurements_and_reset_increments_epoch(backend):
    instance, sink = backend
    ticks(instance, 11)
    old_epoch = instance.navigation_status()["navigation_epoch"]
    instance.set_navigation_fault("imu", True)
    ticks(instance, 2)
    assert sink.navigation[-1].imu.gyro_valid is False
    assert sink.navigation[-1].imu.angular_velocity_rad_s is None
    instance.set_navigation_fault("imu", False)
    instance.set_navigation_fault("wheel", True)
    ticks(instance, 2)
    assert sink.navigation[-1].wheels.valid is False
    instance.set_navigation_fault("wheel", False)
    instance.set_navigation_fault("scan", True)
    ticks(instance, 10)
    scan = next(
        item.scan for item in reversed(sink.navigation) if item.scan is not None
    )
    assert scan.status_counts == {
        "HIT": 0,
        "NO_RETURN": 0,
        "INVALID": 360,
        "SELF_OCCLUDED": 0,
    }
    instance.set_navigation_fault("scan", False)
    reset = human_request(instance, "reset", {"confirm": True})
    assert instance.submit(reset).status == "APPLIED"
    ticks(instance, 12)
    new_epoch = instance.navigation_status()["navigation_epoch"]
    assert new_epoch == old_epoch + 1
    assert sink.navigation[-1].clock.navigation_epoch == new_epoch
    assert abs(instance.navigation_status()["navigation_cancel_epoch"]) >= 1
