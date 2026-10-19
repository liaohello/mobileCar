import math
from pathlib import Path
from uuid import uuid4

import pytest

from mobilecar.backends.mujoco.backend import MujocoBackend
from mobilecar.config import load_config
from mobilecar.contracts.models import CommandRequest, OperatorKind


class Sequence:
    def __init__(self):
        self.value = 0
        self.session = f"dynamics_{uuid4().hex[:8]}"

    def request(self, backend, command, payload, ttl=2.0):
        self.value += 1
        snapshot = backend.snapshot()
        return CommandRequest(
            run_id=snapshot.run_id,
            backend_generation=snapshot.backend_generation,
            episode_id=snapshot.episode_id,
            session_id=self.session,
            operator_id="dynamics_test",
            operator_kind=OperatorKind.AGENT_UI,
            view_source="third_person_3d",
            command_id=f"dyn_{uuid4().hex}",
            command_seq=self.value,
            command_type=command,
            requested_ttl_s=ttl,
            payload=payload,
        )


class Sink:
    def __init__(self):
        self.events = []
        self.observations = []

    def publish(self, event):
        self.events.append(event)

    def publish_observation(self, observation):
        self.observations.append(observation)

    def recording_gate(self, episode_id):
        return True


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


def twist(backend, sequence, vx, vy, wz):
    request = sequence.request(
        backend,
        "set_base_twist",
        {"frame_id": "base_link", "vx_m_s": vx, "vy_m_s": vy, "wz_rad_s": wz},
    )
    assert backend.submit(request).status == "DISPATCHED"
    ticks(backend, 500)
    zero = sequence.request(backend, "zero_base", {"reason": "test_axis_complete"})
    assert backend.submit(zero).status == "DISPATCHED"
    ticks(backend, 180)
    return backend.snapshot()


def reset(backend, sequence):
    request = sequence.request(backend, "reset", {"confirm": True})
    assert backend.submit(request).status == "APPLIED"
    ticks(backend, 20)
    return backend.snapshot()


def yaw(snapshot):
    x, y, z, w = snapshot.base.orientation_world_sim_xyzw
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def test_four_mecanum_directional_motion_release_and_combined_twist(backend):
    sequence = Sequence()
    forward = twist(backend, sequence, 0.05, 0.0, 0.0)
    assert forward.base.position_world_sim_m[0] > 0.035
    assert abs(forward.base.vx_m_s) <= 0.005
    assert abs(forward.base.vy_m_s) <= 0.005
    assert abs(forward.base.wz_rad_s) <= 0.02

    reset(backend, sequence)
    backward = twist(backend, sequence, -0.05, 0.0, 0.0)
    assert backward.base.position_world_sim_m[0] < -0.025

    reset(backend, sequence)
    left = twist(backend, sequence, 0.0, 0.05, 0.0)
    assert left.base.position_world_sim_m[1] > 0.025

    reset(backend, sequence)
    right = twist(backend, sequence, 0.0, -0.05, 0.0)
    assert right.base.position_world_sim_m[1] < -0.025

    reset(backend, sequence)
    ccw = twist(backend, sequence, 0.0, 0.0, 0.30)
    assert yaw(ccw) > 0.20

    reset(backend, sequence)
    cw = twist(backend, sequence, 0.0, 0.0, -0.30)
    assert yaw(cw) < -0.20

    reset(backend, sequence)
    combined = twist(backend, sequence, 0.03, 0.02, 0.10)
    assert combined.base.position_world_sim_m[0] > 0.015
    assert combined.base.position_world_sim_m[1] > 0.010
    assert yaw(combined) > 0.05
    assert combined.safety_latched is False
    assert combined.contact_summary["unexpected_contact"] is False


def test_five_axis_and_gripper_finite_motion(backend):
    sequence = Sequence()
    with backend.lock:
        before = backend.snapshot()
    target = [0.04, -0.03, 0.035, 0.05, -0.04]
    request = sequence.request(
        backend,
        "set_arm_joint_goal",
        {"joint_names": [f"joint{i}" for i in range(1, 6)], "joint_positions_rad": target},
    )
    assert backend.submit(request).status == "DISPATCHED"
    ticks(backend, 700)
    snapshot = backend.snapshot()
    assert any(
        abs(after - prior) > 0.005
        for after, prior in zip(snapshot.arm.user_joint_positions_rad, before.arm.user_joint_positions_rad)
    )

    open_request = sequence.request(backend, "set_gripper_width_goal", {"width_m": 0.04})
    assert backend.submit(open_request).status == "DISPATCHED"
    ticks(backend, 300)
    opened = backend.snapshot().gripper.width_m
    close_request = sequence.request(backend, "set_gripper_width_goal", {"width_m": 0.018})
    assert backend.submit(close_request).status == "DISPATCHED"
    ticks(backend, 400)
    closed = backend.snapshot().gripper.width_m
    assert opened > closed
    assert closed >= 0.012


def test_software_stop_holds_motion_and_records_raw_envelope_streams(backend):
    sink = Sink()
    backend.set_event_sink(sink)
    sequence = Sequence()

    arm = sequence.request(
        backend,
        "set_arm_joint_goal",
        {
            "joint_names": [f"joint{i}" for i in range(1, 6)],
            "joint_positions_rad": [0.30, -0.25, 0.20, 0.30, -0.20],
        },
    )
    assert backend.submit(arm).status == "DISPATCHED"
    ticks(backend, 35)

    gripper = sequence.request(backend, "set_gripper_width_goal", {"width_m": 0.018})
    assert backend.submit(gripper).status == "DISPATCHED"
    ticks(backend, 25)

    stop = sequence.request(backend, "stop", {"latch": True, "reason": "test_stop"})
    assert backend.submit(stop).status == "DISPATCHED"
    stopped = backend.snapshot()
    assert stopped.safety_latched is True
    assert stopped.arm_owner_session_id is None
    assert max(
        abs(target - measured)
        for target, measured in zip(
            stopped.arm.user_joint_targets_rad,
            stopped.arm.user_joint_positions_rad,
        )
    ) < 0.02
    assert abs(stopped.gripper.target_width_m - stopped.gripper.width_m) < 0.002
    ticks(backend, 120)
    settled = backend.snapshot()
    assert settled.base.stopped is True
    assert abs(settled.gripper.target_width_m - settled.gripper.width_m) < 0.002

    requested = [event for event in sink.events if event.event_type == "requested"]
    assert requested
    raw = requested[-1].payload["request"]
    assert raw["command_type"] == "stop"
    assert raw["session_id"] == sequence.session
    assert raw["operator_id"] == "dynamics_test"
    assert raw["requested_ttl_s"] == 2.0
    dispatch = [event for event in sink.events if event.event_type == "dispatch"]
    assert dispatch and all(event.control_tick_id for event in dispatch)
    kinds = {str(item.sensor_kind) for item in sink.observations}
    assert {"imu", "encoders", "joint_feedback"} <= kinds
    assert any(event.event_type == "oracle_sample" for event in sink.events)

    old_episode = settled.episode_id
    reset_request = sequence.request(backend, "reset", {"confirm": True})
    assert backend.submit(reset_request).status == "APPLIED"
    reset_snapshot = backend.snapshot()
    assert reset_snapshot.episode_id != old_episode
    assert reset_snapshot.safety_latched is False
