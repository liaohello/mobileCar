from __future__ import annotations

import queue
import sys
import threading
from pathlib import Path
from types import SimpleNamespace

from geometry_msgs.msg import Twist


ROOT = Path(__file__).resolve().parents[2]
for path in (
    ROOT / "ros2_ws/src/mobilecar_goal_supervisor",
    ROOT / "ros2_ws/src/mobilecar_navigation_bridge",
    ROOT / "tools/g3b",
):
    sys.path.insert(0, str(path))

from mobilecar_goal_supervisor.g3b_bridge_node import G3BSupervisor  # noqa: E402
from run_single_goal import summarize_map_to_odom  # noqa: E402


IDENTITY = {
    "run_id": "run_fixture",
    "episode_id": "episode_fixture",
    "backend_generation": 1,
    "navigation_epoch": 2,
}


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def fake_supervisor() -> G3BSupervisor:
    value = object.__new__(G3BSupervisor)
    value._supervisor_lock = threading.RLock()
    value._receipt_condition = threading.Condition(value._supervisor_lock)
    value.receipts = {}
    value.active_goal = None
    value.command_gate_open = False
    value.backend_authorized = False
    value.abort_event = threading.Event()
    value.last_raw_command_s = None
    value.raw_command_count = 0
    value.nonzero_raw_command_count = 0
    value.last_raw_command_values = [0.0, 0.0, 0.0]
    value.last_accepted_control_s = None
    value.pending_velocity_request_id = None
    value.pending_velocity_since_s = None
    value.pending_velocity_requests = {}
    value.accepted_velocity_receipt_count = 0
    value.last_rejected_receipt = None
    value.governed_publisher = Publisher()
    value.raw_commands = queue.Queue(maxsize=2)
    value.gate_transitions = []
    value.health_reasons = []
    value.health_status = "INITIALIZING"
    value.localization_health = {
        "status": "READY",
        "stop_required": False,
        "scan_fresh": True,
        "odom_fresh": True,
        "reasons": [],
    }
    value.navigation_health = {
        "status": "HEALTHY",
        "stop_required": False,
        "source_health": {"scan_available": True},
    }
    parameters = {
        "command_lease_ttl_s": 0.35,
        "goal_barrier_s": 0.30,
    }
    value.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    value.current_transport_state = lambda: (dict(IDENTITY), {})
    value.send_g3b_control = lambda _control: True
    return value


def goal() -> dict:
    return {
        "goal_id": "goal_fixture",
        "goal_generation": 3,
        "map_id": "map_fixture",
        "session_id": "session_fixture",
        "identity": dict(IDENTITY),
        "gate_opened_s": 0.0,
        "reasons": [],
    }


def test_health_requires_localization_match_not_only_tf():
    value = fake_supervisor()
    value.localization_health["pose_match_fresh"] = False
    value.localization_health["status"] = "STALE"
    value.localization_health["stop_required"] = True
    healthy, reasons = G3BSupervisor._health_state(value)
    assert healthy is False
    assert "localization_stale" in reasons


def test_closed_gate_drops_raw_twist_but_counts_it():
    value = fake_supervisor()
    command = Twist()
    command.linear.x = 0.02
    G3BSupervisor.on_raw_twist(value, command)
    assert value.raw_command_count == 1
    assert value.nonzero_raw_command_count == 1
    assert value.raw_commands.empty()


def test_control_message_uses_captured_goal_identity():
    value = fake_supervisor()
    captured = goal()
    value.current_transport_state = lambda: (
        {**IDENTITY, "episode_id": "new_episode"},
        {},
    )
    control = G3BSupervisor._control_message(
        value, goal=captured, message_type="cmd_vel", vx=0.03
    )
    assert control["episode_id"] == "episode_fixture"
    assert control["session_id"] == "session_fixture"
    assert control["goal_generation"] == 3


def test_identity_change_closes_final_gate_and_zeroes():
    value = fake_supervisor()
    value.active_goal = goal()
    value.command_gate_open = True
    value.backend_authorized = True
    value.current_transport_state = lambda: (
        {**IDENTITY, "navigation_epoch": 3},
        {},
    )
    value.send_g3b_control = lambda _control: (_ for _ in ()).throw(
        AssertionError("stale goal must not send backend command")
    )
    G3BSupervisor.on_command_values(value, [0.04, 0.0, 0.1])
    assert value.command_gate_open is False
    assert value.abort_event.is_set()
    assert len(value.governed_publisher.messages) == 1
    assert value.governed_publisher.messages[0].linear.x == 0.0


def test_rejected_backend_receipt_closes_gate_and_records_reason():
    value = fake_supervisor()
    value.active_goal = goal()
    value.command_gate_open = True
    value.backend_authorized = True
    value.on_control_receipt = G3BSupervisor.on_control_receipt.__get__(value)
    value.on_control_receipt(
        {
            "request_id": "req_rejected",
            "accepted": False,
            "reason": "lease conflict",
            "backend_status": "REJECTED",
        }
    )
    assert value.command_gate_open is False
    assert value.last_rejected_receipt["reason"] == "lease conflict"
    assert value.governed_publisher.messages[-1].linear.x == 0.0


def test_nonfinite_raw_velocity_is_never_governed():
    value = fake_supervisor()
    value.active_goal = goal()
    value.command_gate_open = True
    value.backend_authorized = True
    command = Twist()
    command.linear.x = float("nan")
    G3BSupervisor.on_raw_twist(value, command)
    assert value.command_gate_open is False
    assert value.abort_event.is_set()
    assert value.raw_commands.empty()


def test_identical_map_tf_repeats_are_not_conflicts():
    record = {
        "sim_stamp_s": 10.0,
        "monotonic_s": 100.0,
        "translation_xyz": [0.1, 0.2, 0.0],
        "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
    }
    summary = summarize_map_to_odom(
        [
            record,
            {**record, "monotonic_s": 100.1},
            {
                **record,
                "sim_stamp_s": 10.1,
                "monotonic_s": 100.2,
            },
        ]
    )
    assert summary["duplicate_stamp_count"] == 1
    assert summary["identical_repeat_count"] == 1
    assert summary["conflicting_stamp_count"] == 0


def test_same_stamp_with_different_transform_is_a_conflict():
    record = {
        "sim_stamp_s": 10.0,
        "monotonic_s": 100.0,
        "translation_xyz": [0.1, 0.2, 0.0],
        "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
    }
    summary = summarize_map_to_odom(
        [
            record,
            {
                **record,
                "monotonic_s": 100.1,
                "translation_xyz": [0.2, 0.3, 0.0],
            },
        ]
    )
    assert summary["conflicting_stamp_count"] == 1


def test_same_stamp_numerical_update_is_not_a_duplicate_source():
    record = {
        "sim_stamp_s": 10.0,
        "monotonic_s": 100.0,
        "translation_xyz": [0.1, 0.2, 0.0],
        "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
    }
    summary = summarize_map_to_odom(
        [
            record,
            {
                **record,
                "monotonic_s": 100.1,
                "translation_xyz": [0.10000001, 0.2, 0.0],
            },
        ]
    )
    assert summary["conflicting_stamp_count"] == 0
    assert summary["minor_same_stamp_update_count"] == 1
