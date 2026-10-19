import socket
import time
from pathlib import Path

from mobilecar.contracts.g3b import ControlReceipt, G3BControlMessage
from mobilecar.contracts.navigation import NavigationControlMessage
from mobilecar.ros_ipc import (
    NavigationIpcServer,
    decode_any_control_line,
    decode_control_line,
)
from mobilecar.ros_ipc.server import NavigationIpcStats


def test_control_codec_rejects_multiple_messages():
    first = {
        "schema_version": 1,
        "message_type": "release",
        "source": "ros_navigation_test",
        "run_id": "run",
        "episode_id": "episode",
        "backend_generation": 1,
        "navigation_epoch": 0,
        "session_id": "bridge",
        "lease_ttl_s": 0.2,
    }
    import json

    line = (json.dumps(first) + "\n").encode()
    assert decode_control_line(line).message_type == "release"
    try:
        decode_control_line(line + line)
    except ValueError as exc:
        assert "one newline-terminated" in str(exc)
    else:
        raise AssertionError("multiple lines must be rejected")


def test_server_drops_live_messages_without_client(tmp_path: Path):
    received: list[NavigationControlMessage] = []
    server = NavigationIpcServer(
        tmp_path / "navigation.sock",
        received.append,
        queue_capacity=2,
    )
    server.start()
    try:
        server.publish_heartbeat(
            run_id="run",
            episode_id="episode",
            backend_generation=1,
            navigation_epoch=0,
            backend_monotonic_s=1.0,
        )
        stats: NavigationIpcStats = server.stats()
        assert stats.running is True
        assert stats.connected is False
        assert stats.dropped_outbound == 1
    finally:
        server.close()
    assert not (tmp_path / "navigation.sock").exists()


def test_server_reads_control_and_sends_heartbeat(tmp_path: Path):
    received: list[NavigationControlMessage] = []
    server = NavigationIpcServer(
        tmp_path / "navigation.sock",
        received.append,
        queue_capacity=4,
    )
    server.start()
    client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        client.connect(str(tmp_path / "navigation.sock"))
        deadline = time.monotonic() + 1.0
        while not server.stats().connected and time.monotonic() < deadline:
            time.sleep(0.005)
        server.publish_heartbeat(
            run_id="run",
            episode_id="episode",
            backend_generation=1,
            navigation_epoch=2,
            backend_monotonic_s=3.0,
        )
        payload = (
            b'{"schema_version":1,"message_type":"release",'
            b'"source":"ros_navigation_test","run_id":"run",'
            b'"episode_id":"episode","backend_generation":1,'
            b'"navigation_epoch":2,"session_id":"bridge",'
            b'"lease_ttl_s":0.2}\n'
        )
        client.sendall(payload)
        client.settimeout(1.0)
        line = client.recv(4096)
        assert b'"message_type":"heartbeat"' in line
        deadline = time.monotonic() + 1.0
        while not received and time.monotonic() < deadline:
            time.sleep(0.005)
        assert received[0].navigation_epoch == 2
        assert received[0].message_type == "release"
    finally:
        client.close()
        server.close()


def test_server_returns_g3b_dispatch_receipt(tmp_path: Path):
    def handler(message):
        assert isinstance(message, G3BControlMessage)
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
            authorized=True,
            accepted=True,
            backend_status="APPLIED",
            dispatch_state="AUTHORIZED",
            reason="accepted",
            control_tick_id="tick",
            backend_monotonic_s=10.0,
            sim_time_s=1.0,
        )

    server = NavigationIpcServer(
        tmp_path / "navigation.sock",
        handler,
        queue_capacity=4,
    )
    server.start()
    client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        client.connect(str(tmp_path / "navigation.sock"))
        deadline = time.monotonic() + 1.0
        while not server.stats().connected and time.monotonic() < deadline:
            time.sleep(0.005)
        line = (
            b'{"schema_version":2,"interface_profile":"g3b-1.0",'
            b'"message_type":"authorize","source":"nav2_goal",'
            b'"request_id":"request_1","goal_id":"goal_1",'
            b'"goal_generation":1,"map_id":"map_1","run_id":"run",'
            b'"episode_id":"episode","backend_generation":1,'
            b'"navigation_epoch":0,"session_id":"session_1",'
            b'"lease_ttl_s":0.35}\n'
        )
        decoded = decode_any_control_line(line)
        assert isinstance(decoded, G3BControlMessage)
        client.sendall(line)
        client.settimeout(1.0)
        received = client.recv(8192)
        receipt = ControlReceipt.model_validate_json(received.decode().strip())
        assert receipt.request_id == "request_1"
        assert receipt.accepted is True
        assert receipt.backend_status == "APPLIED"
    finally:
        client.close()
        server.close()
