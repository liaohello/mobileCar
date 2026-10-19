from __future__ import annotations

import base64
import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from mobilecar.contracts import (
    EpisodeStartRequest,
    EventRecord,
    RobotSnapshot,
    SensorKind,
    SensorObservation,
)
from mobilecar.data import Recorder, ReplayReader, validate_episode


ROOT = Path(__file__).resolve().parents[2]


def _snapshot() -> RobotSnapshot:
    return RobotSnapshot.model_validate_json(
        (ROOT / "configs/examples/snapshot_fixture.json").read_text(encoding="utf-8")
    )


def _wall() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _event(snapshot: RobotSnapshot, event_type: str, command_id: str | None = None, **payload) -> EventRecord:
    return EventRecord(
        run_id=snapshot.run_id,
        episode_id=snapshot.episode_id,
        event_type=event_type,
        wall_utc=_wall(),
        backend_monotonic_s=snapshot.backend_monotonic_s,
        sim_time_s=snapshot.sim_time_s,
        control_tick_id=snapshot.control_tick_id,
        command_id=command_id,
        payload=payload,
    )


def _start(recorder: Recorder, snapshot: RobotSnapshot):
    return recorder.start(
        EpisodeStartRequest(
            operator_id="agent_fixture",
            operator_kind="agent_ui",
            view_source="third_person_3d",
            require_recording=True,
        ),
        snapshot,
    )


def _lifecycle_events(recorder: Recorder, snapshot: RobotSnapshot) -> None:
    command_id = "fixture_command"
    for event_type in ("requested", "approved", "dispatch", "actuator_reference", "feedback"):
        recorder.publish(
            _event(
                snapshot,
                event_type,
                command_id=command_id,
                action={"base": {"vx_m_s": 0.03}},
            )
        )


def test_nonrecording_observations_ignore_and_numpy_payload_is_json(tmp_path):
    snapshot = _snapshot()
    observation = SensorObservation(
        run_id=snapshot.run_id,
        episode_id=snapshot.episode_id,
        stream_id="imu",
        sample_id="inactive_sample",
        sensor_kind=SensorKind.IMU,
        measurement_time_s=1.0,
        arrival_time_s=1.01,
        clock_id="backend_monotonic",
        frame_id="imu_link",
        source="fixture",
        quality="valid",
        value={"acceleration": np.array([0.0, 0.0, 9.81])},
    )
    recorder = Recorder(root=tmp_path, queue_capacity=64)
    recorder.publish_observation(observation)
    recorder.publish(_event(snapshot, "debug_before_recording", array=np.arange(3)))
    assert recorder.status().status == "IDLE"

    assert _start(recorder, snapshot).accepted
    recorder.publish(_event(snapshot, "feedback", state=np.arange(6, dtype=float)))
    recorder.publish_observation(
        SensorObservation(
            **{
                **observation.model_dump(mode="python"),
                "sample_id": "active_sample",
                "metadata": {"matrix": np.arange(4, dtype=float).reshape(2, 2)},
            }
        )
    )
    stopped = recorder.stop(final_snapshot=snapshot)
    assert stopped.accepted, stopped.reason
    recorder.close()
    episode_dir = tmp_path / snapshot.run_id / "episodes" / snapshot.episode_id
    assert (episode_dir / "events.jsonl").is_file()
    assert "active_sample" in (episode_dir / "observations/imu.jsonl").read_text(encoding="utf-8")


def test_complete_episode_records_assets_and_replays_read_only(tmp_path):
    snapshot = _snapshot()
    recorder = Recorder(root=tmp_path, queue_capacity=64)
    started = _start(recorder, snapshot)
    assert started.accepted
    assert recorder.recording_gate(snapshot.episode_id)
    _lifecycle_events(recorder, snapshot)
    for index, payload in enumerate((b"\x89PNG-fixture-1", b"\x89PNG-fixture-2")):
        transform_metadata = {
            "width": 2,
            "height": 2,
            "T_base_link4": [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
            "T_link4_camera": [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
            "T_base_camera": [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
        }
        observation = SensorObservation(
            run_id=snapshot.run_id,
            episode_id=snapshot.episode_id,
            stream_id="rgbd",
            sample_id=f"frame_{index}",
            sensor_kind=SensorKind.RGB,
            measurement_time_s=10.0 + index * 0.2,
            arrival_time_s=10.01 + index * 0.2,
            clock_id="backend_monotonic",
            frame_id="rgbd_optical",
            source="fixture_camera",
            quality="valid",
            asset_uri=f"assets/rgb/frame_{index}.png",
            metadata=transform_metadata,
        )
        recorder.publish_observation(observation, asset_data=payload)
        recorder.publish_observation(
            SensorObservation(
                run_id=snapshot.run_id,
                episode_id=snapshot.episode_id,
                stream_id="rgbd",
                sample_id=f"frame_{index}",
                sensor_kind=SensorKind.DEPTH,
                measurement_time_s=10.0 + index * 0.2,
                arrival_time_s=10.01 + index * 0.2,
                clock_id="backend_monotonic",
                frame_id="rgbd_optical",
                source="fixture_camera",
                quality="valid",
                asset_uri=f"assets/depth/frame_{index}.npy",
                valid_mask_uri=f"assets/masks/frame_{index}.png",
                metadata={
                    **transform_metadata,
                    "depth_unit": "m",
                    "depth_convention": "optical_axis_z_metres_invalid_nan",
                    "valid_mask_data_b64": base64.b64encode(b"\x89PNG-mask").decode(),
                },
            ),
            asset_data=b"depth-fixture",
        )
    stopped = recorder.stop(reason="OPERATOR_FINISHED", final_snapshot=snapshot)
    assert stopped.accepted, stopped.reason
    assert stopped.recording_status == "COMPLETE"

    episode_dir = tmp_path / snapshot.run_id / "episodes" / snapshot.episode_id
    assert (episode_dir / "episode_complete.json").is_file()
    assert not (episode_dir / "maps").exists()
    assert (episode_dir / "assets/rgb/frame_0.png").read_bytes() == b"\x89PNG-fixture-1"
    report = validate_episode(tmp_path, snapshot.episode_id)
    assert report.passed, report.errors
    assert report.audit_status == "COMPLETE"
    assert report.counts["events"] >= 8
    assert report.counts["observations"] == 4
    assert report.counts["transforms"] == 2
    assert report.actual_rates_hz["rgbd"] == 5.0

    replay = ReplayReader(tmp_path)
    index = replay.list_episodes()
    assert index["episodes"][0]["episode_id"] == snapshot.episode_id
    detail = replay.read_episode(snapshot.episode_id)
    assert detail["read_only"] is True
    assert detail["episode"]["operator_kind"] == "agent_ui"
    assert detail["events"][0]["event_type"] == "recording_start"
    assert detail["latest_rgb_uri"] == "assets/rgb/frame_1.png"
    recorder.close()


def test_missing_asset_and_duplicate_frame_fail_validation(tmp_path):
    snapshot = _snapshot()
    recorder = Recorder(root=tmp_path, queue_capacity=64)
    assert _start(recorder, snapshot).accepted
    for index, uri in enumerate(("assets/rgb/missing.png", "assets/rgb/missing.png")):
        recorder.publish_observation(
            SensorObservation(
                run_id=snapshot.run_id,
                episode_id=snapshot.episode_id,
                stream_id="rgbd",
                sample_id=f"missing_{index}",
                sensor_kind=SensorKind.RGB,
                measurement_time_s=10.0 + index * 0.2,
                arrival_time_s=10.01 + index * 0.2,
                clock_id="backend_monotonic",
                frame_id="rgbd_optical",
                source="fixture_camera",
                quality="valid",
                asset_uri=uri,
            )
        )
    assert recorder.stop(final_snapshot=snapshot).accepted
    report = validate_episode(tmp_path, snapshot.episode_id)
    assert not report.passed
    assert any("asset missing" in error for error in report.errors)
    recorder.close()


def test_recording_fault_and_truncated_tail_are_incomplete(tmp_path):
    snapshot = _snapshot()
    recorder = Recorder(root=tmp_path, queue_capacity=64, fault_after_events=1)
    assert _start(recorder, snapshot).accepted
    _lifecycle_events(recorder, snapshot)
    stopped = recorder.stop(final_snapshot=snapshot)
    assert not stopped.accepted
    assert stopped.recording_status == "FAILED"
    report = validate_episode(tmp_path, snapshot.episode_id)
    assert not report.passed
    assert report.audit_status == "INCOMPLETE"
    assert any("INCOMPLETE" in item or "terminal" in item for item in report.warnings + report.errors)
    recorder.close()

    # A missing completion tail is an incomplete record, not a complete one.
    complete = tmp_path / snapshot.run_id / "episodes" / snapshot.episode_id / "episode_complete.json"
    complete.unlink()
    report = validate_episode(tmp_path, snapshot.episode_id)
    assert not report.passed
    assert any("episode_complete.json is missing" in error for error in report.errors)


def test_reopened_segment_has_new_identity(tmp_path):
    snapshot = _snapshot()
    recorder = Recorder(root=tmp_path, queue_capacity=64)
    first = _start(recorder, snapshot)
    assert first.accepted
    assert recorder.stop(final_snapshot=snapshot).accepted
    second = _start(recorder, snapshot)
    assert second.accepted
    assert second.recording_segment_id != first.recording_segment_id
    assert recorder.stop(final_snapshot=snapshot).accepted
    episode = json.loads(
        (
            tmp_path
            / snapshot.run_id
            / "episodes"
            / snapshot.episode_id
            / "episode.json"
        ).read_text(encoding="utf-8")
    )
    assert len(episode["segments"]) == 2
    assert episode["segments"][1]["previous_recording_segment_id"] == first.recording_segment_id
    recorder.close()


def test_close_drains_active_episode_and_truth_leak_fails_audit(tmp_path):
    snapshot = _snapshot()
    recorder = Recorder(root=tmp_path, queue_capacity=64)
    assert _start(recorder, snapshot).accepted
    recorder.publish(
        _event(
            snapshot,
            "sensor_observation",
            world_sim_truth={"cube_position_m": [0.4, 0.0, 0.112]},
        )
    )
    recorder.close()
    recorder._worker.join(timeout=2)
    report = validate_episode(tmp_path, snapshot.episode_id)
    assert not report.passed
    assert any("world_sim truth" in error for error in report.errors)


def test_backend_incoming_asset_is_copied_into_episode(tmp_path):
    snapshot = _snapshot()
    recorder = Recorder(root=tmp_path, queue_capacity=64)
    assert _start(recorder, snapshot).accepted
    incoming = tmp_path / snapshot.run_id / "_incoming" / snapshot.run_id / "rgbd"
    incoming.mkdir(parents=True)
    source = incoming / "camera_rgb.png"
    source.write_bytes(b"\x89PNG-backend-owned")
    depth_source = incoming / "camera_depth.npy"
    depth_source.write_bytes(b"depth-backend-owned")
    relative = source.relative_to(tmp_path).as_posix()
    depth_relative = depth_source.relative_to(tmp_path).as_posix()
    recorder.publish_observation(
        SensorObservation(
            run_id=snapshot.run_id,
            episode_id=snapshot.episode_id,
            stream_id="rgbd",
            sample_id="camera_sample",
            sensor_kind=SensorKind.RGB,
            measurement_time_s=10.0,
            arrival_time_s=10.01,
            clock_id="backend_monotonic",
            frame_id="rgbd_optical",
            source="mujoco_renderer",
            quality="simulated_nominal",
            asset_uri=relative,
            metadata={
                "rgb_path": relative,
                "T_base_link4": [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
                "T_link4_camera": [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
                "T_base_camera": [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
            },
        )
    )
    recorder.publish_observation(
        SensorObservation(
            run_id=snapshot.run_id,
            episode_id=snapshot.episode_id,
            stream_id="rgbd",
            sample_id="camera_sample",
            sensor_kind=SensorKind.DEPTH,
            measurement_time_s=10.0,
            arrival_time_s=10.01,
            clock_id="backend_monotonic",
            frame_id="rgbd_optical",
            source="mujoco_renderer",
            quality="simulated_nominal",
            asset_uri=depth_relative,
            valid_mask_uri=relative,
            metadata={
                "depth_path": depth_relative,
                "depth_unit": "m",
                "T_base_link4": [[1.0, 0.0, 0.0, 0.0]],
                "T_link4_camera": [[1.0, 0.0, 0.0, 0.0]],
                "T_base_camera": [[1.0, 0.0, 0.0, 0.0]],
            },
        )
    )
    assert recorder.stop(final_snapshot=snapshot).accepted
    copied = tmp_path / snapshot.run_id / "episodes" / snapshot.episode_id / "assets/rgb/camera_rgb.png"
    assert copied.read_bytes() == source.read_bytes()
    report = validate_episode(tmp_path, snapshot.episode_id)
    assert report.passed, report.errors
    recorder.close()


def test_depth_must_declare_metric_convention_and_valid_mask(tmp_path):
    snapshot = _snapshot()
    recorder = Recorder(root=tmp_path, queue_capacity=64)
    assert _start(recorder, snapshot).accepted
    recorder.publish_observation(
        SensorObservation(
            run_id=snapshot.run_id,
            episode_id=snapshot.episode_id,
            stream_id="rgbd",
            sample_id="depth_sample",
            sensor_kind=SensorKind.DEPTH,
            measurement_time_s=10.0,
            arrival_time_s=10.01,
            clock_id="backend_monotonic",
            frame_id="rgbd_optical",
            source="mujoco_renderer",
            quality="simulated_nominal",
            asset_uri="assets/depth/depth.npy",
            metadata={
                "depth_unit": "raw",
                "T_base_link4": [[1.0, 0.0, 0.0, 0.0]],
                "T_link4_camera": [[1.0, 0.0, 0.0, 0.0]],
                "T_base_camera": [[1.0, 0.0, 0.0, 0.0]],
            },
        ),
        asset_data=b"depth-fixture",
    )
    assert recorder.stop(final_snapshot=snapshot).accepted
    report = validate_episode(tmp_path, snapshot.episode_id)
    assert not report.passed
    assert any("not explicitly metric" in item for item in report.errors)
    assert any("valid mask" in item for item in report.errors)
    recorder.close()
