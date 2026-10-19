from __future__ import annotations

from types import SimpleNamespace

from mobilecar.data.export.models import ImageSample, ObservationSample
from mobilecar.data.g4a2_export import extract_g4a2_rows


class _Reader:
    pass


def _reader(*, teacher_plan_id=None):
    reader = _Reader()
    reader.episode_dir = None
    reader.images = [
        ImageSample(
            sample_id="rgbd_1",
            stream_id="rgbd",
            recording_segment_id="seg",
            capture_sim_time_s=1.0,
            capture_backend_monotonic_s=10.0,
            persist_backend_monotonic_s=10.1,
            publish_backend_monotonic_s=10.0,
            arrival_backend_monotonic_s=10.1,
            control_tick_id="tick_1",
            feedback_sample_id="feedback_1",
            rgb_uri="rgb.png",
            depth_uri="depth.npy",
            valid_mask_uri="mask.png",
            width=640,
            height=480,
        )
    ]
    reader.feedback = [
        ObservationSample(
            sample_id="feedback_1",
            sensor_kind="joint_feedback",
            stream_id="joint_feedback",
            recording_segment_id="seg",
            control_tick_id="tick_1",
            capture_sim_time_s=1.0,
            capture_backend_monotonic_s=10.0,
            arrival_backend_monotonic_s=10.1,
            value={"model_joint_positions_rad": [0, 0, 0, -1.5707963267948966, 0], "gripper_width_m": 0.025},
        )
    ]
    request = {
        "source": "human_teleop",
        "teacher_plan_id": teacher_plan_id,
        "joint_order": ["joint1", "joint2", "joint3", "joint4", "joint5"],
        "units": ["rad", "rad", "rad", "rad", "rad", "m"],
        "arm_user_joint_positions_rad": [0, 0, 0, 0, 0],
        "gripper_width_m": 0.025,
        "action_frame_id": "frame_1",
        "action_seq": 1,
    }
    reader.events = [
        {
            "event_type": "human_intent",
            "payload": {
                "human_intent_id": "intent_1",
                "assist_mode": "manual",
                "control_generator": "human_sixdim_sender",
            },
        },
        {
            "event_type": "g4a1_action_approved",
            "backend_monotonic_s": 10.2,
            "payload": {"request": request},
        },
        {
            "event_type": "g4a2_action_linked",
            "backend_monotonic_s": 10.2,
            "payload": {
                "action_frame_id": "frame_1",
                "human_intent_id": "intent_1",
                "display_sample_id": "rgbd_1",
                "receipt": {"control_tick_id": "tick_2"},
            },
        },
    ]
    return reader


def test_human_rows_keep_display_and_target_binding():
    rows, rejected = extract_g4a2_rows(_reader())
    assert not rejected
    assert len(rows) == 1
    assert rows[0].human_intent_id == "intent_1"
    assert rows[0].display_sample_id == "rgbd_1"
    assert rows[0].control_tick_id == "tick_2"


def test_hidden_teacher_plan_is_rejected():
    rows, rejected = extract_g4a2_rows(_reader(teacher_plan_id="hidden"))
    assert not rows
    assert any(item["reason"].endswith("hidden_teacher_plan") for item in rejected)
