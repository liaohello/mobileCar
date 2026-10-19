"""Contract checks for the G2B causal-alignment fixture."""

from __future__ import annotations

import json
from pathlib import Path


FIXTURE = Path(__file__).parent / "fixtures" / "causal_input_v1.json"
REQUIRED_REJECTIONS = {
    "future_arrival": "image_not_arrived_by_decision",
    "clock_mismatch": "clock_mismatch",
    "cross_episode": "cross_episode",
    "missing_media": "missing_media",
    "long_gap": "long_gap",
    "unknown_action": "unknown_action",
    "preview_rejected": "preview_or_rejected_is_not_effective_action",
    "world_sim_leak": "forbidden_policy_feature",
    "parent_split_leak": "parent_episode_split_leak",
}


def _payload() -> dict:
    return json.loads(FIXTURE.read_text(encoding="utf-8"))


def test_fixture_has_versioned_profile_and_single_parent_identity() -> None:
    data = _payload()
    assert data["schema_version"] == 1
    assert data["fixture_kind"] == "causal_input_v1"
    assert data["profile"]["grid_rate_hz"] == 5
    assert data["profile"]["grid_origin_s"] == 10.0
    source = data["source"]
    assert source["run_id"] == "run_fixture_g2b"
    assert source["episode_id"] == "episode_fixture_parent"
    assert {item["episode_id"] for item in source["segments"]} == {
        source["episode_id"]
    }
    assert source["segments"][1]["previous_recording_segment_id"] == (
        source["segments"][0]["recording_segment_id"]
    )


def test_fixture_samples_keep_identity_clock_and_causal_bindings() -> None:
    data = _payload()
    source = data["source"]
    sample_groups = (
        data["image_samples"],
        data["feedback_samples"],
        data["fast_samples"],
        data["action_events"],
    )
    for rows in sample_groups:
        assert rows
        for row in rows:
            assert row["run_id"] == source["run_id"]
            assert row["episode_id"] == source["episode_id"]
            assert row["clock_id"] == source["clock_id"] == "backend_monotonic"
            if "capture_backend_monotonic_s" in row:
                assert (
                    row["capture_backend_monotonic_s"]
                    <= row["arrival_backend_monotonic_s"]
                )

    for image in data["image_samples"]:
        assert image["capture_backend_monotonic_s"] <= image[
            "publish_backend_monotonic_s"
        ]
        assert image["publish_backend_monotonic_s"] <= image[
            "arrival_backend_monotonic_s"
        ]
        assert image["arrival_backend_monotonic_s"] <= image[
            "persist_backend_monotonic_s"
        ]
        feedback = next(
            row
            for row in data["feedback_samples"]
            if row["sample_id"] == image["feedback_sample_id"]
        )
        assert feedback["control_tick_id"] == image["control_tick_id"]
        assert abs(
            image["capture_backend_monotonic_s"]
            - feedback["capture_backend_monotonic_s"]
        ) <= data["profile"]["max_image_feedback_skew_s"]


def test_fixture_normal_case_and_declared_action_layer_are_consistent() -> None:
    data = _payload()
    normal = [
        case for case in data["cases"] if case["case_id"] == "normal_causal"
    ]
    assert len(normal) == 1
    assert normal[0]["expected"] == "accept"
    assert normal[0]["expected_reason"] is None
    references = [
        row
        for row in data["action_events"]
        if row["event_type"] == "actuator_reference"
    ]
    assert references
    reference = references[0]
    assert reference["payload"]["base_twist"] == [0.02, 0.0, 0.03]
    assert len(reference["payload"]["arm_model_reference_rad"]) == 6
    assert isinstance(reference["payload"]["gripper_width_target_m"], float)
    assert all(
        event["event_type"] not in {"preview", "rejected"}
        for event in data["action_events"]
        if event.get("event_type") in {"dispatch", "actuator_reference"}
    )


def test_fixture_declares_all_required_negative_cases() -> None:
    data = _payload()
    cases = {case["case_id"]: case for case in data["cases"]}
    for case_id, reason in REQUIRED_REJECTIONS.items():
        assert case_id in cases, f"missing negative case {case_id}"
        assert cases[case_id]["expected"] == "reject"
        assert cases[case_id]["expected_reason"] == reason


def test_fixture_does_not_promote_truth_or_parent_leakage() -> None:
    data = _payload()
    leak = next(
        case for case in data["cases"] if case["case_id"] == "world_sim_leak"
    )
    assert "world_sim.object_pose" in leak["feature_keys"]
    split = next(
        case
        for case in data["cases"]
        if case["case_id"] == "parent_split_leak"
    )
    assert split["source_episode_id"] == data["source"]["episode_id"]
    assert split["splits"] == ["train", "validation"]
    assert len(set(split["segment_ids"])) == 2
