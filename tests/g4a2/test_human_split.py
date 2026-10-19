from __future__ import annotations

import json
from pathlib import Path

from mobilecar.data.g4a2_qualification import validate_collection_manifest


ROOT = Path(__file__).resolve().parents[2]


def test_fixture_split_is_valid():
    value = json.loads((ROOT / "tests/g4a2/fixtures/collection_manifest_v1.json").read_text())
    assert validate_collection_manifest(value["entries"])["passed"] is True


def test_family_and_parent_split_leaks_rejected():
    entries = [
        {"episode_id": "a", "parent_episode_id": "p", "family_id": "f", "split": "train", "operator_kind": "human_teleop", "task_result": "success", "retained": True},
        {"episode_id": "b", "parent_episode_id": "p2", "family_id": "f", "split": "test", "operator_kind": "human_teleop", "task_result": "success", "retained": True},
    ]
    result = validate_collection_manifest(entries)
    assert result["passed"] is False
    assert "family_split_leak" in result["reasons"]


def test_dev_pilot_can_use_dev_split():
    entries = [
        {"episode_id": "a", "parent_episode_id": "p", "family_id": "f", "split": "dev", "operator_kind": "human_teleop", "task_result": "success", "retained": True},
        {"episode_id": "b", "parent_episode_id": "p2", "family_id": "f2", "split": "dev", "operator_kind": "human_teleop", "task_result": "cancelled", "retained": True},
    ]
    assert validate_collection_manifest(entries)["passed"] is True
