import json
from pathlib import Path

import pytest
from pydantic import ValidationError

from mobilecar.contracts.g3b import (
    ControlReceipt,
    G3BControlMessage,
    MapPackageManifest,
)


ROOT = Path(__file__).resolve().parents[2]
FIXTURE = ROOT / "tests/g3b/fixtures/map_goal_binding_v1.json"


def test_map_goal_fixture_parses_accepted_bindings():
    payload = json.loads(FIXTURE.read_text(encoding="utf-8"))
    manifest = MapPackageManifest.model_validate(payload["map_manifest"])
    authorize = G3BControlMessage.model_validate(payload["authorize_message"])
    authorize_receipt = ControlReceipt.model_validate(payload["authorize_receipt"])
    command = G3BControlMessage.model_validate(payload["cmd_vel_message"])
    command_receipt = ControlReceipt.model_validate(payload["cmd_vel_receipt"])

    assert manifest.map_id == authorize.map_id == command.map_id
    assert manifest.serialized_pose_graph is True
    assert authorize_receipt.accepted is True
    assert authorize_receipt.backend_status == "APPLIED"
    assert command_receipt.accepted is True
    assert command_receipt.backend_status == "DISPATCHED"
    assert authorize_receipt.request_id == authorize.request_id
    assert command_receipt.request_id == command.request_id


def test_fixture_stale_goal_is_rejected_by_contract():
    payload = json.loads(FIXTURE.read_text(encoding="utf-8"))
    stale = payload["expected_rejection"]["message"]
    with pytest.raises(ValidationError):
        G3BControlMessage.model_validate(stale)
