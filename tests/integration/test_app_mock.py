import copy
from pathlib import Path

import yaml
from fastapi.testclient import TestClient

from mobilecar.app import create_app
from mobilecar.config import load_config


def _profile(tmp_path: Path) -> Path:
    base = load_config()
    data = copy.deepcopy(base.data)
    data["backend"]["kind"] = "mock"
    data["recording"]["required_for_motion"] = True
    data["recording"]["root"] = str(tmp_path / "runs")
    data["quality"]["report_root"] = str(tmp_path / "reports")
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    return path


def _command(snapshot, command_type, sequence, payload):
    return {
        "contract_version": "mobilecar-1.0",
        "run_id": snapshot["run_id"],
        "backend_generation": snapshot["backend_generation"],
        "episode_id": snapshot["episode_id"],
        "session_id": "integration_session",
        "operator_id": "integration_agent",
        "operator_kind": "agent_ui",
        "view_source": "third_person_3d",
        "command_id": f"integration_command_{sequence}",
        "command_seq": sequence,
        "command_type": command_type,
        "requested_ttl_s": 0.5,
        "payload": payload,
    }


def test_app_records_mock_episode_and_replay_api_is_read_only(tmp_path):
    app = create_app(_profile(tmp_path))
    with TestClient(app) as client:
        assert client.get("/api/v1/health").status_code == 200
        capabilities = client.get("/api/v1/capabilities").json()
        scene = client.get("/api/v1/scene").json()
        assert capabilities["model_sha256"] == scene["model_sha256"]
        initial = client.get("/api/v1/snapshot").json()

        before_start = client.post(
            "/api/v1/commands",
            json=_command(
                initial,
                "set_base_twist",
                1,
                {"frame_id": "base_link", "vx_m_s": 0.01, "vy_m_s": 0.0, "wz_rad_s": 0.0},
            ),
        ).json()
        assert before_start["status"] == "REJECTED"

        started = client.post(
            "/api/v1/episodes/start",
            json={
                "task_language": "integration mock recording",
                "operator_id": "integration_agent",
                "operator_kind": "agent_ui",
                "view_source": "third_person_3d",
                "map_mode": "unavailable",
                "require_recording": True,
            },
        ).json()
        assert started["accepted"] is True
        snapshot = client.get("/api/v1/snapshot").json()
        assert snapshot["recording"]["command_capture_ready"] is True

        response = client.post(
            "/api/v1/commands",
            json=_command(
                snapshot,
                "set_base_twist",
                2,
                {"frame_id": "base_link", "vx_m_s": 0.01, "vy_m_s": 0.0, "wz_rad_s": 0.0},
            ),
        ).json()
        assert response["status"] == "DISPATCHED"
        renewed = client.post(
            "/api/v1/lease/renew",
            json={"session_id": "integration_session"},
        ).json()
        assert renewed["accepted"] is True

        stopped = client.post(
            "/api/v1/episodes/stop",
            json={"reason": "INTEGRATION_FINISHED"},
        ).json()
        assert stopped["accepted"] is True
        assert stopped["recording_status"] == "COMPLETE"

        episode_id = stopped["episode_id"]
        detail = client.get(f"/api/v1/episodes/{episode_id}")
        assert detail.status_code == 200
        payload = detail.json()
        assert payload["read_only"] is True
        assert payload["validation"]["audit_status"] == "COMPLETE"
        assert payload["episode"]["operator_kind"] == "agent_ui"
        assert payload["events"]
        requested = [
            event for event in payload["events"] if event["event_type"] == "requested"
        ]
        assert requested
        raw_request = requested[0]["payload"]["request"]
        assert raw_request["session_id"] == "integration_session"
        assert raw_request["operator_kind"] == "agent_ui"
        assert raw_request["requested_ttl_s"] == 0.5

        validation = client.post(f"/api/v1/episodes/{episode_id}/validate").json()
        assert validation["passed"] is True
        eligibility = client.get(f"/api/v1/episodes/{episode_id}/eligibility").json()
        assert eligibility["source_operator_kind"] == "agent_ui"
        g2a_quality = client.get(f"/api/v1/episodes/{episode_id}/g2a-quality")
        assert g2a_quality.status_code == 200
        quality_payload = g2a_quality.json()
        assert quality_payload["profile_id"] == "g2a-rgbd-5hz-original"
        assert quality_payload["output_scope"] == "external_report_only"
        g2a_eligibility = client.get(
            f"/api/v1/episodes/{episode_id}/g2a-eligibility"
        )
        assert g2a_eligibility.status_code == 200
        assert g2a_eligibility.json()["profile_id"] == "g2a-rgbd-5hz-original"
