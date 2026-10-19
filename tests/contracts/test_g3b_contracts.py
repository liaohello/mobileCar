from pathlib import Path

import pytest
import yaml
from pydantic import ValidationError

from mobilecar.contracts.g3b import (
    ControlReceipt,
    G3BControlMessage,
    G3B_INTERFACE_PROFILE,
    G3B_NAVIGATION_PROFILE_ID,
    MapAsset,
    MapInitialPrior,
    MapPackageManifest,
)


ROOT = Path(__file__).resolve().parents[2]
HASH = "0" * 64


def _assets(*, pose_graph: bool = True) -> list[MapAsset]:
    values = [
        MapAsset(
            role="occupancy_yaml",
            relative_path="map.yaml",
            sha256=HASH,
            bytes=100,
        ),
        MapAsset(
            role="occupancy_image",
            relative_path="map.pgm",
            sha256=HASH,
            bytes=200,
        ),
        MapAsset(
            role="map_manifest",
            relative_path="manifest.sha256",
            sha256=HASH,
            bytes=76,
        ),
    ]
    if pose_graph:
        values.extend(
            [
                MapAsset(
                    role="pose_graph",
                    relative_path="posegraph.data",
                    sha256=HASH,
                    bytes=400,
                ),
                MapAsset(
                    role="pose_graph_data",
                    relative_path="posegraph.posegraph",
                    sha256=HASH,
                    bytes=500,
                ),
            ]
        )
    return values


def _map_manifest(*, pose_graph: bool = True) -> MapPackageManifest:
    return MapPackageManifest(
        profile_id=G3B_NAVIGATION_PROFILE_ID,
        profile_version="1.0",
        profile_sha256=HASH,
        map_id="map_room_a",
        scene_id="room_a",
        model_sha256=HASH,
        navigation_profile_sha256=HASH,
        sensor_profile_sha256=HASH,
        scan_config_sha256=HASH,
        resolution_m=0.05,
        origin_xy_yaw=[-1.0, -1.0, 0.0],
        occupied_thresh=0.65,
        free_thresh=0.25,
        created_run_id="run",
        created_episode_id="episode",
        created_navigation_epoch=1,
        serialized_pose_graph=pose_graph,
        initial_prior=MapInitialPrior(
            x_m=0.0,
            y_m=0.0,
            yaw_rad=0.0,
            covariance=[0.0] * 36,
        ),
        assets=_assets(pose_graph=pose_graph),
    )


def test_g3b_profile_is_frozen():
    profile = yaml.safe_load(
        (ROOT / "configs/profiles/g3b_navigation_v1.yaml").read_text(
            encoding="utf-8"
        )
    )
    assert profile["interface_profile"] == G3B_INTERFACE_PROFILE
    assert profile["profile_id"] == G3B_NAVIGATION_PROFILE_ID
    assert profile["mode"]["allowed"] == ["MAPPING", "LOCALIZATION"]
    assert profile["control"]["backend_receipt_required"] is True
    assert profile["localization"]["tf_alone_is_sufficient"] is False
    assert profile["acceptance"]["formal_cases"] == 10
    assert profile["acceptance"]["goal_position_tolerance_m"] == 0.10
    assert profile["acceptance"]["goal_yaw_tolerance_deg"] == 10.0
    assert profile["nav2"]["max_vel_y"] == 0.05


def test_map_package_requires_pose_graph_and_safe_assets():
    manifest = _map_manifest()
    assert manifest.serialized_pose_graph is True
    assert "pose_graph" in {asset.role for asset in manifest.assets}

    with pytest.raises(ValidationError):
        _map_manifest(pose_graph=False)

    with pytest.raises(ValidationError):
        MapAsset(
            role="occupancy_image",
            relative_path="../escape.pgm",
            sha256=HASH,
            bytes=10,
        )


def _control(message_type: str = "cmd_vel", **values) -> G3BControlMessage:
    payload = {
        "message_type": message_type,
        "request_id": "control_1",
        "goal_id": "goal_1",
        "goal_generation": 3,
        "map_id": "map_room_a",
        "run_id": "run",
        "episode_id": "episode",
        "backend_generation": 1,
        "navigation_epoch": 4,
        "session_id": "nav2_goal_3",
        "lease_ttl_s": 0.35,
        **values,
    }
    return G3BControlMessage(**payload)


def test_goal_control_is_identity_bound_and_receipt_is_not_send_success():
    control = _control(vx_m_s=0.03, vy_m_s=0.01, wz_rad_s=0.1)
    assert control.goal_generation == 3
    assert control.navigation_epoch == 4

    with pytest.raises(ValidationError):
        _control("authorize", vx_m_s=0.01)

    receipt = ControlReceipt(
        request_id=control.request_id,
        goal_id=control.goal_id,
        goal_generation=control.goal_generation,
        map_id=control.map_id,
        session_id=control.session_id,
        run_id=control.run_id,
        episode_id=control.episode_id,
        backend_generation=control.backend_generation,
        navigation_epoch=control.navigation_epoch,
        received=True,
        authorized=True,
        accepted=True,
        backend_status="DISPATCHED",
        dispatch_state="DISPATCHED",
        control_tick_id="tick",
        backend_monotonic_s=10.0,
        sim_time_s=2.0,
    )
    assert receipt.accepted is True
    rejected = receipt.model_copy(
        update={
            "request_id": "control_2",
            "accepted": False,
            "backend_status": "REJECTED",
            "dispatch_state": "REJECTED",
            "reason": "stale navigation_epoch",
        }
    )
    assert rejected.received is True
    assert rejected.accepted is False
