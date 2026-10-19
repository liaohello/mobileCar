from mobilecar.config import load_config


def test_default_config_freezes_g1_limits():
    config = load_config()
    assert config.data["contract_version"] == "mobilecar-1.0"
    assert config.safety["command_ttl_s"] == 0.5
    assert config.safety["lease_timeout_s"] == 0.5
    assert config.safety["single_action_timeout_s"] == 30.0
    assert config.safety["episode_timeout_s"] == 300.0
    assert config.safety["parking_confirm_timeout_s"] == 5.0
    assert config.recording["queue_capacity"] >= 1024
    assert config.backend["observation_profile_path"].endswith("g2a_rgbd_5hz.yaml")
    assert config.ui["adaptive_rendering"] is True


def test_server_profile_extends_default_without_running_in_release():
    config = load_config("configs/g2a_server.yaml")
    assert config.profile == "g2a_server_quality"
    assert config.backend["kind"] == "mujoco"
    assert config.recording["root"] == "../../var/runs"
    assert config.quality["report_root"] == "../../var/reports"


def test_g3a_server_profile_isolated_outputs_and_ipc():
    config = load_config("configs/g3a_server.yaml")
    assert config.profile == "g3a_navigation_foundation_server"
    assert config.backend["scene_path"].endswith("navigation_diagnostic.xml")
    assert config.backend["navigation_profile_path"].endswith(
        "g3a_navigation_v1.yaml"
    )
    assert config.navigation["enabled"] is True
    assert config.navigation["ros_domain_id"] == 73
    assert config.navigation_socket_path.as_posix().endswith(
        "var/ipc/g3a_navigation.sock"
    )
    assert ".." not in config.navigation_socket_path.parts
