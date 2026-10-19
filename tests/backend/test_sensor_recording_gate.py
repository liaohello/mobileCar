from pathlib import Path

import numpy as np

from mobilecar.backends.mujoco.backend import MujocoBackend
from mobilecar.config import load_config
from mobilecar.contracts.models import SensorKind


class GateSink:
    def __init__(self):
        self.ready = False
        self.observations = []

    def publish(self, event):
        pass

    def publish_observation(self, observation):
        self.observations.append(observation)

    def recording_gate(self, episode_id):
        return self.ready


class FakeRenderer:
    def __init__(self):
        self.depth = False

    def disable_depth_rendering(self):
        self.depth = False

    def enable_depth_rendering(self):
        self.depth = True

    def update_scene(self, data, camera=None):
        assert camera == "rgbd"

    def render(self):
        if self.depth:
            return np.full((480, 640), 0.5, dtype=np.float64)
        return np.zeros((480, 640, 3), dtype=np.uint8)


def test_rgbd_assets_are_committed_only_after_recording_gate(tmp_path):
    config = load_config()
    backend_config = dict(config.backend)
    backend_config["render_rgbd"] = True
    backend_config["realtime"] = False
    backend = MujocoBackend(
        backend_config,
        config.safety,
        config.data["acceptance"],
        tmp_path,
    )
    sink = GateSink()
    backend.set_event_sink(sink)
    backend._renderer = FakeRenderer()
    try:
        sink.ready = False
        backend._capture_rgbd()
        assert sink.observations == []
        assert not list(backend._incoming_root.rglob("*.png"))
        assert not backend._sensor_states[SensorKind.RGB].asset_uri

        sink.ready = True
        with backend.lock:
            backend._control_tick(0.01, 10)
            backend._cached_snapshot = backend._build_snapshot()
        backend._capture_rgbd()
        assert len(sink.observations) == 2
        assert {item.sensor_kind for item in sink.observations} == {
            SensorKind.RGB,
            SensorKind.DEPTH,
        }
        assert backend._sensor_states[SensorKind.RGB].asset_uri
        assert backend._sensor_states[SensorKind.DEPTH].asset_uri
        assert list(backend._incoming_root.rglob("*.png"))
        assert list(backend._incoming_root.rglob("*.npy"))
        timing = backend._last_capture_metric
        assert timing["capture_control_tick_id"]
        assert timing["feedback_sample_id"]
        assert (
            timing["capture_backend_monotonic_s"]
            <= timing["render_start_backend_monotonic_s"]
            <= timing["render_complete_backend_monotonic_s"]
            <= timing["publish_backend_monotonic_s"]
            <= timing["arrival_backend_monotonic_s"]
        )
        assert timing["lock_acquire_ms"] >= 0.0
        assert timing["state_copy_ms"] >= 0.0
        assert all(item.capture_control_tick_id for item in sink.observations)
        assert all(item.feedback_sample_id for item in sink.observations)
        assert all(
            item.timing["capture_sim_time_s"] == item.capture_sim_time_s
            for item in sink.observations
        )
    finally:
        backend.close()
