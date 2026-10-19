import threading
import time

from mobilecar.contracts.models import EventRecord
from mobilecar.distribution import FanoutEventSink, NavigationTelemetrySink


def _event(sequence: int) -> EventRecord:
    return EventRecord(
        event_type="fixture",
        run_id="run",
        episode_id="episode",
        wall_utc="2026-10-17T00:00:00Z",
        backend_monotonic_s=float(sequence),
        payload={"sequence": sequence},
    )


class _Recorder:
    def __init__(self):
        self.events = []
        self.gate_calls = []

    def publish(self, event):
        self.events.append(event)

    def publish_observation(self, observation):
        raise AssertionError("not used")

    def recording_gate(self, episode_id):
        self.gate_calls.append(episode_id)
        return True


class _SlowNavigation:
    def __init__(self):
        self.release = threading.Event()
        self.started = threading.Event()
        self.count = 0

    def publish_navigation(self, observation):
        self.started.set()
        self.release.wait(timeout=1.0)
        self.count += 1


def test_slow_navigation_does_not_block_recorder_or_producer():
    recorder = _Recorder()
    navigation = _SlowNavigation()
    sink = FanoutEventSink(
        recorder=recorder,
        navigation_consumers=(("navigation", navigation),),
        queue_capacity=2,
    )
    try:
        for sequence in range(10):
            sink.publish(_event(sequence))
        for _ in range(10):
            sink.publish_navigation(object())
        assert sink.recording_gate("episode") is True
        deadline = time.monotonic() + 1.0
        while len(recorder.events) < 10 and time.monotonic() < deadline:
            time.sleep(0.005)
        assert recorder.events
        assert recorder.events[-1].payload["sequence"] == 9
        assert [item.payload["sequence"] for item in recorder.events] == sorted(
            item.payload["sequence"] for item in recorder.events
        )
        navigation.release.set()
        deadline = time.monotonic() + 1.0
        while not sink.drain(0.05) and time.monotonic() < deadline:
            pass
        stats = {item.name: item for item in sink.stats().consumers}
        assert stats["recorder"].queued <= 2
        assert stats["navigation"].dropped >= 8
    finally:
        navigation.release.set()
        sink.close()


def test_observation_mode_is_per_consumer():
    recorder = _Recorder()
    navigation = _SlowNavigation()
    sink = FanoutEventSink(
        recorder=recorder,
        navigation_consumers=(("navigation", navigation),),
        queue_capacity=4,
    )
    try:
        sink.publish(_event(1))
        sink.publish(_event(2))
        assert sink.drain() is True
        assert len(recorder.events) == 2
        assert navigation.started.is_set() is False
    finally:
        sink.close()


def test_telemetry_summary_does_not_embed_full_scan():
    telemetry = NavigationTelemetrySink()
    status = telemetry.status()
    assert status["observation_count"] == 0
    assert status["latest_scan"] is None
    assert status["latest_estimate"] is None
