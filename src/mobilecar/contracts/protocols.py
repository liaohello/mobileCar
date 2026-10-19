"""Structural interfaces shared by the three implementation work packages."""

from __future__ import annotations

from typing import Protocol

from .models import (
    Capabilities,
    CommandRequest,
    CommandResponse,
    EpisodeControlResponse,
    EpisodeStartRequest,
    EventRecord,
    RecordingState,
    RobotSnapshot,
    SceneManifest,
    SensorObservation,
    ValidationReport,
)


class EventSink(Protocol):
    """Non-blocking event boundary owned by the data system."""

    def publish(self, event: EventRecord) -> None: ...

    def publish_observation(self, observation: SensorObservation) -> None: ...

    def recording_gate(self, episode_id: str) -> bool: ...


class RobotBackend(Protocol):
    """One physical owner per backend instance."""

    def capabilities(self) -> Capabilities: ...

    def scene_manifest(self) -> SceneManifest: ...

    def scene_asset(self, asset_id: str) -> bytes: ...

    def start(self) -> None: ...

    def submit(self, request: CommandRequest) -> CommandResponse: ...

    def snapshot(self) -> RobotSnapshot: ...

    def reset(self, request: CommandRequest) -> CommandResponse: ...

    def set_event_sink(self, sink: EventSink) -> None: ...

    def renew_lease(self, session_id: str) -> bool: ...

    def close(self, timeout_s: float = 5.0) -> None: ...


class EpisodeRecorder(Protocol):
    """Recorder boundary consumed by application assembly, never by the UI."""

    def start(
        self,
        request: EpisodeStartRequest,
        initial_snapshot: RobotSnapshot,
    ) -> EpisodeControlResponse: ...

    def stop(
        self,
        *,
        reason: str,
        final_snapshot: RobotSnapshot | None = None,
    ) -> EpisodeControlResponse: ...

    def status(self) -> RecordingState: ...

    def validate(self, episode_id: str) -> ValidationReport: ...

    def close(self, timeout_s: float = 10.0) -> None: ...


class ReplayReader(Protocol):
    """Read-only episode access used by the application and data tools."""

    def list_episodes(self) -> list[dict]: ...

    def read_episode(self, episode_id: str) -> dict: ...

    def close(self) -> None: ...
