"""Bounded, per-consumer fanout for recorder and navigation observations."""

from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass
from typing import Any, Protocol

from ..contracts import (
    EstimatedBaseState,
    EventRecord,
    NavigationHealth,
    NavigationObservation,
    SensorObservation,
)


class NavigationConsumer(Protocol):
    """A consumer that may implement any navigation payload method."""

    def publish_navigation(self, observation: NavigationObservation) -> None: ...


@dataclass(frozen=True)
class ConsumerStats:
    name: str
    queued: int
    delivered: int
    dropped: int
    errors: int
    last_error: str | None


@dataclass(frozen=True)
class FanoutStats:
    closed: bool
    consumers: tuple[ConsumerStats, ...]


@dataclass(frozen=True)
class _Message:
    kind: str
    payload: Any


class _Consumer:
    def __init__(self, name: str, sink: Any, capacity: int) -> None:
        if capacity < 1:
            raise ValueError("consumer queue capacity must be positive")
        self.name = name
        self.sink = sink
        self.capacity = int(capacity)
        self.queue: queue.Queue[_Message | None] = queue.Queue(maxsize=self.capacity)
        self.delivered = 0
        self.dropped = 0
        self.errors = 0
        self.last_error: str | None = None
        self._closed = False
        self._lock = threading.Lock()
        self.thread = threading.Thread(
            target=self._run,
            name=f"mobilecar-fanout-{name}",
            daemon=True,
        )
        self.thread.start()

    def _enqueue(self, message: _Message) -> None:
        with self._lock:
            if self._closed:
                self.dropped += 1
                return
            try:
                self.queue.put_nowait(message)
                return
            except queue.Full:
                pass
            try:
                self.queue.get_nowait()
                self.dropped += 1
            except queue.Empty:
                pass
            try:
                self.queue.put_nowait(message)
            except queue.Full:
                self.dropped += 1

    def _run(self) -> None:
        while True:
            message = self.queue.get()
            if message is None:
                return
            try:
                if message.kind == "event":
                    self.sink.publish(message.payload)
                elif message.kind == "observation":
                    self.sink.publish_observation(message.payload)
                elif message.kind == "navigation":
                    self.sink.publish_navigation(message.payload)
                elif message.kind == "estimate":
                    self.sink.publish_estimate(message.payload)
                elif message.kind == "health":
                    self.sink.publish_health(message.payload)
                else:
                    raise ValueError(f"unsupported fanout message kind {message.kind!r}")
                self.delivered += 1
            except Exception as exc:
                self.errors += 1
                self.last_error = f"{type(exc).__name__}: {exc}"

    def close(self, timeout_s: float) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            try:
                self.queue.put_nowait(None)
            except queue.Full:
                try:
                    self.queue.get_nowait()
                    self.dropped += 1
                except queue.Empty:
                    pass
                try:
                    self.queue.put_nowait(None)
                except queue.Full:
                    pass
        self.thread.join(timeout=max(0.0, timeout_s))

    def stats(self) -> ConsumerStats:
        return ConsumerStats(
            name=self.name,
            queued=self.queue.qsize(),
            delivered=self.delivered,
            dropped=self.dropped,
            errors=self.errors,
            last_error=self.last_error,
        )


class FanoutEventSink:
    """Fan out immutable records without allowing a slow consumer to block.

    Each consumer owns an independent bounded queue. A full queue drops its
    oldest item and increments that consumer's counter, preserving isolation
    between Recorder, estimator and ROS bridge.
    """

    def __init__(
        self,
        *,
        recorder: Any,
        navigation_consumers: tuple[tuple[str, Any], ...] = (),
        queue_capacity: int = 16,
        recorder_queue_capacity: int | None = None,
        drain_timeout_s: float = 2.0,
    ) -> None:
        if queue_capacity < 1:
            raise ValueError("queue_capacity must be positive")
        self.recorder = recorder
        self.queue_capacity = int(queue_capacity)
        self.drain_timeout_s = float(drain_timeout_s)
        self._consumers: list[_Consumer] = [
            _Consumer(
                "recorder",
                recorder,
                int(queue_capacity)
                if recorder_queue_capacity is None
                else int(recorder_queue_capacity),
            )
        ]
        self._navigation_names: set[str] = set()
        for name, sink in navigation_consumers:
            if not name or any(item.name == name for item in self._consumers):
                raise ValueError(f"duplicate or empty consumer name: {name!r}")
            self._consumers.append(_Consumer(name, sink, queue_capacity))
            self._navigation_names.add(name)
        self._closed = False
        self._lock = threading.Lock()

    @staticmethod
    def _supports(sink: Any, method: str) -> bool:
        return callable(getattr(sink, method, None))

    def publish(self, event: EventRecord) -> None:
        self._consumers[0]._enqueue(_Message("event", event))

    def publish_observation(self, observation: SensorObservation) -> None:
        self._consumers[0]._enqueue(_Message("observation", observation))

    def publish_navigation(self, observation: NavigationObservation) -> None:
        for consumer in self._consumers:
            if (
                consumer.name in self._navigation_names
                and self._supports(consumer.sink, "publish_navigation")
            ):
                consumer._enqueue(_Message("navigation", observation))

    def publish_estimate(self, estimate: EstimatedBaseState) -> None:
        for consumer in self._consumers:
            if (
                consumer.name in self._navigation_names
                and self._supports(consumer.sink, "publish_estimate")
            ):
                consumer._enqueue(_Message("estimate", estimate))

    def publish_health(self, health: NavigationHealth) -> None:
        for consumer in self._consumers:
            if (
                consumer.name in self._navigation_names
                and self._supports(consumer.sink, "publish_health")
            ):
                consumer._enqueue(_Message("health", health))

    def recording_gate(self, episode_id: str) -> bool:
        method = getattr(self.recorder, "recording_gate", None)
        return bool(method is not None and method(episode_id))

    def stats(self) -> FanoutStats:
        with self._lock:
            closed = self._closed
        return FanoutStats(
            closed=closed,
            consumers=tuple(item.stats() for item in self._consumers),
        )

    def drain(self, timeout_s: float | None = None) -> bool:
        """Wait until all currently queued messages are consumed or timeout."""

        timeout = self.drain_timeout_s if timeout_s is None else float(timeout_s)
        end = time.monotonic() + max(0.0, timeout)
        while time.monotonic() < end:
            if all(item.queue.empty() for item in self._consumers):
                return True
            time.sleep(0.005)
        return all(item.queue.empty() for item in self._consumers)

    def close(self, timeout_s: float | None = None) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
        timeout = self.drain_timeout_s if timeout_s is None else float(timeout_s)
        for consumer in self._consumers:
            consumer.close(timeout)
