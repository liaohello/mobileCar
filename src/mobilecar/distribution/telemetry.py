"""Thread-safe in-memory navigation telemetry for health and acceptance."""

from __future__ import annotations

import threading
import time
from collections import Counter, deque
from typing import Any

from ..contracts import (
    EstimatedBaseState,
    NavigationHealth,
    NavigationObservation,
)


class NavigationTelemetrySink:
    """Retain rate/age summaries without exposing high-rate data to the UI."""

    def __init__(self, window_s: float = 5.0) -> None:
        if window_s <= 0.0:
            raise ValueError("telemetry window must be positive")
        self.window_s = float(window_s)
        self._lock = threading.RLock()
        self._capture_times: deque[float] = deque()
        self._scan_times: deque[float] = deque()
        self._estimates: deque[float] = deque()
        self._observation_count = 0
        self._scan_count = 0
        self._estimate_count = 0
        self._health_count = 0
        self._last_observation: NavigationObservation | None = None
        self._last_scan = None
        self._last_estimate: EstimatedBaseState | None = None
        self._last_health: NavigationHealth | None = None
        self._received_backend_monotonic_s: float | None = None
        self._last_delivery_lag_s: float | None = None
        self._scan_status_totals: Counter[str] = Counter()

    def publish_navigation(self, observation: NavigationObservation) -> None:
        received = time.monotonic()
        with self._lock:
            self._observation_count += 1
            self._received_backend_monotonic_s = received
            self._last_delivery_lag_s = max(
                0.0, received - float(observation.clock.capture_backend_monotonic_s)
            )
            self._last_observation = observation
            self._capture_times.append(float(observation.clock.capture_backend_monotonic_s))
            if observation.scan is not None:
                self._last_scan = observation.scan
                self._scan_count += 1
                self._scan_times.append(
                    float(observation.scan.clock.capture_backend_monotonic_s)
                )
                self._scan_status_totals.update(observation.scan.status_counts)
            self._trim(received)

    def publish_estimate(self, estimate: EstimatedBaseState) -> None:
        received = time.monotonic()
        with self._lock:
            self._estimate_count += 1
            self._last_estimate = estimate
            self._estimates.append(float(estimate.clock.capture_backend_monotonic_s))
            self._trim(received)

    def publish_health(self, health: NavigationHealth) -> None:
        with self._lock:
            self._health_count += 1
            self._last_health = health

    def _trim(self, now: float) -> None:
        threshold = now - self.window_s
        for values in (self._capture_times, self._scan_times, self._estimates):
            while values and values[0] < threshold:
                values.popleft()

    @staticmethod
    def _rate(values: deque[float]) -> float:
        if len(values) < 2:
            return 0.0
        elapsed = max(1e-6, values[-1] - values[0])
        return float((len(values) - 1) / elapsed) if elapsed > 0 else 0.0

    def status(self) -> dict[str, Any]:
        with self._lock:
            observation = self._last_observation
            estimate = self._last_estimate
            health = self._last_health
            scan = self._last_scan
            return {
                "observation_count": self._observation_count,
                "scan_count": self._scan_count,
                "estimate_count": self._estimate_count,
                "health_count": self._health_count,
                "observation_rate_hz": self._rate(self._capture_times),
                "scan_rate_hz": self._rate(self._scan_times),
                "estimate_rate_hz": self._rate(self._estimates),
                "last_delivery_lag_s": self._last_delivery_lag_s,
                "last_received_backend_monotonic_s": self._received_backend_monotonic_s,
                "scan_status_totals": dict(self._scan_status_totals),
                "latest_clock": (
                    observation.clock.model_dump(mode="json")
                    if observation is not None
                    else None
                ),
                "latest_scan": (
                    {
                        "scan_id": scan.scan_id,
                        "beam_count": scan.beam_count,
                        "valid_count": scan.valid_count,
                        "status_counts": scan.status_counts,
                        "frame_id": scan.frame_id,
                    }
                    if scan is not None
                    else None
                ),
                "latest_estimate": (
                    estimate.model_dump(mode="json")
                    if estimate is not None
                    else None
                ),
                "latest_health": (
                    health.model_dump(mode="json") if health is not None else None
                ),
            }

    def latest_scan(self) -> dict[str, Any] | None:
        with self._lock:
            if self._last_scan is None:
                return None
            return self._last_scan.model_dump(mode="json")

    def latest_observation(self) -> dict[str, Any] | None:
        with self._lock:
            if self._last_observation is None:
                return None
            return self._last_observation.model_dump(mode="json")

    def latest_evidence(self) -> dict[str, Any]:
        with self._lock:
            return {
                "status": self.status(),
                "scan": (
                    self._last_scan.model_dump(mode="json")
                    if self._last_scan is not None
                    else None
                ),
                "estimate": (
                    self._last_estimate.model_dump(mode="json")
                    if self._last_estimate is not None
                    else None
                ),
                "health": (
                    self._last_health.model_dump(mode="json")
                    if self._last_health is not None
                    else None
                ),
            }

    def close(self) -> None:
        return None
