"""Threaded adapter from raw navigation observations to fusion outputs."""

from __future__ import annotations

import threading
import time
from typing import Any, Callable

from ..contracts.navigation import (
    EstimatedBaseState,
    NavigationHealth,
    NavigationObservation,
)
from .planar import WheelGyroPlanarOdometry


class NavigationEstimationRuntime:
    def __init__(
        self,
        odometry: WheelGyroPlanarOdometry,
        emitter: Callable[[EstimatedBaseState | None, NavigationHealth], None],
    ) -> None:
        self.odometry = odometry
        self.emitter = emitter
        self._lock = threading.RLock()
        self.latest_estimate: EstimatedBaseState | None = None
        self.latest_health: NavigationHealth | None = None
        self.last_received_monotonic_s: float | None = None
        self.last_error: str | None = None

    def publish_navigation(self, observation: NavigationObservation) -> None:
        received = time.monotonic()
        try:
            step = self.odometry.observe(
                observation, received_backend_monotonic_s=received
            )
            with self._lock:
                self.last_received_monotonic_s = received
                self.latest_health = step.health
                if step.estimate is not None:
                    self.latest_estimate = step.estimate
            self.emitter(step.estimate, step.health)
        except Exception as exc:
            with self._lock:
                self.last_error = f"{type(exc).__name__}: {exc}"
            raise

    def status(self) -> dict[str, Any]:
        with self._lock:
            value = self.odometry.status()
            value.update(
                {
                    "runtime_last_received_monotonic_s": self.last_received_monotonic_s,
                    "runtime_last_error": self.last_error,
                    "latest_estimate": (
                        self.latest_estimate.model_dump(mode="json")
                        if self.latest_estimate is not None
                        else None
                    ),
                    "latest_health": (
                        self.latest_health.model_dump(mode="json")
                        if self.latest_health is not None
                        else None
                    ),
                }
            )
            return value
