"""Backend selection owned by application assembly."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping


def create_backend(
    backend_config: Mapping[str, Any],
    safety_config: Mapping[str, Any],
    acceptance_config: Mapping[str, Any],
    run_root: Path,
):
    kind = str(backend_config.get("kind", "mock")).lower()
    if kind == "mock":
        from .mock.backend import MockBackend

        return MockBackend(backend_config, safety_config, acceptance_config, run_root)
    if kind == "mujoco":
        from .mujoco.backend import MujocoBackend

        return MujocoBackend(backend_config, safety_config, acceptance_config, run_root)
    raise ValueError(f"unknown backend kind: {kind!r}")

