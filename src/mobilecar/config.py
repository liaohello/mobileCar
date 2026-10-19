"""Application configuration loading without importing backend implementations."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = REPO_ROOT / "configs/default.yaml"


@dataclass(frozen=True)
class AppConfig:
    path: Path
    data: dict[str, Any]

    @property
    def backend(self) -> dict[str, Any]:
        return self.data["backend"]

    @property
    def profile(self) -> str:
        return str(self.data.get("profile", "unknown"))

    @property
    def safety(self) -> dict[str, Any]:
        return self.data["safety"]

    @property
    def recording(self) -> dict[str, Any]:
        return self.data["recording"]

    @property
    def ui(self) -> dict[str, Any]:
        return self.data["ui"]

    @property
    def quality(self) -> dict[str, Any]:
        return self.data.get("quality", {})

    @property
    def navigation(self) -> dict[str, Any]:
        return self.data.get("navigation", {})

    @property
    def g3b(self) -> dict[str, Any]:
        return self.data.get("g3b", {})

    @property
    def quality_root(self) -> Path:
        value = Path(self.quality.get("report_root", "runs"))
        return value if value.is_absolute() else (REPO_ROOT / value)

    @property
    def navigation_socket_path(self) -> Path:
        value = Path(
            self.navigation.get(
                "socket_path", "var/ipc/g3a_navigation.sock"
            )
        )
        path = value if value.is_absolute() else (REPO_ROOT / value)
        return path.resolve()

    @property
    def recording_root(self) -> Path:
        value = Path(self.recording["root"])
        return value if value.is_absolute() else (REPO_ROOT / value)

    def resolve_path(self, value: str | Path) -> Path:
        path = Path(value)
        return path if path.is_absolute() else (REPO_ROOT / path)


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    result = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = value
    return result


def load_config(
    path: str | Path | None = None,
    *,
    _seen: tuple[Path, ...] = (),
) -> AppConfig:
    config_path = Path(path).resolve() if path else DEFAULT_CONFIG
    raw = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError(f"{config_path}: expected a YAML mapping")
    extends = raw.pop("extends", None)
    if extends:
        parent_path = (config_path.parent / str(extends)).resolve()
        if parent_path in _seen or parent_path == config_path:
            raise ValueError(f"{config_path}: recursive extends")
        parent = load_config(parent_path, _seen=(*_seen, config_path))
        raw = _deep_merge(parent.data, raw)
    if raw.get("contract_version") != "mobilecar-1.0":
        raise ValueError(f"{config_path}: unsupported contract_version")
    for key in ("backend", "safety", "recording", "ui", "acceptance"):
        if not isinstance(raw.get(key), dict):
            raise ValueError(f"{config_path}: {key} must be a mapping")
    return AppConfig(config_path, raw)
