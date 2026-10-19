"""Strict JSON-lines codec for navigation stream and control messages."""

from __future__ import annotations

import json

from ..contracts import (
    ControlReceipt,
    EstimatedBaseState,
    G3BControlMessage,
    NavigationControlMessage,
    NavigationHealth,
    NavigationObservation,
    NavigationStreamEnvelope,
)


DEFAULT_MAX_LINE_BYTES = 8 * 1024 * 1024


def encode_stream_envelope(
    message_type: str,
    payload: NavigationObservation | EstimatedBaseState | NavigationHealth,
) -> bytes:
    clock = payload.clock
    envelope = NavigationStreamEnvelope(
        message_type=message_type,
        run_id=clock.run_id,
        episode_id=clock.episode_id,
        backend_generation=clock.backend_generation,
        navigation_epoch=clock.navigation_epoch,
        backend_monotonic_s=clock.capture_backend_monotonic_s,
        payload=payload.model_dump(mode="json"),
    )
    return (
        json.dumps(
            envelope.model_dump(mode="json"),
            ensure_ascii=False,
            separators=(",", ":"),
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")


def decode_control_line(
    line: bytes,
    *,
    max_line_bytes: int = DEFAULT_MAX_LINE_BYTES,
) -> NavigationControlMessage:
    if not line or len(line) > max_line_bytes:
        raise ValueError("navigation control line has invalid size")
    if b"\n" in line[:-1] or line[-1:] != b"\n":
        raise ValueError("navigation control must be one newline-terminated message")
    value = json.loads(line[:-1].decode("utf-8"))
    return NavigationControlMessage.model_validate(value)


def decode_any_control_line(
    line: bytes,
    *,
    max_line_bytes: int = DEFAULT_MAX_LINE_BYTES,
) -> NavigationControlMessage | G3BControlMessage:
    if not line or len(line) > max_line_bytes:
        raise ValueError("navigation control line has invalid size")
    if b"\n" in line[:-1] or line[-1:] != b"\n":
        raise ValueError("navigation control must be one newline-terminated message")
    value = json.loads(line[:-1].decode("utf-8"))
    if value.get("schema_version") == 2 or value.get("interface_profile") == "g3b-1.0":
        return G3BControlMessage.model_validate(value)
    return NavigationControlMessage.model_validate(value)


def encode_control_receipt(receipt: ControlReceipt) -> bytes:
    return (
        json.dumps(
            receipt.model_dump(mode="json"),
            ensure_ascii=False,
            separators=(",", ":"),
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")
