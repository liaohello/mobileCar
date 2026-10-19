"""Translate bridge controls into the single backend command boundary."""

from __future__ import annotations

import threading
from typing import Any
from uuid import uuid4

from ..contracts import (
    CommandRequest,
    CommandResponse,
    CommandType,
    NavigationControlMessage,
    OperatorKind,
    ControlReceipt,
    G3BControlMessage,
)


class NavigationCommandSubmitter:
    """Submit ROS navigation controls without writing physics state.

    Backends may provide the preferred narrow ``submit_navigation_control``
    method. Older compatible backends receive a standard CommandRequest with
    an explicit navigation provenance payload.
    """

    def __init__(self, backend: Any) -> None:
        self.backend = backend
        self._sequence_lock = threading.Lock()
        self._session_sequences: dict[str, int] = {}
        self.last_response: CommandResponse | None = None
        self.last_error: str | None = None

    def submit(self, message: NavigationControlMessage) -> CommandResponse:
        narrow = getattr(self.backend, "submit_navigation_control", None)
        if callable(narrow):
            response = narrow(message)
            if not isinstance(response, CommandResponse):
                response = CommandResponse.model_validate(response)
            self.last_response = response
            return response

        with self._sequence_lock:
            sequence = self._session_sequences.get(message.session_id, 0) + 1
            self._session_sequences[message.session_id] = sequence
        provenance = {
            "navigation_source": "ros_navigation_test",
            "navigation_epoch": int(message.navigation_epoch),
            "source_session_id": message.session_id,
        }
        if message.message_type == "cmd_vel":
            command = CommandType.SET_BASE_TWIST
            payload = {
                "frame_id": "base_link",
                "vx_m_s": float(message.vx_m_s),
                "vy_m_s": float(message.vy_m_s),
                "wz_rad_s": float(message.wz_rad_s),
                **provenance,
            }
        elif message.message_type == "release":
            command = CommandType.ZERO_BASE
            payload = provenance
        else:
            command = CommandType.STOP
            payload = {
                "latch": True,
                "reason": message.reason or "ros_navigation_stop",
                **provenance,
            }
        request = CommandRequest(
            run_id=message.run_id,
            backend_generation=message.backend_generation,
            episode_id=message.episode_id,
            session_id=message.session_id,
            operator_id="g3a_ros_navigation_bridge",
            operator_kind=OperatorKind.AUTONOMOUS_POLICY,
            view_source="ros_navigation_test",
            command_id=f"ros_nav_{uuid4().hex}",
            command_seq=sequence,
            command_type=command,
            requested_ttl_s=float(message.lease_ttl_s),
            payload=payload,
        )
        response = self.backend.submit(request)
        self.last_response = response
        return response

    def submit_control(
        self, message: NavigationControlMessage | G3BControlMessage
    ) -> CommandResponse | ControlReceipt:
        if isinstance(message, G3BControlMessage):
            method = getattr(self.backend, "submit_g3b_control", None)
            if not callable(method):
                raise RuntimeError("backend does not support G3B goal-bound control")
            result = method(message)
            return (
                result
                if isinstance(result, ControlReceipt)
                else ControlReceipt.model_validate(result)
            )
        return self.submit(message)
