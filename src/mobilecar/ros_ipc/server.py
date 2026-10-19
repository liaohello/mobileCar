"""Single-client Unix socket transport for the G3A ROS bridge."""

from __future__ import annotations

import json
import os
import queue
import socket
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

from ..contracts import (
    ControlReceipt,
    EstimatedBaseState,
    G3BControlMessage,
    NavigationControlMessage,
    NavigationHealth,
    NavigationObservation,
)
from .codec import (
    DEFAULT_MAX_LINE_BYTES,
    decode_any_control_line,
    encode_control_receipt,
    encode_stream_envelope,
)


@dataclass(frozen=True)
class NavigationIpcStats:
    running: bool
    connected: bool
    queued_outbound: int
    sent_messages: int
    dropped_outbound: int
    received_controls: int
    invalid_controls: int
    command_handler_errors: int
    write_errors: int
    last_error: str | None


class NavigationIpcServer:
    """Non-blocking producer side with a bounded writer and command reader.

    The server allows one bridge client at a time. Live observations are only
    queued while a client is connected, so a reconnecting bridge cannot receive
    stale captures from a previous connection as if they were current.
    """

    def __init__(
        self,
        socket_path: str | Path,
        command_handler: Callable[
            [NavigationControlMessage | G3BControlMessage], object
        ],
        *,
        queue_capacity: int = 64,
        max_line_bytes: int = DEFAULT_MAX_LINE_BYTES,
        drain_timeout_s: float = 2.0,
    ) -> None:
        if queue_capacity < 1:
            raise ValueError("IPC queue_capacity must be positive")
        if max_line_bytes < 1024:
            raise ValueError("IPC max_line_bytes is too small")
        self.socket_path = Path(socket_path)
        self.command_handler = command_handler
        self.queue_capacity = int(queue_capacity)
        self.max_line_bytes = int(max_line_bytes)
        self.drain_timeout_s = float(drain_timeout_s)
        self._outbound: queue.Queue[bytes | None] = queue.Queue(
            maxsize=self.queue_capacity
        )
        self._control_outbound: queue.Queue[bytes | None] = queue.Queue(
            maxsize=max(64, self.queue_capacity)
        )
        self._listener: socket.socket | None = None
        self._client: socket.socket | None = None
        self._client_lock = threading.Lock()
        self._closed = threading.Event()
        self._accept_thread: threading.Thread | None = None
        self._writer_thread: threading.Thread | None = None
        self._client_threads: set[threading.Thread] = set()
        self._threads_lock = threading.Lock()
        self._sent = 0
        self._dropped = 0
        self._received = 0
        self._invalid = 0
        self._handler_errors = 0
        self._write_errors = 0
        self._last_error: str | None = None

    @property
    def running(self) -> bool:
        return bool(self._listener is not None and not self._closed.is_set())

    def start(self) -> None:
        if self.running:
            return
        parent = self.socket_path.parent
        parent.mkdir(parents=True, exist_ok=True)
        if self.socket_path.exists():
            if not self.socket_path.is_socket():
                raise ValueError(f"IPC path exists and is not a socket: {self.socket_path}")
            self.socket_path.unlink()
        listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        listener.settimeout(0.2)
        listener.bind(str(self.socket_path))
        os.chmod(self.socket_path, 0o600)
        listener.listen(1)
        self._listener = listener
        self._closed.clear()
        self._accept_thread = threading.Thread(
            target=self._accept_loop,
            name="mobilecar-ipc-accept",
            daemon=True,
        )
        self._writer_thread = threading.Thread(
            target=self._writer_loop,
            name="mobilecar-ipc-writer",
            daemon=True,
        )
        self._accept_thread.start()
        self._writer_thread.start()

    def publish_navigation(self, observation: NavigationObservation) -> None:
        self._publish(encode_stream_envelope("observation", observation))

    def publish_estimate(self, estimate: EstimatedBaseState) -> None:
        self._publish(encode_stream_envelope("estimate", estimate))

    def publish_health(self, health: NavigationHealth) -> None:
        self._publish(encode_stream_envelope("health", health))

    def publish_heartbeat(
        self,
        *,
        run_id: str,
        episode_id: str,
        backend_generation: int,
        navigation_epoch: int,
        backend_monotonic_s: float,
    ) -> None:
        now = float(backend_monotonic_s)
        payload = (
            json.dumps(
                {
                    "schema_version": 1,
                    "message_type": "heartbeat",
                    "run_id": run_id,
                    "episode_id": episode_id,
                    "backend_generation": int(backend_generation),
                    "navigation_epoch": int(navigation_epoch),
                    "backend_monotonic_s": now,
                    "payload": {},
                },
                separators=(",", ":"),
                allow_nan=False,
            )
            + "\n"
        ).encode("utf-8")
        self._publish(payload)

    def _publish(self, payload: bytes) -> None:
        if self._closed.is_set():
            self._dropped += 1
            return
        with self._client_lock:
            connected = self._client is not None
        if not connected:
            self._dropped += 1
            return
        try:
            self._outbound.put_nowait(payload)
            return
        except queue.Full:
            pass
        try:
            self._outbound.get_nowait()
            self._dropped += 1
        except queue.Empty:
            pass
        try:
            self._outbound.put_nowait(payload)
        except queue.Full:
            self._dropped += 1

    def _accept_loop(self) -> None:
        assert self._listener is not None
        while not self._closed.is_set():
            try:
                client, _ = self._listener.accept()
            except socket.timeout:
                continue
            except OSError as exc:
                if not self._closed.is_set():
                    self._last_error = f"accept: {exc}"
                return
            client.settimeout(0.5)
            old = self._replace_client(client)
            if old is not None:
                _close_socket(old)
            thread = threading.Thread(
                target=self._client_loop,
                args=(client,),
                name="mobilecar-ipc-client",
                daemon=True,
            )
            with self._threads_lock:
                self._client_threads.add(thread)
            thread.start()

    def _replace_client(self, client: socket.socket) -> socket.socket | None:
        with self._client_lock:
            old = self._client
            self._client = client
            return old

    def _client_identity_matches(self, client: socket.socket) -> bool:
        with self._client_lock:
            return self._client is client

    def _client_loop(self, client: socket.socket) -> None:
        buffer = bytearray()
        try:
            while not self._closed.is_set() and self._client_identity_matches(client):
                try:
                    chunk = client.recv(65536)
                except socket.timeout:
                    continue
                except OSError:
                    break
                if not chunk:
                    break
                buffer.extend(chunk)
                if len(buffer) > self.max_line_bytes and b"\n" not in buffer:
                    self._invalid += 1
                    self._last_error = "control line exceeds maximum size"
                    break
                while True:
                    newline = buffer.find(b"\n")
                    if newline < 0:
                        break
                    line = bytes(buffer[: newline + 1])
                    del buffer[: newline + 1]
                    try:
                        message = decode_any_control_line(
                            line, max_line_bytes=self.max_line_bytes
                        )
                        self._received += 1
                    except Exception as exc:
                        self._invalid += 1
                        self._last_error = f"invalid control: {exc}"
                        continue
                    try:
                        result = self.command_handler(message)
                        if isinstance(message, G3BControlMessage):
                            if isinstance(result, ControlReceipt):
                                self._publish_control_receipt(
                                    encode_control_receipt(result)
                                )
                            elif isinstance(result, dict):
                                self._publish_control_receipt(
                                    encode_control_receipt(
                                        ControlReceipt.model_validate(result)
                                    )
                                )
                    except Exception as exc:
                        self._handler_errors += 1
                        self._last_error = f"command handler: {exc}"
        finally:
            with self._client_lock:
                if self._client is client:
                    self._client = None
            _close_socket(client)
            with self._threads_lock:
                self._client_threads.discard(threading.current_thread())

    def _writer_loop(self) -> None:
        while True:
            try:
                message = self._control_outbound.get_nowait()
            except queue.Empty:
                try:
                    message = self._outbound.get(timeout=0.02)
                except queue.Empty:
                    continue
            if message is None:
                return
            with self._client_lock:
                client = self._client
            if client is None:
                self._dropped += 1
                continue
            try:
                client.sendall(message)
                self._sent += 1
            except OSError as exc:
                self._write_errors += 1
                self._last_error = f"write: {exc}"
                with self._client_lock:
                    if self._client is client:
                        self._client = None
                _close_socket(client)

    def _publish_control_receipt(self, payload: bytes) -> None:
        if self._closed.is_set():
            self._dropped += 1
            return
        with self._client_lock:
            connected = self._client is not None
        if not connected:
            self._dropped += 1
            return
        try:
            self._control_outbound.put_nowait(payload)
        except queue.Full:
            try:
                self._control_outbound.get_nowait()
            except queue.Empty:
                pass
            self._dropped += 1
            try:
                self._control_outbound.put_nowait(payload)
            except queue.Full:
                self._dropped += 1

    def stats(self) -> NavigationIpcStats:
        with self._client_lock:
            connected = self._client is not None
        return NavigationIpcStats(
            running=self.running,
            connected=connected,
            queued_outbound=self._outbound.qsize()
            + self._control_outbound.qsize(),
            sent_messages=self._sent,
            dropped_outbound=self._dropped,
            received_controls=self._received,
            invalid_controls=self._invalid,
            command_handler_errors=self._handler_errors,
            write_errors=self._write_errors,
            last_error=self._last_error,
        )

    def close(self, timeout_s: float | None = None) -> None:
        if self._closed.is_set():
            return
        self._closed.set()
        listener = self._listener
        self._listener = None
        if listener is not None:
            _close_socket(listener)
        with self._client_lock:
            client = self._client
            self._client = None
        if client is not None:
            _close_socket(client)
        timeout = self.drain_timeout_s if timeout_s is None else float(timeout_s)
        end = time.monotonic() + max(0.0, timeout)
        try:
            self._control_outbound.put_nowait(None)
        except queue.Full:
            try:
                self._control_outbound.get_nowait()
                self._control_outbound.put_nowait(None)
            except queue.Empty:
                pass
            except queue.Full:
                pass
        for thread in (self._accept_thread, self._writer_thread):
            if thread is not None:
                thread.join(timeout=max(0.0, end - time.monotonic()))
        with self._threads_lock:
            clients = tuple(self._client_threads)
        for thread in clients:
            thread.join(timeout=max(0.0, end - time.monotonic()))
        if self.socket_path.exists() and self.socket_path.is_socket():
            self.socket_path.unlink()


def _close_socket(value: socket.socket) -> None:
    try:
        value.shutdown(socket.SHUT_RDWR)
    except OSError:
        pass
    try:
        value.close()
    except OSError:
        pass
