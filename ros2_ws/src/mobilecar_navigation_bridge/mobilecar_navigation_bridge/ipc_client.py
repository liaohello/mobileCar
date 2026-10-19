"""Reconnectable JSON-lines Unix socket client with no ROS dependency."""

from __future__ import annotations

import json
import socket
import threading
import time
from pathlib import Path
from typing import Callable


MAX_LINE_BYTES = 8 * 1024 * 1024


class JsonLineIpcClient:
    def __init__(
        self,
        socket_path: str | Path,
        on_message: Callable[[dict], None],
        *,
        reconnect_interval_s: float = 0.25,
    ) -> None:
        self.socket_path = Path(socket_path)
        self.on_message = on_message
        self.reconnect_interval_s = float(reconnect_interval_s)
        self._stop = threading.Event()
        self._send_lock = threading.Lock()
        self._socket: socket.socket | None = None
        self._thread = threading.Thread(
            target=self._read_loop,
            name="mobilecar-navigation-ipc",
            daemon=True,
        )
        self.connected = False
        self.received = 0
        self.invalid = 0
        self.sent = 0
        self.send_failures = 0
        self.last_error: str | None = None

    def start(self) -> None:
        self._thread.start()

    def _connect(self) -> socket.socket | None:
        try:
            client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            client.settimeout(1.0)
            client.connect(str(self.socket_path))
            client.settimeout(None)
            with self._send_lock:
                self._socket = client
            self.connected = True
            self.last_error = None
            return client
        except OSError as exc:
            self.connected = False
            self.last_error = f"connect: {exc}"
            return None

    def _read_loop(self) -> None:
        while not self._stop.is_set():
            client = self._connect()
            if client is None:
                self._stop.wait(self.reconnect_interval_s)
                continue
            buffer = bytearray()
            try:
                while not self._stop.is_set():
                    chunk = client.recv(262144)
                    if not chunk:
                        break
                    buffer.extend(chunk)
                    if len(buffer) > MAX_LINE_BYTES and b"\n" not in buffer:
                        self.invalid += 1
                        self.last_error = "stream line exceeds maximum size"
                        break
                    while True:
                        newline = buffer.find(b"\n")
                        if newline < 0:
                            break
                        line = bytes(buffer[:newline])
                        del buffer[: newline + 1]
                        try:
                            message = json.loads(line.decode("utf-8"))
                            if not isinstance(message, dict):
                                raise ValueError("message must be an object")
                        except Exception as exc:
                            self.invalid += 1
                            self.last_error = f"decode: {exc}"
                            continue
                        self.received += 1
                        try:
                            self.on_message(message)
                        except Exception as exc:
                            self.last_error = f"message handler: {exc}"
            except OSError as exc:
                self.last_error = f"receive: {exc}"
            finally:
                with self._send_lock:
                    if self._socket is client:
                        self._socket = None
                self.connected = False
                try:
                    client.close()
                except OSError:
                    pass
            self._stop.wait(self.reconnect_interval_s)

    def send(self, message: dict) -> bool:
        payload = (
            json.dumps(
                message,
                ensure_ascii=False,
                separators=(",", ":"),
                allow_nan=False,
            )
            + "\n"
        ).encode("utf-8")
        with self._send_lock:
            client = self._socket
            if client is None:
                self.send_failures += 1
                return False
            try:
                client.sendall(payload)
                self.sent += 1
                return True
            except OSError as exc:
                self.send_failures += 1
                self.last_error = f"send: {exc}"
                return False

    def close(self) -> None:
        self._stop.set()
        with self._send_lock:
            client = self._socket
            self._socket = None
        if client is not None:
            try:
                client.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                client.close()
            except OSError:
                pass
        self._thread.join(timeout=2.0)
