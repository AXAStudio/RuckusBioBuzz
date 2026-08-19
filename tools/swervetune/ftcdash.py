"""Minimal FTC Dashboard WebSocket client, enough to init/start/stop an OpMode.

There is no Driver Station on this setup and the Robot Controller's own web server has no OpMode
control, so without this every code deploy would need someone to walk over and press buttons. A
tuning session is dozens of deploys.

RFC 6455 framing is implemented by hand rather than pulling in ``websockets``: this has to run from
a plain Python install on whatever laptop is on the robot's WiFi.
"""

from __future__ import annotations

import base64
import json
import os
import secrets
import socket
import struct
import time

HOST = os.environ.get("SWERVE_HOST", "192.168.43.1:8080").split(":")[0]
PORT = int(os.environ.get("SWERVE_DASH_PORT", "8000"))


class DashError(RuntimeError):
    pass


class Dash:
    """A short-lived connection to the dashboard's WebSocket."""

    def __init__(self, host: str = HOST, port: int = PORT, timeout: float = 8.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.settimeout(timeout)
        self._buf = b""
        self._handshake(host, port)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass

    # ---- websocket plumbing ----------------------------------------------------

    def _handshake(self, host: str, port: int) -> None:
        key = base64.b64encode(secrets.token_bytes(16)).decode()
        req = (
            f"GET / HTTP/1.1\r\n"
            f"Host: {host}:{port}\r\n"
            f"Upgrade: websocket\r\n"
            f"Connection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\n"
            f"Sec-WebSocket-Version: 13\r\n\r\n"
        )
        self.sock.sendall(req.encode())

        while b"\r\n\r\n" not in self._buf:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise DashError("dashboard closed the connection during handshake")
            self._buf += chunk
        head, self._buf = self._buf.split(b"\r\n\r\n", 1)
        if b"101" not in head.split(b"\r\n")[0]:
            raise DashError(f"dashboard refused the upgrade: {head.decode(errors='replace')[:200]}")

    def _recv_exact(self, n: int) -> bytes:
        while len(self._buf) < n:
            chunk = self.sock.recv(65536)
            if not chunk:
                raise DashError("dashboard closed the connection")
            self._buf += chunk
        out, self._buf = self._buf[:n], self._buf[n:]
        return out

    def send(self, message: dict) -> None:
        payload = json.dumps(message).encode()
        header = bytearray([0x81])  # FIN + text frame
        n = len(payload)
        if n < 126:
            header.append(0x80 | n)
        elif n < (1 << 16):
            header.append(0x80 | 126)
            header += struct.pack(">H", n)
        else:
            header.append(0x80 | 127)
            header += struct.pack(">Q", n)
        mask = secrets.token_bytes(4)  # clients must mask
        header += mask
        self.sock.sendall(bytes(header) + bytes(b ^ mask[i % 4] for i, b in enumerate(payload)))

    def recv(self) -> dict | None:
        """Reads one frame. Returns the decoded JSON, or None for a non-text frame."""
        b0, b1 = self._recv_exact(2)
        opcode = b0 & 0x0F
        length = b1 & 0x7F
        if length == 126:
            length = struct.unpack(">H", self._recv_exact(2))[0]
        elif length == 127:
            length = struct.unpack(">Q", self._recv_exact(8))[0]
        if b1 & 0x80:  # server frames should not be masked, but handle it anyway
            mask = self._recv_exact(4)
            data = bytes(b ^ mask[i % 4] for i, b in enumerate(self._recv_exact(length)))
        else:
            data = self._recv_exact(length)

        if opcode == 0x8:
            raise DashError("dashboard sent close")
        if opcode == 0x9:  # ping -> pong
            self.sock.sendall(b"\x8a\x80" + secrets.token_bytes(4))
            return None
        if opcode != 0x1:
            return None
        try:
            return json.loads(data.decode("utf-8", "replace"))
        except json.JSONDecodeError:
            return None

    # ---- protocol --------------------------------------------------------------

    def status(self, timeout: float = 8.0) -> dict:
        """Asks for robot status and returns the first one that comes back.

        The payload sits at the top level under ``status`` - this protocol does not wrap message
        bodies in a ``data`` field, whatever the client-side naming suggests.
        """
        self.send({"type": "GET_ROBOT_STATUS"})
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.recv()
            if msg and msg.get("type") == "RECEIVE_ROBOT_STATUS":
                return msg.get("status") or {}
        raise DashError("no RECEIVE_ROBOT_STATUS within timeout")

    def init_op_mode(self, name: str) -> None:
        self.send({"type": "INIT_OP_MODE", "opModeName": name})

    def start_op_mode(self) -> None:
        self.send({"type": "START_OP_MODE"})

    def stop_op_mode(self) -> None:
        self.send({"type": "STOP_OP_MODE"})


def run_op_mode(name: str = "Swerve Bring-Up", settle_s: float = 2.5) -> dict:
    """Initialises and starts an OpMode, returning the robot status afterwards.

    Kept as a one-shot function because the dashboard drops idle connections; holding one open
    across a long tuning session is less reliable than reconnecting for the few seconds it takes.
    """
    with Dash() as d:
        before = d.status()
        if before.get("activeOpMode") not in (None, "", "$Stop$Robot$"):
            d.stop_op_mode()
            time.sleep(1.0)
        d.init_op_mode(name)
        time.sleep(1.5)
        d.start_op_mode()
        time.sleep(settle_s)
        return d.status()


def stop_op_mode() -> dict:
    with Dash() as d:
        d.stop_op_mode()
        time.sleep(1.0)
        return d.status()


if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "stop":
        print(json.dumps(stop_op_mode(), indent=2)[:1500])
    elif len(sys.argv) > 1 and sys.argv[1] == "status":
        with Dash() as d:
            print(json.dumps(d.status(), indent=2)[:3000])
    else:
        name = sys.argv[1] if len(sys.argv) > 1 else "Swerve Bring-Up"
        print(json.dumps(run_op_mode(name), indent=2)[:1500])
