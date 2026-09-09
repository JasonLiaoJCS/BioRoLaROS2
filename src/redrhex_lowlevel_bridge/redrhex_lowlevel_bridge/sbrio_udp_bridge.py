"""Provisional NI sbRIO UDP backend skeleton.

The intended deployment is:
Jetson ROS2 -> UDP packets -> sbRIO LabVIEW RT -> FPGA / motor drivers.

This is not a final sbRIO protocol. It provides a guarded packet shape so the
Jetson side can be tested while the sbRIO LabVIEW project is being written.
"""

from __future__ import annotations

import socket
import struct
import time
import zlib
from math import isfinite

from redrhex_msgs.msg import RedRhexMotorState

from .bridge_base import LowLevelBridgeBase


class SbrioUdpBridge(LowLevelBridgeBase):
    MAGIC = b"RRHX"
    VERSION = 1
    PACKET_COMMAND = 1
    PACKET_FEEDBACK = 2
    PACKET_HEARTBEAT = 3

    def __init__(
        self,
        remote_host: str = "192.168.0.2",
        command_port: int = 15000,
        bind_host: str = "0.0.0.0",
        feedback_port: int = 15001,
        timeout_s: float = 0.002,
        heartbeat_timeout_s: float = 0.25,
        allow_enable: bool = False,
        require_feedback: bool = False,
    ) -> None:
        self.remote_host = str(remote_host)
        self.command_port = int(command_port)
        self.bind_host = str(bind_host)
        self.feedback_port = int(feedback_port)
        self.timeout_s = float(timeout_s)
        self.heartbeat_timeout_s = float(heartbeat_timeout_s)
        self.allow_enable = bool(allow_enable)
        self.require_feedback = bool(require_feedback)
        self.sock: socket.socket | None = None
        self.sequence = 0
        self.last_tx_time = 0.0
        self.last_rx_time: float | None = None
        self.last_joint_names: list[str] = []

    def connect(self) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(self.timeout_s)
        self.sock.bind((self.bind_host, self.feedback_port))
        self.last_tx_time = time.monotonic()

    def send_motor_command(self, cmd) -> None:
        if self.sock is None:
            raise RuntimeError("sbRIO UDP bridge is not connected")
        packet = self._encode_command(cmd)
        self.sock.sendto(packet, (self.remote_host, self.command_port))
        self.last_joint_names = list(cmd.joint_names)
        self.last_tx_time = time.monotonic()

    def read_motor_state(self):
        if self.sock is None:
            return None
        newest_state = None
        while True:
            try:
                packet, _addr = self.sock.recvfrom(8192)
            except socket.timeout:
                break
            self.last_rx_time = time.monotonic()
            parsed = self._decode_packet(packet)
            if parsed is not None:
                newest_state = parsed
        return newest_state

    def is_alive(self) -> bool:
        if self.sock is None:
            return False
        if not self.require_feedback:
            return True
        if self.last_rx_time is None:
            return False
        return time.monotonic() - self.last_rx_time <= self.heartbeat_timeout_s

    def shutdown(self) -> None:
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def _encode_command(self, cmd) -> bytes:
        if bool(cmd.enable) and not self.allow_enable:
            raise RuntimeError("sbrio.allow_enable is false; refusing enabled motor command over provisional UDP protocol")
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF
        n = len(cmd.joint_names)
        if n <= 0 or n > 255:
            raise ValueError(f"joint count must be 1..255, got {n}")
        arrays = [
            list(cmd.target_position_rad),
            list(cmd.target_velocity_rad_s),
            list(cmd.kp),
            list(cmd.kd),
            list(cmd.effort_limit_nm),
        ]
        for arr in arrays:
            if len(arr) != n:
                raise ValueError("All command arrays must match joint_names length")
            if not all(isfinite(float(x)) for x in arr):
                raise ValueError("Command arrays contain NaN or Inf")

        payload = struct.pack(
            "<BBIdHB",
            self.VERSION,
            self.PACKET_COMMAND,
            self.sequence,
            time.time(),
            n,
            1 if cmd.enable else 0,
        )
        for arr in arrays:
            payload += struct.pack("<" + "f" * n, *[float(x) for x in arr])
        return self._wrap_payload(payload)

    def _wrap_payload(self, payload: bytes) -> bytes:
        crc = zlib.crc32(payload) & 0xFFFFFFFF
        return self.MAGIC + struct.pack("<H", len(payload)) + payload + struct.pack("<I", crc)

    def _decode_packet(self, packet: bytes):
        if len(packet) < 10 or packet[:4] != self.MAGIC:
            return None
        payload_len = struct.unpack_from("<H", packet, 4)[0]
        expected_len = 4 + 2 + payload_len + 4
        if len(packet) != expected_len:
            return None
        payload = packet[6 : 6 + payload_len]
        received_crc = struct.unpack_from("<I", packet, 6 + payload_len)[0]
        if (zlib.crc32(payload) & 0xFFFFFFFF) != received_crc:
            return None
        if payload_len < struct.calcsize("<BBIdH"):
            return None
        version, packet_type, _sequence, _timestamp, n = struct.unpack_from("<BBIdH", payload, 0)
        if version != self.VERSION:
            return None
        if packet_type == self.PACKET_HEARTBEAT:
            return None
        if packet_type != self.PACKET_FEEDBACK:
            return None

        offset = struct.calcsize("<BBIdH")
        floats_needed = 5 * n
        bytes_needed = offset + 4 * floats_needed + n
        if payload_len != bytes_needed:
            return None
        values = struct.unpack_from("<" + "f" * floats_needed, payload, offset)
        offset += 4 * floats_needed
        fault_bytes = struct.unpack_from("<" + "B" * n, payload, offset)

        names = self.last_joint_names if len(self.last_joint_names) == n else [f"joint_{i}" for i in range(n)]
        state = RedRhexMotorState()
        state.joint_names = list(names)
        state.position_rad = [float(x) for x in values[0:n]]
        state.velocity_rad_s = [float(x) for x in values[n : 2 * n]]
        state.effort_nm = [float(x) for x in values[2 * n : 3 * n]]
        state.current_a = [float(x) for x in values[3 * n : 4 * n]]
        state.temperature_c = [float(x) for x in values[4 * n : 5 * n]]
        state.fault = [bool(x) for x in fault_bytes]
        return state
