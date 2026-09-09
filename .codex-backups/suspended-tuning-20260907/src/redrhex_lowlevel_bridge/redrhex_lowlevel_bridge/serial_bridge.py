"""Provisional serial backend skeleton.

This is intentionally not a final MCU protocol. It provides a replaceable
framing example so Jetson-side software can be tested while the ESP32/CAN/
micro-ROS protocol is still being designed.
"""

from __future__ import annotations

import struct
import time
import zlib
from math import isfinite

from .bridge_base import LowLevelBridgeBase


class SerialLowLevelBridge(LowLevelBridgeBase):
    MAGIC = b"RRHX"
    VERSION = 1

    def __init__(self, port: str, baudrate: int = 921600, timeout_s: float = 0.005, allow_enable: bool = False) -> None:
        self.port = port
        self.baudrate = int(baudrate)
        self.timeout_s = float(timeout_s)
        self.allow_enable = bool(allow_enable)
        self.serial = None
        self.sequence = 0
        self.last_tx_time = 0.0

    def connect(self) -> None:
        try:
            import serial
        except Exception as exc:  # pragma: no cover - hardware dependency
            raise RuntimeError("pyserial is required for serial backend: pip install pyserial") from exc
        self.serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout_s)
        self.last_tx_time = time.monotonic()

    def send_motor_command(self, cmd) -> None:
        if self.serial is None:
            raise RuntimeError("Serial bridge is not connected")
        packet = self._encode_command(cmd)
        self.serial.write(packet)
        self.last_tx_time = time.monotonic()

    def read_motor_state(self):
        # TODO: replace after MCU feedback packet is finalized.
        return None

    def is_alive(self) -> bool:
        return self.serial is not None and bool(getattr(self.serial, "is_open", False))

    def shutdown(self) -> None:
        if self.serial is not None:
            self.serial.close()
            self.serial = None

    def _encode_command(self, cmd) -> bytes:
        if bool(cmd.enable) and not self.allow_enable:
            raise RuntimeError("serial.allow_enable is false; refusing enabled motor command over provisional serial protocol")
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
            int(cmd.mode) & 0xFF,
            self.sequence,
            time.time(),
            n,
            1 if cmd.enable else 0,
        )
        for arr in arrays:
            payload += struct.pack("<" + "f" * n, *[float(x) for x in arr])
        crc = zlib.crc32(payload) & 0xFFFFFFFF
        return self.MAGIC + struct.pack("<H", len(payload)) + payload + struct.pack("<I", crc)
