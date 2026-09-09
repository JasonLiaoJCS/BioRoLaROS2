"""Low-level board bridge interface."""

from __future__ import annotations

from abc import ABC, abstractmethod


class LowLevelBridgeBase(ABC):
    @abstractmethod
    def connect(self) -> None:
        """Open the backend connection."""

    @abstractmethod
    def send_motor_command(self, cmd) -> None:
        """Send a RedRhexMotorCommand-like object."""

    @abstractmethod
    def read_motor_state(self):
        """Return a RedRhexMotorState-like payload or None."""

    @abstractmethod
    def is_alive(self) -> bool:
        """Return True if the backend heartbeat/transport is healthy."""

    @abstractmethod
    def shutdown(self) -> None:
        """Close the backend connection."""
