"""Abstract CAN bus base class.

Concrete buses (SocketCAN, WaveShare USB-CAN, ...) inherit from
:class:`CANBus`, manage their own reader thread, and publish
:class:`CANFrame` events on the shared :class:`EventBus` under the
``"frame"`` topic.
"""

from __future__ import annotations

import abc
import logging
import threading
from typing import Optional

from ..events import CANFrame, EventBus, StatusEvent

logger = logging.getLogger(__name__)


class CANBusError(RuntimeError):
    """Raised for bus-level open/send/recv failures."""


class CANBus(abc.ABC):
    """Common interface for every CAN transport."""

    EVENT_FRAME = "frame"
    EVENT_STATUS = "status"

    def __init__(self, name: str, event_bus: EventBus) -> None:
        self.name = name
        self.events = event_bus
        self._lock = threading.Lock()
        self._opened = False

    # ---- lifecycle -----------------------------------------------------

    @abc.abstractmethod
    def _open(self) -> None:
        """Backend-specific open (called with the lock held)."""

    @abc.abstractmethod
    def _close(self) -> None:
        """Backend-specific close (called with the lock held)."""

    def open(self) -> None:
        with self._lock:
            if self._opened:
                return
            logger.info("Opening CAN bus %s", self.name)
            self._open()
            self._opened = True
        self._emit_status("info", f"opened {self.name}")

    def close(self) -> None:
        with self._lock:
            if not self._opened:
                return
            logger.info("Closing CAN bus %s", self.name)
            try:
                self._close()
            finally:
                self._opened = False
        self._emit_status("info", f"closed {self.name}")

    # ---- sending -------------------------------------------------------

    @abc.abstractmethod
    def send_remote_frame(self, can_id: int) -> None:
        """Send a CAN RTR (remote) frame."""

    def send_data_frame(self, can_id: int, data: bytes) -> None:
        raise CANBusError(f"{type(self).__name__} does not implement data frames")

    # ---- helpers -------------------------------------------------------

    def _publish_frame(self, frame: CANFrame) -> None:
        self.events.emit(self.EVENT_FRAME, frame)

    def _emit_status(self, level: str, message: str) -> None:
        self.events.emit(self.EVENT_STATUS, StatusEvent(self.name, level, message))

    @property
    def is_open(self) -> bool:
        return self._opened
