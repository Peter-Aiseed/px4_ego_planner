"""Abstract device base class.

A device:

* subscribes to ``frame`` events on the shared event bus,
* parses the frames that match its protocol into measurements,
* emits ``measurement`` and ``status`` events.

It also exposes ``start()`` / ``stop()`` which the host can use to send
control frames (e.g. an AFBR-S50 Start/Stop Remote Frame).
"""

from __future__ import annotations

import abc
import logging
from typing import Any, Callable, Optional

from ..can_bus.base import CANBus
from ..events import CANFrame, DeviceMeasurement, EventBus, StatusEvent

logger = logging.getLogger(__name__)


class Device(abc.ABC):
    EVENT_MEASUREMENT = "measurement"
    EVENT_STATUS = "status"

    def __init__(self, device_id: str, event_bus: Optional[EventBus] = None) -> None:
        self.device_id = device_id
        self.events: EventBus = event_bus  # type: ignore[assignment]
        self._bus: Optional[CANBus] = None
        self._unsubscribe: Optional[Callable[[], None]] = None

    # ---- wiring --------------------------------------------------------

    def attach(self, bus: CANBus) -> None:
        """Subscribe to the bus's frame events."""
        if self._unsubscribe is not None:
            return
        self._bus = bus
        self._unsubscribe = bus.events.on(bus.EVENT_FRAME, self._on_frame)
        self._on_attached()

    def detach(self) -> None:
        if self._unsubscribe is not None:
            try:
                self._unsubscribe()
            except Exception:
                pass
            self._unsubscribe = None
        self._bus = None

    # ---- lifecycle hooks ----------------------------------------------

    def _on_attached(self) -> None:
        """Hook for subclasses (e.g. install bus filters)."""

    @abc.abstractmethod
    def start(self) -> None:
        """Tell the device to begin emitting measurements."""

    @abc.abstractmethod
    def stop(self) -> None:
        """Tell the device to stop emitting measurements."""

    # ---- frame handling -----------------------------------------------

    @abc.abstractmethod
    def _on_frame(self, frame: CANFrame) -> None:
        """Parse a frame and emit a measurement if applicable."""

    # ---- helpers -------------------------------------------------------

    def _send_remote(self, can_id: int) -> None:
        if self._bus is None:
            raise RuntimeError(f"device {self.device_id!r} not attached to a bus")
        self._bus.send_remote_frame(can_id)

    def _emit_measurement(self, measurement: Any, frame: CANFrame, kind: str = "afbr") -> None:
        self.events.emit(
            self.EVENT_MEASUREMENT,
            DeviceMeasurement(self.device_id, kind, measurement, frame),
        )

    def _emit_status(self, level: str, message: str) -> None:
        self.events.emit(self.EVENT_STATUS, StatusEvent(self.device_id, level, message))
