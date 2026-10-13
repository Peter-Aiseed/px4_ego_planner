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
from typing import Any

from ..events import CANFrame, EventBus

try:
    import can
    from can.typechecking import CanFilter
except Exception:  # noqa: BLE001
    can = None  # type: ignore[assignment]
    CanFilter = Any  # type: ignore[assignment,misc]

logger = logging.getLogger(__name__)


class CANBusError(RuntimeError):
    """Raised for bus-level open/send/recv failures."""


class CANBus(abc.ABC):
    """Common interface for every CAN transport."""

    EVENT_FRAME = "frame"

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
        logger.info("CAN bus %s opened", self.name)

    def close(self) -> None:
        with self._lock:
            if not self._opened:
                return
            logger.info("Closing CAN bus %s", self.name)
            try:
                self._close()
            finally:
                self._opened = False
        logger.info("CAN bus %s closed", self.name)

    # ---- sending -------------------------------------------------------

    @abc.abstractmethod
    def _send_message(self, msg: Any) -> None:
        """Backend-specific send (can.Message or equivalent)."""

    @staticmethod
    def _make_message(can_id: int, data: bytes = b"", is_remote: bool = False) -> Any:
        """Create a python-can Message (requires ``can`` extra)."""
        if can is None:
            raise RuntimeError("python-can is required for message creation")
        return can.Message(
            arbitration_id=can_id,
            is_extended_id=can_id > 0x7FF,
            is_remote_frame=is_remote,
            dlc=len(data),
            data=bytes(data) if not is_remote else b"",
        )

    def send_remote_frame(self, can_id: int) -> None:
        """Send a CAN RTR (remote) frame."""
        self._send_message(self._make_message(can_id, is_remote=True))

    def send_data_frame(self, can_id: int, data: bytes) -> None:
        """Send a CAN data frame."""
        self._send_message(self._make_message(can_id, data, is_remote=False))

    # ---- filters -------------------------------------------------------

    def set_filters(self, filters: Any) -> None:
        """Install hardware acceptance filters, e.g. ``[{"can_id": 0x1C, "can_mask": 0x7FF}]``.

        Backends that cannot filter in hardware ignore the request and keep
        passing every frame up; overriding implementations are free to apply
        the filters late (next open) rather than immediately.  Failures are
        logged, never raised, since filtering is an optimisation.
        """

    # ---- helpers -------------------------------------------------------

    def _publish_frame(self, frame: CANFrame) -> None:
        self.events.emit(self.EVENT_FRAME, frame)

    @property
    def is_open(self) -> bool:
        return self._opened
