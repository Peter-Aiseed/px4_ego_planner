"""SLCAN-over-serial CAN bus.

For boards where CAN is reached through a UART-to-CAN controller wired to a
``/dev/ttyS*`` / ``/dev/ttyTHS*`` / ``/dev/ttyUSB*`` / ``/dev/ttyACM*`` port
— e.g. an MCP2515-on-UART (common on Jetson Nano/NX devkits), or a Seed
Studio CAN-BUS shield in ``slcan`` mode.

Uses python-can's ``slcan`` interface with a batch-read override to avoid the
byte-at-a-time read loop that burns CPU on noisy adapters.
"""

from __future__ import annotations

import logging
import time
from typing import Optional

import can
from can.interfaces.slcan import slcanBus
from can.exceptions import error_check
from serial import Timeout

from ..events import CANFrame, EventBus
from .base import CANBus, CANBusError
from ._listener import FrameListener

logger = logging.getLogger(__name__)


class _FastSlcanBus(slcanBus):
    """``slcanBus`` with batch reads instead of ``read(1)`` byte-at-a-time."""

    def _read(self, timeout: Optional[float]) -> Optional[str]:
        _timeout = Timeout(timeout)

        with error_check("Could not read from serial device"):
            while not _timeout.expired():
                # Drain any already-complete message from the buffer before
                # blocking on the serial port.  Otherwise bytes that arrived
                # together with a previous message sit in the buffer until
                # *new* data shows up, which adds latency.
                for terminator in (self._ERROR, self._OK):
                    idx = self._buffer.find(terminator)
                    if idx != -1:
                        string = self._buffer[: idx + 1].decode()
                        del self._buffer[: idx + 1]
                        return string

                in_waiting = self.serialPortOrig.in_waiting
                if in_waiting == 0:
                    time.sleep(0.001)
                    continue

                chunk = self.serialPortOrig.read(in_waiting)
                if not chunk:
                    continue

                self._buffer.extend(chunk)

                for terminator in (self._ERROR, self._OK):
                    idx = self._buffer.find(terminator)
                    if idx != -1:
                        string = self._buffer[: idx + 1].decode()
                        del self._buffer[: idx + 1]
                        return string

                if len(self._buffer) > 4096:
                    self._buffer.clear()

        return None


class SlcanSerial(CANBus):
    """SLCAN-over-serial CAN bus.

    ``port``      serial device path (``/dev/ttyTHS1``, ``/dev/ttyACM0``, …).
    ``baudrate``  UART line rate (default 115200; 2 000 000 for WaveShare).
    ``bitrate``   CAN bitrate negotiated by the slcan adapter.
    """

    def __init__(
        self,
        port: str,
        event_bus: EventBus,
        bitrate: int = 1_000_000,
        baudrate: int = 115_200,
    ) -> None:
        super().__init__(name=f"slcan:{port}", event_bus=event_bus)
        self.port = port
        self.bitrate = bitrate
        self.baudrate = baudrate
        self._bus: Optional[can.BusABC] = None
        self._notifier: Optional[can.Notifier] = None

    def _open(self) -> None:
        try:
            self._bus = _FastSlcanBus(
                channel=self.port,
                tty_baudrate=self.baudrate,
                bitrate=self.bitrate,
                timeout=0.1,
            )
        except Exception as exc:
            raise CANBusError(
                f"slcan open on {self.port} at {self.baudrate} baud failed: {exc}. "
                f"Ensure the port exists, no other process holds it, and the "
                f"baud rate matches the adapter firmware."
            ) from exc

        self._notifier = can.Notifier(
            self._bus, [FrameListener(self._publish_frame, self._emit_status)], timeout=0.1,
        )

    def _close(self) -> None:
        if self._notifier is not None:
            try:
                self._notifier.stop()
            except Exception:
                pass
            self._notifier = None
        if self._bus is not None and hasattr(self._bus, "shutdown"):
            try:
                self._bus.shutdown()
            except Exception:
                pass
        self._bus = None

    def send_remote_frame(self, can_id: int) -> None:
        if self._bus is None:
            raise CANBusError("bus not open")
        msg = can.Message(
            arbitration_id=can_id,
            is_extended_id=can_id > 0x7FF,
            is_remote_frame=True,
            dlc=0,
            data=b"",
        )
        try:
            self._bus.send(msg, timeout=1.0)
        except can.CanError as exc:
            raise CANBusError(f"send remote 0x{can_id:X} failed: {exc}") from exc

    def send_data_frame(self, can_id: int, data: bytes) -> None:
        if self._bus is None:
            raise CANBusError("bus not open")
        msg = can.Message(
            arbitration_id=can_id,
            is_extended_id=can_id > 0x7FF,
            is_remote_frame=False,
            dlc=len(data),
            data=bytes(data),
        )
        try:
            self._bus.send(msg, timeout=1.0)
        except can.CanError as exc:
            raise CANBusError(f"send data 0x{can_id:X} failed: {exc}") from exc

    def set_filters(self, filters) -> None:
        if self._bus is not None and hasattr(self._bus, "set_filters"):
            return self._bus.set_filters(filters)
        return None
