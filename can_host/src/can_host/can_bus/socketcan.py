"""Linux SocketCAN bus (``can0``, ``can1``, ...)."""

from __future__ import annotations

import logging

import can
from can.typechecking import CanFilter

from ..events import EventBus
from ._listener import FrameListener
from .base import CANBus, CANBusError
from .factory import register_bus

logger = logging.getLogger(__name__)


@register_bus("socketcan")
class SocketCANBus(CANBus):
    def __init__(
        self,
        channel: str,
        event_bus: EventBus,
        bitrate: int = 1_000_000,
        no_filter: bool = False,
    ) -> None:
        super().__init__(name=f"socketcan:{channel}", event_bus=event_bus)
        self.channel = channel
        self.bitrate = bitrate
        self.no_filter = no_filter
        self._bus: can.BusABC | None = None
        self._notifier: can.Notifier | None = None
        self._filters: list[CanFilter] = []

    def _open(self) -> None:
        try:
            self._bus = can.interface.Bus(
                interface="socketcan",
                channel=self.channel,
                bitrate=self.bitrate,
                receive_own_messages=False,
            )
        except Exception as exc:
            raise CANBusError(
                f"failed to open {self.channel}: {exc}. "
                f"Try: sudo ip link set {self.channel} up type can bitrate {self.bitrate}"
            ) from exc

        if not self.no_filter and self._filters:
            try:
                self._bus.set_filters(self._filters)
            except Exception as exc:  # noqa: BLE001
                logger.warning("could not install filters: %s", exc)

        self._notifier = can.Notifier(
            self._bus,
            [FrameListener(self._publish_frame)],
            timeout=0.1,
        )

    def _close(self) -> None:
        if self._notifier is not None:
            try:
                self._notifier.stop()
            except Exception:  # noqa: BLE001
                pass
            self._notifier = None
        if self._bus is not None:
            try:
                self._bus.shutdown()
            except Exception:  # noqa: BLE001
                pass
            self._bus = None

    def _send_message(self, msg: can.Message) -> None:
        if self._bus is None:
            raise CANBusError("bus not open")
        try:
            self._bus.send(msg, timeout=1.0)
        except can.CanError as exc:
            raise CANBusError(f"send 0x{msg.arbitration_id:X} failed: {exc}") from exc

    def set_filters(self, filters: list[CanFilter]) -> None:
        """Install hardware filters. Effective on next ``open()`` or live."""
        self._filters = list(filters)
        if self._bus is not None:
            try:
                self._bus.set_filters(self._filters)
            except Exception as exc:  # noqa: BLE001
                logger.warning("filter failed: %s", exc)
