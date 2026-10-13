"""Native gs_usb backend (CANable, candleLight, …).

Used only when the kernel has *not* claimed the device with the in-tree
``gs_usb`` driver — i.e. userspace (python-can + libusb) drives it directly.
"""

from __future__ import annotations

import can

from ..events import EventBus
from ._listener import FrameListener
from .base import CANBus, CANBusError
from .factory import register_bus


@register_bus("gs_usb")
class GsUsbBus(CANBus):
    def __init__(self, channel: str, event_bus: EventBus, bitrate: int) -> None:
        super().__init__(name=f"gs_usb:{channel}", event_bus=event_bus)
        self._bitrate = bitrate
        self._channel = channel
        self._bus: can.BusABC | None = None
        self._notifier: can.Notifier | None = None

    def _open(self) -> None:
        try:
            try:
                channel_arg: int | str = int(self._channel)
            except (TypeError, ValueError):
                channel_arg = self._channel
            self._bus = can.interface.Bus(
                interface="gs_usb",
                channel=channel_arg,
                bitrate=self._bitrate,
                receive_own_messages=False,
            )
        except Exception as exc:
            raise CANBusError(f"gs_usb open failed: {exc}") from exc

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
