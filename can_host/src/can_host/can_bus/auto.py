"""Auto-detecting CAN bus.

Tries, in priority order:

1. USB-detected CAN adapters (gs_usb native → WaveShare → serial slcan).
2. UART CAN adapters (``/dev/ttyTHS*`` on Jetson).
3. SocketCAN interfaces (``can0``, …).

If a native ``gs_usb`` open fails because userspace can't claim the device
(missing udev rule / permissions), it transparently falls back to the
SocketCAN interface the same kernel driver already created.

A new transport only needs to:
  * add a row to :data:`can_host.usb_scan._KNOWN`, and
  * add a case in :meth:`AutoCANBus._instantiate`.
"""

from __future__ import annotations

import logging
from typing import Any, Optional

from ..events import CANFrame, EventBus
from ..usb_scan import (
    AdapterMatch,
    detect_adapters,
    list_socketcan_interfaces,
    list_uart_can_devices,
)
from .base import CANBus, CANBusError
from .slcan_serial import SlcanSerial
from .socketcan import SocketCANBus

logger = logging.getLogger(__name__)


class AutoCANBus(CANBus):
    """Picks a known CAN adapter; falls back to ``can0`` / UART."""

    def __init__(
        self,
        event_bus: EventBus,
        bitrate: int = 1_000_000,
        prefer: Optional[str] = None,
        index: int = 0,
        baudrate: int = 115200,
    ) -> None:
        super().__init__(name="auto", event_bus=event_bus)
        self.bitrate = bitrate
        self.prefer = prefer
        self.index = index
        self._baudrate = baudrate
        self._inner: Optional[CANBus] = None
        self._match: Optional[AdapterMatch] = None

    # -- selection ---------------------------------------------------------

    def _pick(self) -> CANBus:
        """Run the detection priority chain and return a concrete bus."""
        adapters = detect_adapters()
        if self.prefer:
            adapters = [a for a in adapters if a.backend == self.prefer] + adapters

        # 1. USB-detected adapter (respect the user's --index).
        if self.index < len(adapters):
            return self._instantiate(adapters[self.index])

        # 2. UART CAN on embedded boards (Jetson ttyTHS*, TI ttyO*).
        uart_matches = list_uart_can_devices()
        if self.index < len(uart_matches):
            m = uart_matches[self.index]
            self._match = m
            return SlcanSerial(
                port=m.channel, event_bus=self.events,
                bitrate=self.bitrate, baudrate=self._baudrate,
            )

        # 3. SocketCAN interfaces (can0, can1, …).
        sc = list_socketcan_interfaces()
        if sc:
            ch = sc[self.index] if self.index < len(sc) else sc[0]
            self._match = AdapterMatch(
                device=None,  # type: ignore[arg-type]
                backend="socketcan",
                channel=ch,
                label="SocketCAN (no USB/UART adapter matched)",
            )
            return SocketCANBus(channel=ch, event_bus=self.events, bitrate=self.bitrate)

        raise CANBusError(
            "no CAN adapter detected and no active SocketCAN interface. "
            "Run with --list to see what was scanned."
        )

    def _instantiate(self, m: AdapterMatch) -> CANBus:
        """Create the concrete bus for a detected USB adapter match."""
        self._match = m
        backend = m.backend

        if backend == "gs_usb":
            try:
                import can
            except ImportError:
                raise CANBusError("python-can is not installed") from None
            from ._gsusb import GsUsbBus
            return GsUsbBus(channel=m.channel, event_bus=self.events, bitrate=self.bitrate)

        if backend == "slcan":
            return SlcanSerial(
                port=m.channel, event_bus=self.events,
                bitrate=self.bitrate, baudrate=self._baudrate,
            )

        if backend == "waveshare":
            from .waveshare import WaveShareUSBCAN
            return WaveShareUSBCAN(
                channel=m.channel, event_bus=self.events,
                bitrate=self.bitrate, no_config=False, raw_debug=False,
            )

        if backend == "socketcan":
            return SocketCANBus(channel=m.channel, event_bus=self.events, bitrate=self.bitrate)

        raise CANBusError(f"unsupported backend: {backend!r}")

    # -- lifecycle ---------------------------------------------------------

    def _open(self) -> None:
        try:
            self._inner = self._pick()
            ch = self._match.channel if self._match else "?"
            self.name = f"auto[{type(self._inner).__name__}:{ch}]"
            logger.info("Auto-selected %s (%s)", self._match.backend if self._match else "?", ch)
            self._inner.open()
        except CANBusError as exc:
            # Self-heal: if a native gs_usb open failed because userspace
            # can't claim the device (permissions / missing udev rule), fall
            # back to the SocketCAN interface the same kernel driver already
            # created.
            if self._match and self._match.backend == "gs_usb":
                sc = list_socketcan_interfaces()
                if sc:
                    logger.warning("gs_usb open failed (%s); falling back to SocketCAN %s", exc, sc[0])
                    self._match = AdapterMatch(
                        device=self._match.device,
                        backend="socketcan",
                        channel=sc[0],
                        label="SocketCAN (fallback: gs_usb permission denied)",
                    )
                    self._inner = SocketCANBus(
                        channel=sc[0], event_bus=self.events, bitrate=self.bitrate,
                    )
                    self.name = f"auto[SocketCANBus:{sc[0]}]"
                    self._inner.open()
                    return
            raise

    def _close(self) -> None:
        if self._inner is not None:
            self._inner.close()
            self._inner = None

    # -- delegation --------------------------------------------------------

    def send_remote_frame(self, can_id: int) -> None:
        if self._inner is None:
            raise CANBusError("bus not open")
        self._inner.send_remote_frame(can_id)

    def send_data_frame(self, can_id: int, data: bytes) -> None:
        if self._inner is None:
            raise CANBusError("bus not open")
        self._inner.send_data_frame(can_id, data)

    def set_filters(self, filters: Any) -> None:
        if self._inner is not None and hasattr(self._inner, "set_filters"):
            self._inner.set_filters(filters)
            return
        return None

    @property
    def selected(self) -> Optional[AdapterMatch]:
        return self._match
