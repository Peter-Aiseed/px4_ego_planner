"""Factory for instantiating a CAN bus by name."""

from __future__ import annotations

from typing import Optional

from ..events import EventBus
from .auto import AutoCANBus
from .base import CANBus
from .slcan_serial import SlcanSerial
from .socketcan import SocketCANBus
from .waveshare import WaveShareUSBCAN


def create_can_bus(
    kind: str,
    *,
    channel: str,
    event_bus: EventBus,
    bitrate: int = 1_000_000,
    no_config: bool = False,
    no_filter: bool = False,
    raw_debug: bool = False,
    index: int = 0,
    baudrate: int = 2_000_000,
) -> CANBus:
    """Return a :class:`CANBus` instance for the requested transport.

    ``kind`` is one of ``"auto"``, ``"socketcan"``, ``"waveshare"``,
    ``"gs_usb"``, ``"slcan"``, ``"uart"``, or ``"uavcan"``.

    * ``"auto"`` ignores ``channel`` and picks the first known USB adapter
      (gs_usb → slcan → SocketCAN fallback).
    * ``"uart"`` / ``"slcan"`` / ``"uavcan"`` use the slcan ASCII protocol
      over a serial port (``channel`` = ``/dev/ttyTHS1`` …).
      ``baudrate`` is the UART line rate (default 115200 for uart/slcan).
    * ``"waveshare"`` uses the WaveShare USB-CAN-A binary protocol over a
      serial port (``channel`` = COM port / ``/dev/ttyUSB0``).
    """

    if kind == "auto":
        return AutoCANBus(
            event_bus=event_bus, bitrate=bitrate, index=index, baudrate=baudrate,
        )
    if kind == "socketcan":
        return SocketCANBus(
            channel=channel,
            event_bus=event_bus,
            bitrate=bitrate,
            no_filter=no_filter,
        )
    if kind == "waveshare":
        return WaveShareUSBCAN(
            channel=channel,
            event_bus=event_bus,
            bitrate=bitrate,
            no_config=no_config,
            raw_debug=raw_debug,
        )
    if kind == "gs_usb":
        from ._gsusb import GsUsbBus
        return GsUsbBus(channel=channel, event_bus=event_bus, bitrate=bitrate)
    if kind in ("slcan", "uart", "uavcan"):
        # slcan: plain serial-line CAN (MCP2515-on-UART, CAN-BUS shield, …)
        # uart: UART CAN (Jetson ttyTHS1)
        # uavcan: same physical layer as slcan; protocol handled by UAVCAN device
        return SlcanSerial(port=channel, event_bus=event_bus, bitrate=bitrate, baudrate=baudrate)
    raise ValueError(f"Unknown CAN bus kind: {kind!r}")
