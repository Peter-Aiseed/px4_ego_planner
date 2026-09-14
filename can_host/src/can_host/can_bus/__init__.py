"""CAN bus abstractions and concrete implementations.

Each bus implementation publishes raw :class:`CANFrame` events onto the
shared :class:`EventBus` using the mixin below, then exposes
``open() / close() / send_remote() / send_data()`` plus lifecycle status.
"""

from .base import CANBus, CANBusError
from .factory import create_can_bus
from ._listener import FrameListener
from .slcan_serial import SlcanSerial
from .socketcan import SocketCANBus
from .waveshare import WaveShareUSBCAN

__all__ = [
    "CANBus",
    "CANBusError",
    "SocketCANBus",
    "SlcanSerial",
    "WaveShareUSBCAN",
    "FrameListener",
    "create_can_bus",
]