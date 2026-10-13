"""Public package for the AFBR-S50 CAN host."""

from .can_bus.factory import create_can_bus
from .devices.factory import create_device
from .events import (
    CANFrame,
    DeviceMeasurement,
    EventBus,
    StatusEvent,
)

__all__ = [
    "CANFrame",
    "DeviceMeasurement",
    "EventBus",
    "StatusEvent",
    "create_can_bus",
    "create_device",
]
