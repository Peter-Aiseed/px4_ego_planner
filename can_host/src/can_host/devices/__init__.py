"""Device layer.

A :class:`Device` listens to ``frame`` events on the shared :class:`EventBus`,
parses them into a high-level measurement, and re-publishes
:class:`DeviceMeasurement` (or :class:`StatusEvent`) events. A device knows
nothing about which physical CAN bus is delivering the frames.
"""

from .afbr_s50 import AFBR_S50
from .base import Device
from .factory import create_all_devices, create_device, list_registered_devices

__all__ = [
    "AFBR_S50",
    "Device",
    "create_all_devices",
    "create_device",
    "list_registered_devices",
]
