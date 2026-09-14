"""Factory for instantiating a device by name."""

from __future__ import annotations

import inspect
from typing import Any, Optional

from ..events import EventBus
from .afbr_s50 import AFBR_S50
from .base import Device
from .uavcan import UAVCANDevice


_REGISTRY: dict[str, type[Device]] = {
    "afbr-s50": AFBR_S50,
    "uavcan": UAVCANDevice,
}


def register_device(name: str, cls: type[Device]) -> None:
    """Register a custom device class under ``name``."""
    _REGISTRY[name] = cls


def list_registered_devices() -> list[str]:
    """Return all registered device names."""
    return list(_REGISTRY.keys())


def create_device(
    name: str,
    event_bus: EventBus,
    **kwargs: Any,
) -> Device:
    """Instantiate a device by registered name."""
    if name not in _REGISTRY:
        raise ValueError(f"unknown device: {name!r}. Known: {sorted(_REGISTRY)}")
    cls = _REGISTRY[name]
    instance = cls(event_bus=event_bus, **kwargs)
    return instance


def create_all_devices(
    event_bus: EventBus,
    **kwargs: Any,
) -> list[Device]:
    """Instantiate all registered devices, filtering kwargs per device."""
    devices: list[Device] = []
    for name, cls in _REGISTRY.items():
        sig = inspect.signature(cls.__init__)
        valid = {k: v for k, v in kwargs.items() if k in sig.parameters}
        devices.append(cls(event_bus=event_bus, **valid))
    return devices
