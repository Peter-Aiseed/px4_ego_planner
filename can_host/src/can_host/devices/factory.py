"""Factory for instantiating a device by name.

Device classes **self-register** by decorating with :func:`register_device`
(or calling it directly) at module-import time.  Discovery is handled by the
generic :class:`~can_host._plugin_manager.PluginManager` — adding a new device requires
only a new module, **this file never needs to be edited** to register one.

A device with an optional dependency (e.g. ``dronecan`` for UAVCAN) is
imported lazily during discovery; if the module fails to import the device is
skipped with a warning rather than aborting the run.
"""

from __future__ import annotations

from typing import Any

from .._plugin_manager import PluginManager
from ..events import EventBus
from .base import Device

_devices: PluginManager[Device] = PluginManager("can_host.devices", "device")

#: Alias so device modules can do ``@register_device("name")``.
register_device = _devices.register


def list_registered_devices() -> list[str]:
    """Return all registered device names."""
    return _devices.list_registered()


def create_device(
    name: str,
    event_bus: EventBus,
    **kwargs: Any,
) -> Device:
    """Instantiate a device by registered name.

    *kwargs* are filtered to the device class's accepted parameters via
    :func:`inspect.signature`.

    Raises :class:`ValueError` if *name* is not registered.
    """
    return _devices.create(name, event_bus, **kwargs)


def create_all_devices(
    event_bus: EventBus,
    **kwargs: Any,
) -> list[Device]:
    """Instantiate all registered devices.

    The *kwargs* are filtered per-class via
    :func:`inspect.signature` so every device only receives the parameters it
    actually accepts (e.g. ``can_ids`` is forwarded to AFBR-S50, ``auto_alloc``
    to UAVCAN, …).  Modules whose optional dependency is missing are skipped
    with a warning — both import-time *and* instantiation-time failures are
    caught, so a missing ``dronecan`` does not prevent the AFBR-S50 device
    from running.
    """
    return _devices.create_all(event_bus, skip_failures=True, **kwargs)
