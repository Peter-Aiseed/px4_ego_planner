"""Factory for instantiating a CAN bus by name.

Bus backends **self-register** by decorating their class with
:func:`register_bus` at module-import time.  Discovery is handled by the
generic :class:`~can_host._plugin_manager.PluginManager` — adding a new transport
requires only a new module, **this file never needs to be edited** to
register a bus.

A backend with an optional dependency (e.g. ``gs-usb``) is imported lazily
during discovery; if the module fails to import the backend is skipped with a
warning rather than aborting the run.
"""

from __future__ import annotations

from typing import Any

from .._plugin_manager import PluginManager
from ..events import EventBus
from .base import CANBus

_buses: PluginManager[CANBus] = PluginManager("can_host.can_bus", "bus")

#: Alias so bus backends can do ``@register_bus("name")``.
register_bus = _buses.register


def list_registered_buses() -> list[str]:
    """Return all registered bus backend names."""
    return _buses.list_registered()


def create_can_bus(
    kind: str,
    *,
    event_bus: EventBus,
    **kwargs: Any,
) -> CANBus:
    """Instantiate a bus backend by registered name.

    *kwargs* are filtered to the backend class's accepted parameters via
    :func:`inspect.signature`, so each backend only receives the parameters
    it declares — no special-casing per transport.

    Raises :class:`ValueError` if *kind* is not a registered backend.
    """
    return _buses.create(kind, event_bus, **kwargs)
