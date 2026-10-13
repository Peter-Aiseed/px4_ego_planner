"""Event types and a tiny event bus used to decouple devices, buses, and the UI.

The event bus lets us plug in different CAN buses (SocketCAN, WaveShare, ...)
and different devices (AFBR-S50 ToF, future sensors, ...) without any of them
knowing about the others. A bus emits raw :class:`CANFrame` events; a device
subscribes to those frames, parses them into high-level measurements, and
emits :class:`DeviceMeasurement` (or :class:`StatusEvent`) events. The main
program just listens and renders.
"""

from __future__ import annotations

import json
import logging
from collections import defaultdict
from collections.abc import Callable
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Union

if TYPE_CHECKING:
    from .devices.measurements import Measurement

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class CANFrame:
    """A raw CAN frame passed from a bus to whoever subscribes."""

    arbitration_id: int
    dlc: int
    data: bytes
    is_extended_id: bool = False
    is_remote_frame: bool = False
    timestamp: float = 0.0


@dataclass
class DeviceMeasurement:
    """A parsed, device-specific measurement (e.g. ToF range).

    ``kind`` is the device family discriminator so consumers can dispatch
    without isinstance() checks::

        match event.kind:
            case "afbr":   ...
            case "uavcan": ...

    The wrapped :attr:`measurement` is always a :class:`Measurement`
    subclass instance that exposes ``measurement_type``, ``id``, and a
    ``to_string()`` helper so upper layers can log it uniformly.
    """

    device_id: str
    kind: str  # "afbr" | "uavcan" | ...
    measurement: Measurement
    frame: CANFrame

    def to_string(self) -> str:
        """Return a concise, human-readable one-liner for log output.

        Delegates to the wrapped measurement's own ``to_string()`` so the
        format is owned by the measurement type, not the printer.
        """
        return self.measurement.to_string()


@dataclass
class StatusEvent:
    """Structured non-measurement device event (node status, allocation, ...).

    Carried on the ``status`` topic for things that are **not** sensor
    measurements — those use :class:`DeviceMeasurement`. ``id`` holds the
    relevant node id so renderers can filter and log it like measurements;
    :meth:`to_string` produces a one-liner for log output.
    """

    device_id: str
    kind: str  # "node_status" | "allocation" | ...
    id: int | None = None  # node id this event refers to
    health: int | None = None
    mode: int | None = None
    uptime_sec: float | None = None
    message: str | None = None
    data: dict[str, Any] | None = None  # arbitrary additional fields

    def to_string(self) -> str:
        """Return a concise one-liner suitable for log output."""
        parts = [self.kind]
        if self.id is not None:
            parts.append(f"id=0x{self.id:02X}")
        if self.health is not None:
            parts.append(f"health={self.health}")
        if self.mode is not None:
            parts.append(f"mode={self.mode}")
        if self.uptime_sec is not None:
            parts.append(f"uptime={self.uptime_sec}s")
        if self.message:
            parts.append(self.message)
        for key, value in (self.data or {}).items():
            parts.append(f"{key}={value}")
        return " ".join(parts)

    def to_json(self) -> str:
        """Return the event as a JSON string for log output."""
        entry: dict[str, Any] = {"kind": self.kind, "id": self.id}
        if self.health is not None:
            entry["health"] = self.health
        if self.mode is not None:
            entry["mode"] = self.mode
        if self.uptime_sec is not None:
            entry["uptime_sec"] = self.uptime_sec
        entry.update(self.data or {})
        return json.dumps(entry, ensure_ascii=False, default=str)


Event = Union[CANFrame, DeviceMeasurement, StatusEvent]


class EventBus:
    """Minimal pub/sub. Handlers are stored per event-type name."""

    def __init__(self) -> None:
        self._handlers: defaultdict[str, list[Callable[[Any], None]]] = defaultdict(list)

    def subscribe(self, event_type: str, handler: Callable[[Any], None]) -> Callable[[], None]:
        self._handlers[event_type].append(handler)

        def unsubscribe() -> None:
            try:
                self._handlers[event_type].remove(handler)
            except (KeyError, ValueError):
                pass

        return unsubscribe

    def emit(self, event_type: str, event: Any) -> None:
        handlers = self._handlers.get(event_type)
        if not handlers:
            return
        # Fast path: the common case is a single subscriber (e.g. only the
        # AFBR device listens for frames) — avoid the tuple() copy per event.
        if len(handlers) == 1:
            try:
                handlers[0](event)
            except Exception as exc:  # noqa: BLE001
                # Last-resort guard: a buggy handler must not kill the producer.
                logger.error("handler for %r raised: %r", event_type, exc)
            return
        for handler in tuple(handlers):
            try:
                handler(event)
            except Exception as exc:  # noqa: BLE001
                # Last-resort guard: a buggy handler must not kill the producer.
                logger.error("handler for %r raised: %r", event_type, exc)