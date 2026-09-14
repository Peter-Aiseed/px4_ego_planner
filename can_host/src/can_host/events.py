"""Event types and a tiny event bus used to decouple devices, buses, and the UI.

The event bus lets us plug in different CAN buses (SocketCAN, WaveShare, ...)
and different devices (AFBR-S50 ToF, future sensors, ...) without any of them
knowing about the others. A bus emits raw :class:`CANFrame` events; a device
subscribes to those frames, parses them into high-level measurements, and
emits :class:`DeviceMeasurement` (or :class:`StatusEvent`) events. The main
program just listens and renders.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Union


@dataclass(frozen=True)
class CANFrame:
    """A raw CAN frame passed from a bus to whoever subscribes."""

    arbitration_id: int
    dlc: int
    data: bytes
    is_extended_id: bool = False
    is_remote_frame: bool = False
    timestamp: float = 0.0

    @property
    def can_id(self) -> int:
        return self.arbitration_id


@dataclass
class DeviceMeasurement:
    """A parsed, device-specific measurement (e.g. ToF range).

    ``kind`` is the device family discriminator so consumers can dispatch
    without isinstance() checks::

        match event.kind:
            case "afbr":   ...
            case "uavcan": ...
    """

    device_id: str
    kind: str  # "afbr" | "uavcan" | ...
    measurement: Any
    frame: CANFrame


@dataclass
class StatusEvent:
    """Lifecycle / error event (bus opened/closed, device started/stopped, ...)."""

    source: str
    level: str  # "info" | "warning" | "error"
    message: str


Event = Union[CANFrame, DeviceMeasurement, StatusEvent]


class EventBus:
    """Minimal pub/sub. Handlers are stored per event-type name."""

    def __init__(self) -> None:
        self._handlers: Dict[str, List[Callable[[Any], None]]] = {}

    def on(self, event_type: str, handler: Callable[[Any], None]) -> Callable[[], None]:
        self._handlers.setdefault(event_type, []).append(handler)

        def unsubscribe() -> None:
            try:
                self._handlers[event_type].remove(handler)
            except (KeyError, ValueError):
                pass

        return unsubscribe

    def emit(self, event_type: str, event: Any) -> None:
        for handler in list(self._handlers.get(event_type, ())):
            try:
                handler(event)
            except Exception as exc:  # noqa: BLE001
                # Last-resort guard: a buggy handler must not kill the producer.
                print(f"[EventBus] handler for {event_type!r} raised: {exc!r}")
