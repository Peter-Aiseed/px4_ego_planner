"""Auto-detecting CAN bus.

Walks the candidate list from :func:`can_host.usb_scan.list_candidates` in
priority order — USB-detected CAN adapters (gs_usb native → WaveShare →
serial slcan_serial) → SocketCAN interfaces (``can0``, ``can1``, …) → UART
CAN adapters (``/dev/ttyTHS*`` on Jetson). That list is shared with
``--bus <name>``, so ``--index`` means the same adapter for both.

If a native ``gs_usb`` open fails because userspace can't claim the device
(missing udev rule / permissions), it transparently falls back to the next
candidate in the discovery chain (typically the SocketCAN interface the same
kernel driver already created).
"""

from __future__ import annotations

import logging
from typing import Any

from ..events import EventBus
from ..usb_scan import (
    AdapterMatch,
    bus_name_for_backend,
    list_candidates,
    resolve_baudrate,
)
from .base import CANBus, CANBusError
from .factory import create_can_bus, register_bus

logger = logging.getLogger(__name__)


@register_bus("auto")
class AutoCANBus(CANBus):
    """Picks a known CAN adapter; falls back through the discovery chain."""

    def __init__(
        self,
        event_bus: EventBus,
        bitrate: int = 1_000_000,
        prefer: str | None = None,
        index: int = 0,
        baudrate: int | None = None,
    ) -> None:
        super().__init__(name="auto", event_bus=event_bus)
        self.bitrate = bitrate
        self.prefer = prefer
        self.index = index
        self._baudrate = baudrate
        self._inner: CANBus | None = None
        self._match: AdapterMatch | None = None

    # -- discovery ---------------------------------------------------------

    def _ordered_candidates(self) -> list[AdapterMatch]:
        """Candidates from :func:`usb_scan.list_candidates`, ``--prefer`` applied.

        Detection itself lives in :mod:`can_host.usb_scan` and is shared with
        ``--bus <name>``, so ``--bus auto --index N`` and
        ``--bus <name> --index N`` count the same adapters. ``--prefer`` is
        matched against the ``--bus`` name an adapter would be opened with, so
        ``--prefer slcan_serial`` also pulls forward on-board UART CAN ports.
        """
        candidates = list_candidates()
        if self.prefer:
            preferred = [
                c for c in candidates if bus_name_for_backend(c.backend) == self.prefer
            ]
            others = [
                c for c in candidates if bus_name_for_backend(c.backend) != self.prefer
            ]
            candidates = preferred + others
        return candidates

    def _resolve_baudrate(self, m: AdapterMatch) -> int:
        """UART line rate to use for one candidate adapter.

        With ``--bus auto`` the adapter is only known once the bus is being
        opened, so the rate is resolved here instead of up front: it has to
        follow the adapter actually selected by ``--index``, since two
        revisions of the same adapter run at different line rates and a
        "first known adapter wins" lookup hands the FT232R the CH340's rate.
        An explicit ``--baudrate`` still wins.
        """
        vid_pid = (m.device.vid, m.device.pid) if m.device is not None else None
        return resolve_baudrate(m.channel, self._baudrate, vid_pid)

    def _instantiate(self, m: AdapterMatch) -> CANBus:
        """Create the concrete bus for a detected adapter match."""
        self._match = m
        return create_can_bus(
            bus_name_for_backend(m.backend),
            channel=m.channel,
            event_bus=self.events,
            bitrate=self.bitrate,
            baudrate=self._resolve_baudrate(m),
        )

    # -- lifecycle ---------------------------------------------------------

    def _open(self) -> None:
        """Try candidates in priority order until one opens."""
        candidates = self._ordered_candidates()

        if not candidates:
            raise CANBusError(
                "no CAN adapter detected and no active SocketCAN interface. "
                "Run with --list to see what was scanned."
            )

        if self.index >= len(candidates):
            raise CANBusError(
                f"adapter index {self.index} out of range for "
                f"{len(candidates)} detected candidate(s). "
                "Run with --list to see what was scanned."
            )

        last_error: CANBusError | None = None
        for match in candidates[self.index :]:
            try:
                self._inner = self._instantiate(match)
                channel = match.channel
                self.name = f"auto[{type(self._inner).__name__}:{channel}]"
                logger.info("Auto-selected %s (%s)", match.backend, channel)
                self._inner.open()
                return
            except (CANBusError, ValueError) as exc:
                last_error = CANBusError(str(exc))
                logger.warning(
                    "Auto-selected %s (%s) failed to open: %s",
                    match.backend,
                    match.channel,
                    exc,
                )

        if last_error is not None:
            raise last_error
        raise CANBusError("no usable CAN adapter found")

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

    def _send_message(self, msg: Any) -> None:
        if self._inner is None:
            raise CANBusError("bus not open")
        self._inner._send_message(msg)

    def set_filters(self, filters: Any) -> None:
        if self._inner is not None:
            self._inner.set_filters(filters)

    @property
    def selected(self) -> AdapterMatch | None:
        return self._match
