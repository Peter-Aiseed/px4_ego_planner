"""AFBR-S50 ToF device.

Parses measurement frames (default ID 0x1C, DLC 8) using the existing
:class:`AFBRParser` and emits :class:`AFBRMeasurement` events. Sends Start /
Stop Remote Frames to control the sensor.
"""

from __future__ import annotations

import logging
from typing import Optional

from ._afbr_parser import AFBRMeasurement, AFBRParser
from ..can_bus.base import CANBus
from ..events import CANFrame, EventBus
from .base import Device

logger = logging.getLogger(__name__)


class AFBR_S50(Device):
    def __init__(
        self,
        device_id: str = "afbr-s50",
        can_ids: Optional[list[int]] = None,
        auto_install_filters: bool = True,
        event_bus: Optional[EventBus] = None,
    ) -> None:
        super().__init__(device_id=device_id, event_bus=event_bus)
        self._can_ids = list(can_ids) if can_ids is not None else list(AFBRParser.FRAME_ID_1D)
        self._auto_install_filters = auto_install_filters
        AFBRParser.FRAME_ID_1D = self._can_ids

    # Device passes its bus subscription through _on_frame; we just need the
    # bus to filter to our IDs. We re-create the subscription here.
    def _on_attached(self) -> None:
        if not self._auto_install_filters or self._bus is None:
            return
        filters = [{"can_id": cid, "can_mask": 0x7FF} for cid in self._can_ids]
        setter = getattr(self._bus, "set_filters", None)
        if callable(setter):
            try:
                setter(filters)
                self._emit_status("info", f"installed filters for {[f'0x{c:02X}' for c in self._can_ids]}")
            except Exception as exc:  # noqa: BLE001
                self._emit_status("warning", f"failed to install filters: {exc!r}")

    def start(self) -> None:
        self._send_remote(AFBRParser.FRAME_ID_START)
        self._emit_status("info", f"sent Start Remote Frame ID=0x{AFBRParser.FRAME_ID_START:02X}")

    def stop(self) -> None:
        self._send_remote(AFBRParser.FRAME_ID_STOP)
        self._emit_status("info", f"sent Stop Remote Frame ID=0x{AFBRParser.FRAME_ID_STOP:02X}")

    def _on_frame(self, frame: CANFrame) -> None:
        if frame.is_remote_frame:
            return
        if not AFBRParser.is_1d_frame(frame.arbitration_id, frame.dlc):
            return
        meas = AFBRParser.parse_1d_frame(frame.data)
        if meas is None:
            return
        # Attach the CAN ID as the device identifier
        meas.id = frame.arbitration_id
        self._emit_measurement(meas, frame)
