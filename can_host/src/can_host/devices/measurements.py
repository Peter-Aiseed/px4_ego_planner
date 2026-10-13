"""Unified measurement dataclasses for AFBR-S50 and UAVCAN devices.

All measurement types inherit from :class:`Measurement`, which guarantees a
consistent interface (``measurement_type``, ``id``, ``to_string()``) so upper
layers (printers, CSV recorders, Hz counters) can dispatch on
``measurement_type`` without ``isinstance()`` chains or dict-style access.

Each measurement also carries a ``to_string()`` method that returns a concise,
human-readable one-liner suitable for log output — callers no longer need to
format measurements differently per device kind.
"""

from __future__ import annotations

import abc
import json
from dataclasses import dataclass, field
from typing import Any

READING_TYPE_NAMES: dict[int, str] = {
    0: "undefined",
    1: "valid range",
    2: "too close",
    3: "too far",
}


class Measurement(abc.ABC):
    """Abstract base for all parsed measurements.

    Subclasses are dataclasses that set ``measurement_type`` and ``id`` and
    implement :meth:`to_string`.
    """

    measurement_type: str
    id: int  # device/node identifier (CAN arbitration ID for AFBR, node ID for UAVCAN)

    @abc.abstractmethod
    def to_string(self) -> str:
        """Return a concise, human-readable one-liner for log output."""
        raise NotImplementedError

    @abc.abstractmethod
    def to_json(self, *, frame_num: int, can_id: int | None = None) -> str:
        """Return the measurement as a JSON string for log output.

        ``frame_num`` is the printer's per-run sequence number and ``can_id``
        the wrapped frame's arbitration ID — both are rendering context, not
        measurement payload, so they are passed in by the caller.
        """
        raise NotImplementedError


@dataclass
class AFBRMeasurement(Measurement):
    """A parsed AFBR-S50 1D range measurement (8-byte CAN frame)."""

    measurement_type: str = "range"
    id: int = 0  # CAN arbitration ID of the frame that produced this measurement
    range_mm: int = 0
    amplitude_lsb: int = 0
    signal_quality: int = 0
    status: int = 0
    status_desc: str = ""
    raw_data: bytes = field(default_factory=bytes)

    @property
    def is_valid(self) -> bool:
        """True when the measurement status indicates a valid reading."""
        return self.status >= 0

    def to_string(self) -> str:
        valid = "OK" if self.is_valid else "INVALID"
        base = (
            f"id=0x{self.id:03X} range={self.range_mm}mm signal_quality={self.signal_quality}% "
            f"amplitude_lsb={self.amplitude_lsb} "
            f"valid={valid}"
        )
        if self.status != 0:
            base += f" status={self.status} status_desc={self.status_desc}"
        return base

    def to_json(self, *, frame_num: int, can_id: int | None = None) -> str:
        entry: dict[str, Any] = {
            "frame": frame_num,
            "id": f"0x{self.id:03X}",
            "range_mm": self.range_mm,
            "amplitude_lsb": self.amplitude_lsb,
            "signal_quality": self.signal_quality,
            "valid": self.is_valid,
        }
        if self.status != 0:
            entry["status"] = self.status
            entry["status_desc"] = self.status_desc
        return json.dumps(entry, ensure_ascii=False)


@dataclass
class UAVCANRangeMeasurement(Measurement):
    """UAVCAN range-sensor measurement (``uavcan.equipment.range_sensor.Measurement``)."""

    measurement_type: str = "range"
    id: int = 0  # source node ID
    sensor_id: int = 0
    range_m: float = 0.0
    range_mm: float = 0.0
    sensor_type: int = 0
    reading_type: int = 0
    field_of_view: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    timestamp_us: int = 0

    def to_string(self) -> str:
        range_mm = round(self.range_mm, 1)
        reading_type = READING_TYPE_NAMES.get(self.reading_type, str(self.reading_type))
        return (
            f"id=0x{self.id:03X} range={range_mm}mm "
            f"quality={reading_type} sensor_id={self.sensor_id}"
        )

    def to_json(self, *, frame_num: int, can_id: int | None = None) -> str:
        entry: dict[str, Any] = {
            "frame": frame_num,
            "id": self.id,
            "can_id": f"0x{can_id:03X}" if can_id is not None else None,
            "type": self.measurement_type,
            "data": {
                "sensor_id": self.sensor_id,
                "range_m": self.range_m,
                "range_mm": self.range_mm,
                "sensor_type": self.sensor_type,
                "reading_type": self.reading_type,
                "field_of_view": self.field_of_view,
                "roll": self.roll,
                "pitch": self.pitch,
                "yaw": self.yaw,
                "timestamp_us": self.timestamp_us,
            },
        }
        return json.dumps(entry, ensure_ascii=False)


@dataclass
class UAVCANFlowMeasurement(Measurement):
    """UAVCAN flow measurement (``uavcan.com.hex.equipment.flow.Measurement``)."""

    measurement_type: str = "flow"
    id: int = 0  # source node ID
    integration_interval: int = 0
    rate_gyro_integral: tuple[float, float] = (0.0, 0.0)
    flow_integral: tuple[float, float] = (0.0, 0.0)
    quality: int = 0

    def to_string(self) -> str:
        return f"id=0x{self.id:02X} FLOW quality={self.quality}/255"

    def to_json(self, *, frame_num: int, can_id: int | None = None) -> str:
        entry: dict[str, Any] = {
            "frame": frame_num,
            "id": self.id,
            "can_id": f"0x{can_id:03X}" if can_id is not None else None,
            "type": self.measurement_type,
            "data": {
                "integration_interval": self.integration_interval,
                "rate_gyro_integral": list(self.rate_gyro_integral),
                "flow_integral": list(self.flow_integral),
                "quality": self.quality,
            },
        }
        return json.dumps(entry, ensure_ascii=False)