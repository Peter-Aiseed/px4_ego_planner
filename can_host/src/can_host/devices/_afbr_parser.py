"""AFBR-S50 CAN frame parser.

Parses the 8-byte 1D measurement frames (default IDs 0x18-0x1F) emitted by
the AFBR-S50 ToF sensor and returns :class:`AFBRMeasurement` instances from
:mod:`can_host.devices.measurements`.

``AFBRMeasurement`` is re-exported here so existing callers that import it from
this module (e.g. tests) keep working.
"""

from __future__ import annotations

from itertools import chain
from typing import ClassVar

from ..status_codes import StatusCAN
from .measurements import AFBRMeasurement


class AFBRParser:
    """Parse AFBR-S50 1D measurement frames.

    The parser is stateless; all methods are static.  It also knows the
    reserved Start / Stop remote-frame IDs used to control the sensor.
    """

    # Default AFBR-S50 1D measurement CAN IDs.
    # 0b11000..0b11111 = 0x18..0x1F (8 IDs in the firmware's 1D range).
    FRAME_ID_1D: ClassVar[list[int]] = list(range(0b11000, 0b11111 + 1))
    FRAME_ID_START = 0x08
    FRAME_ID_STOP = 0x09
    EXPECTED_DLC = 8

    @staticmethod
    def parse_1d_frame(data: bytes) -> AFBRMeasurement | None:
        """Parse an 8-byte AFBR-S50 1D measurement payload.

        Returns ``None`` when *data* is not exactly 8 bytes.
        """
        if len(data) != AFBRParser.EXPECTED_DLC:
            return None

        range_mm = (data[0] << 16) | (data[1] << 8) | data[2]
        amplitude_lsb = (data[3] << 8) | data[4]
        signal_quality = data[5]
        status = int.from_bytes(data[6:8], byteorder="big", signed=True)

        status_enum = StatusCAN.from_code(status)

        return AFBRMeasurement(
            range_mm=range_mm,
            amplitude_lsb=amplitude_lsb,
            signal_quality=signal_quality,
            status=status,
            status_desc=status_enum.description,
            raw_data=data,
        )


def _parse_can_id(value: str) -> int:
    """Parse a single CAN ID token, accepting ``0x``-prefixed hex or decimal."""
    v = value.strip()
    if v.lower().startswith("0x"):
        return int(v, 16)
    return int(v)


def parse_can_id_list(spec: str) -> list[int]:
    """Parse a comma-separated list of IDs and ranges, e.g. ``'0x1C'`` or ``'0x18-0x1F'``."""

    def _expand(token: str) -> list[int]:
        if "-" in token:
            lo, hi = _parse_can_id(token.split("-", 1)[0]), _parse_can_id(token.split("-", 1)[1])
            if hi < lo:
                lo, hi = hi, lo
            return list(range(lo, hi + 1))
        return [_parse_can_id(token)]

    return list(chain.from_iterable(_expand(t.strip()) for t in spec.split(",") if t.strip()))