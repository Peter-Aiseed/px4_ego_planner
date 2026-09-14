from __future__ import annotations
from dataclasses import dataclass
from typing import Optional

from ..status_codes import StatusCAN


@dataclass
class AFBRMeasurement:
    range_mm: int
    amplitude_lsb: int
    signal_quality: int
    status: int
    status_desc: str
    raw_data: bytes
    id: int = 0  # CAN arbitration ID of the frame that produced this measurement

    @property
    def is_valid(self) -> bool:
        return self.status == StatusCAN.STATUS_OK or self.status >= 0


class AFBRParser:
    # Default AFBR-S50 1D measurement CAN IDs.
    # 0b11000..0b11111 = 0x18..0x1F (8 IDs in the firmware's 1D range).
    FRAME_ID_1D = list(range(0b11000, 0b11111 + 1))
    FRAME_ID_START = 0x08
    FRAME_ID_STOP = 0x09
    EXPECTED_DLC = 8

    @staticmethod
    def parse_1d_frame(data: bytes) -> Optional[AFBRMeasurement]:
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

    @staticmethod
    def is_1d_frame(can_id: int, dlc: int) -> bool:
        return can_id in AFBRParser.FRAME_ID_1D and dlc == AFBRParser.EXPECTED_DLC

    @staticmethod
    def is_start_frame(can_id: int) -> bool:
        return can_id == AFBRParser.FRAME_ID_START

    @staticmethod
    def is_stop_frame(can_id: int) -> bool:
        return can_id == AFBRParser.FRAME_ID_STOP


def _parse_can_id(value: str) -> int:
    v = value.strip()
    if v.lower().startswith("0x"):
        return int(v, 16)
    return int(v)


def parse_can_id_list(spec: str) -> list[int]:
    """Parse a comma-separated list of IDs and ranges, e.g. '0x1C' or '0x18-0x1F'."""
    out: list[int] = []
    for token in spec.split(","):
        token = token.strip()
        if not token:
            continue
        if "-" in token:
            lo_s, hi_s = token.split("-", 1)
            lo, hi = _parse_can_id(lo_s), _parse_can_id(hi_s)
            if hi < lo:
                lo, hi = hi, lo
            out.extend(range(lo, hi + 1))
        else:
            out.append(_parse_can_id(token))
    return out
