"""WaveShare USB-CAN-A bus (talks to the adapter over a virtual COM port)."""

from __future__ import annotations

import logging
import threading
import time
from typing import Optional

import serial

from ..events import CANFrame, EventBus
from .base import CANBus, CANBusError

logger = logging.getLogger(__name__)


class WaveShareUSBCAN(CANBus):
    BAUDRATE_SERIAL = 2_000_000

    def __init__(
        self,
        channel: str,
        event_bus: EventBus,
        bitrate: int = 1_000_000,
        no_config: bool = False,
        raw_debug: bool = False,
    ) -> None:
        super().__init__(name=f"waveshare:{channel}", event_bus=event_bus)
        self.channel = channel
        self.bitrate = bitrate
        self.no_config = no_config
        self.raw_debug = raw_debug
        self._ser: Optional[serial.Serial] = None
        self._reader_thread: Optional[threading.Thread] = None
        self._running = False

    def _open(self) -> None:
        self._ser = serial.Serial(
            port=self.channel,
            baudrate=self.BAUDRATE_SERIAL,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
        )
        if not self.no_config:
            self._configure_can()

        self._running = True
        self._reader_thread = threading.Thread(
            target=self._reader_loop, name="waveshare-reader", daemon=True
        )
        self._reader_thread.start()

    def _configure_can(self) -> None:
        baud_code = self._can_baud_code(self.bitrate)
        if baud_code is None:
            raise CANBusError(f"unsupported CAN bitrate: {self.bitrate}")

        pkt = bytearray(20)
        pkt[0] = 0xAA
        pkt[1] = 0x55
        pkt[2] = 0x12
        pkt[3] = baud_code
        pkt[4] = 0x01
        for i in range(5, 19):
            pkt[i] = 0x00
        pkt[19] = sum(pkt[2:19]) & 0xFF

        assert self._ser is not None
        self._ser.write(bytes(pkt))
        self._ser.flush()
        logger.debug("sent CAN config: baud_code=0x%02X", baud_code)

    @staticmethod
    def _can_baud_code(bitrate: int) -> Optional[int]:
        return {
            5_000_000: 0x00,
            1_000_000: 0x01,
            800_000: 0x02,
            500_000: 0x03,
            400_000: 0x04,
            250_000: 0x05,
            200_000: 0x06,
            125_000: 0x07,
            100_000: 0x08,
            50_000: 0x09,
            20_000: 0x0A,
            10_000: 0x0B,
            5_000: 0x0C,
        }.get(bitrate)

    def _reader_loop(self) -> None:
        buf = bytearray()
        assert self._ser is not None
        while self._running and self._ser.is_open:
            try:
                chunk = self._ser.read(256)
                if not chunk:
                    continue
                if self.raw_debug:
                    logger.debug("raw bytes: %s", chunk.hex())
                buf.extend(chunk)
                self._parse_buffer(buf)
            except Exception as exc:  # noqa: BLE001
                self._emit_status("error", f"reader error: {exc!r}")
                break

    def _parse_buffer(self, buf: bytearray) -> None:
        while True:
            idx = buf.find(b"\xAA")
            if idx == -1:
                del buf[:]
                return
            if idx > 0:
                del buf[:idx]
            if len(buf) < 5:
                return

            type_byte = buf[1]
            is_extended = bool(type_byte & 0x20)
            is_remote = bool(type_byte & 0x10)
            dlc = type_byte & 0x0F
            id_bytes = 4 if is_extended else 2
            total_len = 2 + id_bytes + dlc + 1
            if len(buf) < total_len:
                return
            if buf[total_len - 1] != 0x55:
                del buf[0]
                continue

            if is_extended:
                can_id = int.from_bytes(buf[2:6], byteorder="little")
                data = bytes(buf[6 : 6 + dlc])
            else:
                can_id = int.from_bytes(buf[2:4], byteorder="little")
                data = bytes(buf[4 : 4 + dlc])

            self._publish_frame(
                CANFrame(
                    arbitration_id=can_id,
                    dlc=dlc,
                    data=data,
                    is_extended_id=is_extended,
                    is_remote_frame=is_remote,
                    timestamp=time.time(),
                )
            )
            del buf[:total_len]

    def _close(self) -> None:
        self._running = False
        if self._reader_thread and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=2.0)
        self._reader_thread = None
        if self._ser and self._ser.is_open:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None

    def send_remote_frame(self, can_id: int) -> None:
        if self._ser is None or not self._ser.is_open:
            raise CANBusError("bus not open")
        pkt = bytearray(
            [
                0xAA,
                0xD0,  # standard frame, remote, DLC=0
                can_id & 0xFF,
                (can_id >> 8) & 0xFF,
                0x55,
            ]
        )
        try:
            with self._lock:
                self._ser.write(bytes(pkt))
                self._ser.flush()
        except Exception as exc:
            raise CANBusError(f"send remote 0x{can_id:X} failed: {exc}") from exc
