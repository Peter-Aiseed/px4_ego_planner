"""WaveShare USB-CAN-A bus (talks to the adapter over its virtual COM port)."""

from __future__ import annotations

import logging
import struct
import threading
import time

import can
import serial

from ..events import CANFrame, EventBus
from .base import CANBus, CANBusError
from .factory import register_bus

logger = logging.getLogger(__name__)

# Wire format of the variable-length protocol:
#   AA <type> <can id LE> <dlc data bytes> 55
#
# The <type> byte is the vendor's own layout, documented in
# reference/waveshare_USB-CAN-A/send.py:56-58 and re-read by receive.py:57-62:
#   bit 7-6  frame header marker, always set
#   bit 5    0 = standard frame (2-byte id), 1 = extended frame (4-byte id)
#   bit 4    0 = data frame, 1 = remote frame
#   bit 3-0  frame data length, 0..8
HEADER = 0xAA
TRAILER = 0x55
HEADER_MARKER = 0xC0
HEADER_EXTENDED = 0x20
HEADER_REMOTE = 0x10
HEADER_DLC = 0x0F
MAX_DLC = 8

# AA <type> <can id LE> <data…> 55, little-endian id.  Two structs rather than
# a bytearray + bytes() pair: the fixed part is packed in one shot.  The id is
# 2 bytes for a standard frame and 4 for an extended one, so the width has to be
# chosen per frame.
_BINARY_STANDARD = struct.Struct("<BBH")
_BINARY_EXTENDED = struct.Struct("<BBI")
_TRAILER = bytes([TRAILER])
_HEADER_BYTES = bytes([HEADER])

# Config packet byte 4 selects the frame format the adapter uses.  DroneCAN uses
# 29-bit extended identifiers, so the adapter is configured for extended frames.
FRAME_TYPE_STANDARD = 0x01
FRAME_TYPE_EXTENDED = 0x02


class WaveShareMessage(can.Message):
    """A CAN message that also carries the adapter's serial bytes for it.

    ``binary`` is the complete ``AA … 55`` byte string, built once when the
    message is created so the send path never has to take the frame apart again
    just to encode it.  It is raw binary for the serial port, not CAN payload
    text.

    This stays a plain ``can.Message`` otherwise: ``bytes(msg)`` is the CAN
    payload exactly as python-can defines it, so nothing else about the frame
    behaves differently just because this one knows its wire format.

    Args:
        can_id: CAN arbitration id; ids above 0x7FF are encoded as extended.
        data: Payload bytes, ignored for remote frames.
        is_remote: Whether this is an RTR frame with no payload.

    Raises:
        CANBusError: If a data frame carries more than ``MAX_DLC`` bytes.
    """

    __slots__ = ("binary",)

    def __init__(self, can_id: int, data: bytes = b"", is_remote: bool = False) -> None:
        is_extended = can_id > 0x7FF
        super().__init__(
            arbitration_id=can_id,
            is_extended_id=is_extended,
            is_remote_frame=is_remote,
            dlc=len(data),
            data=bytes(data) if not is_remote else b"",
        )
        if len(data) > MAX_DLC and not is_remote:
            raise CANBusError(f"data frame DLC exceeds 8 bytes: {len(data)}")

        # Header mask, then the extended bit, then either the remote bit or the
        # DLC -- the same fields _parse_buffer reads back out of this byte.
        type_byte = (
            HEADER_MARKER
            | (HEADER_EXTENDED if is_extended else 0)
            | (HEADER_REMOTE if is_remote else len(data))
        )
        pack = _BINARY_EXTENDED if is_extended else _BINARY_STANDARD
        self.binary = (
            pack.pack(HEADER, type_byte, can_id)
            + (b"" if is_remote else bytes(data))
            + _TRAILER
        )


@register_bus("waveshare")
class WaveShareUSBCAN(CANBus):
    """WaveShare USB-CAN-A adapter driven through its CH340 virtual COM port.

    A single reader thread pulls bytes off the port and publishes parsed frames
    on the event bus.  It also counts the frames it has to skip because the
    stream desynchronised, since a single dropped or extra byte silently
    truncates a multi-frame transfer and there is no other symptom to go by.
    """

    def __init__(
        self,
        channel: str,
        event_bus: EventBus,
        bitrate: int = 1_000_000,
        baudrate: int = 2_000_000,
        no_config: bool = False,
        raw_debug: bool = False,
    ) -> None:
        super().__init__(name=f"waveshare:{channel}", event_bus=event_bus)
        self.channel = channel
        self.bitrate = bitrate
        self.baudrate = baudrate
        self.no_config = no_config
        self.raw_debug = raw_debug
        self._ser: serial.Serial | None = None
        self._reader_thread: threading.Thread | None = None
        self._running = False
        self._resyncs = 0
        self._rejected_type_bytes: set[int] = set()

    def _open(self) -> None:
        logger.info(
            "port = %s, bit-rate= %d, baud-rate = %d",
            self.channel,
            self.bitrate,
            self.baudrate,
        )
        self._ser = serial.Serial(
            port=self.channel,
            baudrate=self.baudrate,
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
        pkt[4] = FRAME_TYPE_STANDARD
        for i in range(5, 19):
            pkt[i] = 0x00
        pkt[19] = sum(pkt[2:19]) & 0xFF

        assert self._ser is not None
        self._ser.write(bytes(pkt))
        self._ser.flush()
        logger.debug("sent CAN config: baud_code=0x%02X", baud_code)

    @staticmethod
    def _can_baud_code(bitrate: int) -> int | None:
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
        logger.debug("begin")

        buf = bytearray()
        ser = self._ser
        assert ser is not None
        while self._running and ser.is_open:
            try:
                # Read only what the driver already buffered so a frame is parsed
                # the moment it lands.  read(15) would block until 15 bytes
                # accumulate -- with sparse ToF frames that means waiting for the
                # next burst, adding up to a full serial timeout of latency.  When
                # nothing is buffered, read(1) blocks on the first byte instead of
                # spinning, so CPU stays low.
                waiting = ser.in_waiting
                chunk = ser.read(waiting if waiting else 1)
                if not chunk:
                    continue
                if self.raw_debug:
                    logger.debug("raw bytes: %s", chunk.hex())
                buf.extend(chunk)
                self._parse_buffer(buf)
            except Exception as exc:  # noqa: BLE001
                logger.error("reader error: %s", exc)
                break

        logger.debug("end")

    def _parse_buffer(self, buf: bytearray) -> None:
        """Split buffered wire bytes into CAN frames and publish each one.

        Robustness matters more here than speed.  A single dropped or extra byte
        desynchronises the stream, and every frame parsed while desynchronised
        is garbage that still looks plausible.  So a candidate header is only
        accepted when it satisfies the same checks the vendor reference does
        (header mask set, DLC within 0..8, correct trailer at the expected
        offset); anything else is skipped one byte at a time until a real frame
        start turns up again.
        """
        while True:
            idx = buf.find(_HEADER_BYTES)
            if idx == -1:
                del buf[:]
                return
            if idx > 0:
                del buf[:idx]
            if len(buf) < 5:
                return

            type_byte = buf[1]
            if type_byte & HEADER_MARKER != HEADER_MARKER:
                # Not a frame header.  Worth seeing which value this adapter
                # really emits before tightening this check: log it once per
                # distinct value instead of dropping frames silently.
                self._note_type_byte(type_byte)
                del buf[0]
                continue

            is_extended = bool(type_byte & HEADER_EXTENDED)
            is_remote = bool(type_byte & HEADER_REMOTE)
            dlc = type_byte & HEADER_DLC
            if dlc > MAX_DLC:
                # A DLC above 8 cannot be real (CAN FD never reaches this
                # adapter), and computing a length from it would swallow the
                # bytes of the frames that follow.
                del buf[0]
                continue

            id_bytes = 4 if is_extended else 2
            total_len = 2 + id_bytes + dlc + 1
            if len(buf) < total_len:
                return
            if buf[total_len - 1] != TRAILER:
                self._resyncs += 1
                if self._resyncs % 100 == 1:
                    logger.warning(
                        "waveshare stream desynchronised, %d bad frame(s) skipped",
                        self._resyncs,
                    )
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

    def _note_type_byte(self, type_byte: int) -> None:
        """Log a rejected type byte once per distinct value.

        If the adapter's real headers do not satisfy the mask check this tells
        us exactly what it sends, instead of leaving us with a silent bus.
        """
        if type_byte in self._rejected_type_bytes:
            return
        self._rejected_type_bytes.add(type_byte)
        logger.info(
            "waveshare rejected type byte 0x%02X (header marker 0x%02X not set) -- "
            "adapter may use a different frame header format",
            type_byte,
            HEADER_MARKER,
        )

    def _close(self) -> None:
        self._running = False
        if self._reader_thread and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=2.0)
        self._reader_thread = None
        if self._ser and self._ser.is_open:
            try:
                self._ser.close()
            except Exception as exc:  # noqa: BLE001
                logger.warning("failed to close serial port: %s", exc)
        self._ser = None

    @staticmethod
    def _make_message(
        can_id: int, data: bytes = b"", is_remote: bool = False
    ) -> WaveShareMessage:
        """Build a message that already carries its adapter bytes."""
        return WaveShareMessage(can_id, data, is_remote)

    def _send_message(self, msg: WaveShareMessage) -> None:
        """Write a pre-encoded WaveShare message to the port."""
        if self._ser is None or not self._ser.is_open:
            raise CANBusError("bus not open")
        binary = msg.binary

        try:
            with self._lock:
                self._ser.write(binary)
                self._ser.flush()
        except Exception as exc:
            raise CANBusError(f"send 0x{msg.arbitration_id:X} failed: {exc}") from exc
        logger.debug("_send_message aw bytes: %s", binary.hex())
