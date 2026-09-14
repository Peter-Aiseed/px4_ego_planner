#!/usr/bin/env python3
"""
WaveShare USB-CAN-A Test Script
手動測試腳本：發送 Remote Frame 啟動/停止 AFBR-S50 量測
"""

import serial
import time

# 配置
PORT = "COM12"
BAUDRATE = 2_000_000
CAN_BITRATE = 1_000_000


def build_config_packet(bitrate: int) -> bytes:
    """Build 20-byte configuration packet for WaveShare USB-CAN-A"""
    baud_map = {
        5000000: 0x00,
        1000000: 0x01,
        800000: 0x02,
        500000: 0x03,
        400000: 0x04,
        250000: 0x05,
        200000: 0x06,
        125000: 0x07,
        100000: 0x08,
        50000: 0x09,
        20000: 0x0A,
        10000: 0x0B,
        5000: 0x0C,
    }
    baud_code = baud_map.get(bitrate)
    if baud_code is None:
        raise ValueError(f"Unsupported CAN bitrate: {bitrate}")

    pkt = bytearray(20)
    pkt[0] = 0xAA
    pkt[1] = 0x55
    pkt[2] = 0x12  # Variable length protocol
    pkt[3] = baud_code
    pkt[4] = 0x01  # Standard frame
    pkt[5:13] = b"\x00\x00\x00\x00\x00\x00\x00\x00"  # Filter ID (accept all)
    pkt[13:19] = b"\x00\x00\x00\x00\x00\x00"  # Mask ID (accept all)
    pkt[14] = 0x00  # Normal mode
    pkt[15] = 0x00  # Auto retransmit
    pkt[16:19] = b"\x00\x00\x00"  # Reserved
    checksum = sum(pkt[2:19]) & 0xFF
    pkt[19] = checksum
    return bytes(pkt)


def build_remote_frame(can_id: int) -> bytes:
    """Build Remote Frame packet for WaveShare USB-CAN-A"""
    pkt = bytearray()
    pkt.append(0xAA)  # Header
    pkt.append(0xD0)  # Type: Standard frame (bit5=0), Remote (bit4=1), DLC=0
    pkt.append(can_id & 0xFF)  # ID low byte
    pkt.append((can_id >> 8) & 0xFF)  # ID high byte
    pkt.append(0x55)  # End code
    return bytes(pkt)


def main() -> None:
    print(f"Opening {PORT} at {BAUDRATE} baud...")
    ser = serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.1,
    )

    try:
        # Configure CAN
        print(f"Configuring CAN bus: {CAN_BITRATE} bps, standard frames, normal mode")
        config_pkt = build_config_packet(CAN_BITRATE)
        ser.write(config_pkt)
        ser.flush()
        time.sleep(0.1)
        print(f"Config sent: {' '.join(f'{b:02X}' for b in config_pkt)}")

        # Send Start Remote Frame (ID 0x08)
        print("\nSending START Remote Frame (ID=0x08)...")
        start_pkt = build_remote_frame(0x08)
        ser.write(start_pkt)
        ser.flush()
        print(f"Sent: {' '.join(f'{b:02X}' for b in start_pkt)}")

        # Listen for incoming frames
        print("\nListening for CAN frames (press Ctrl+C to stop)...")
        print("Expected: ID=0x1C, DLC=8 (1D measurement data)\n")
        buf = bytearray()
        frame_count = 0
        while True:
            data = ser.read(256)
            if data:
                buf.extend(data)
                # Parse frames
                while True:
                    idx = buf.find(b"\xAA")
                    if idx == -1:
                        del buf[:]
                        break
                    if idx > 0:
                        del buf[:idx]

                    if len(buf) < 5:
                        break

                    type_byte = buf[1]
                    is_extended = bool(type_byte & 0x20)
                    is_remote = bool(type_byte & 0x10)
                    dlc = type_byte & 0x0F

                    id_bytes = 4 if is_extended else 2
                    total_len = 2 + 1 + id_bytes + dlc + 1

                    if len(buf) < total_len:
                        break

                    if buf[total_len - 1] != 0x55:
                        del buf[0]
                        continue

                    if is_extended:
                        can_id = int.from_bytes(buf[2:6], byteorder="little")
                        data_bytes = bytes(buf[6:6 + dlc])
                    else:
                        can_id = int.from_bytes(buf[2:4], byteorder="little")
                        data_bytes = bytes(buf[4:4 + dlc])

                    frame_count += 1
                    print(
                        f"[{frame_count}] ID=0x{can_id:03X} "
                        f"Type={'REMOTE' if is_remote else 'DATA'} "
                        f"DLC={dlc} "
                        f"Data={data_bytes.hex()}"
                    )

                    del buf[:total_len]

    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        ser.close()
        print("Serial port closed")


if __name__ == "__main__":
    main()
