#!/usr/bin/env python3
"""
Unified ROS 1 Driver Node for Broadcom AFBR-S50 ToF Sensors.
Dynamically handles 1D (Range message) vs 3D Matrix (PointCloud2 message) configurations.
"""

import rospy
import serial
import time
import struct
import math

from sensor_msgs.msg import Range, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg

# Safe local fallback if sci_status.py isn't copied directly alongside this node
try:
    from sci_status import StatusSCI
except ImportError:
    class StatusSCI:
        def __init__(self, code): self.value = code
        @classmethod
        def from_code(cls, code): return cls(code)
        def to_string(self): return f"SCI Status Code: {self.value}"

class SCI_Commands:
    """SCI Protocol Command Codes"""
    LOG_MESSAGE = 0x06

    # Measurement Data Types (Base)
    MEAS_DATA_FULL = 0x32
    MEAS_DATA_3D   = 0x34
    MEAS_DATA_1D   = 0x36
    
    # Bit masks for Measurement Data parsing
    MEAS_DEBUG_BIT = 0x01
    MEAS_TYPE_MASK = 0x7F
    
    # Response mask (Device to Host commands have MSB set)
    RESPONSE_MASK  = 0x80

    # Calculated RX Measurement Commands
    RX_MEAS_FULL_DEBUG = MEAS_DATA_FULL | RESPONSE_MASK | MEAS_DEBUG_BIT # 0xB1
    RX_MEAS_FULL       = MEAS_DATA_FULL | RESPONSE_MASK                  # 0xB2
    RX_MEAS_3D_DEBUG   = MEAS_DATA_3D   | RESPONSE_MASK | MEAS_DEBUG_BIT # 0xB3
    RX_MEAS_3D         = MEAS_DATA_3D   | RESPONSE_MASK                  # 0xB4
    RX_MEAS_1D_DEBUG   = MEAS_DATA_1D   | RESPONSE_MASK | MEAS_DEBUG_BIT # 0xB5
    RX_MEAS_1D         = MEAS_DATA_1D   | RESPONSE_MASK                  # 0xB6
    
    RX_MEAS_ALL = (
        RX_MEAS_FULL_DEBUG, RX_MEAS_FULL,
        RX_MEAS_3D_DEBUG, RX_MEAS_3D,
        RX_MEAS_1D_DEBUG, RX_MEAS_1D
    )

class FrameParser:
    def __init__(self, rx: bytearray):
        self.rx = rx
        self.cursor = 3  # Skip START, CMD, and size bytes
        self.d = {}

    def read_u8(self):
        v = self.rx[self.cursor]
        self.cursor += 1
        return v

    def read_s8(self):
        v = self.read_u8()
        return v - 0x100 if v >= 0x80 else v

    def read_u16(self):
        v = (self.rx[self.cursor] << 8) | self.rx[self.cursor+1]
        self.cursor += 2
        return v

    def read_s16(self):
        v = self.read_u16()
        return v - 0x10000 if v >= 0x8000 else v

    def read_u24(self):
        v = (self.rx[self.cursor] << 16) | (self.rx[self.cursor+1] << 8) | self.rx[self.cursor+2]
        self.cursor += 3
        return v

    def read_s24(self):
        v = self.read_u24()
        return v - 0x1000000 if v >= 0x800000 else v

    def read_u32(self):
        v = (self.rx[self.cursor] << 24) | (self.rx[self.cursor+1] << 16) | (self.rx[self.cursor+2] << 8) | self.rx[self.cursor+3]
        self.cursor += 4
        return v

    def read_s32(self):
        v = self.read_u32()
        return v - 0x100000000 if v >= 0x80000000 else v

    @staticmethod
    def popcount(x):
        return bin(x).count('1')

    def parse(self, cmd: int) -> dict:
        try:
            type_val = cmd & SCI_Commands.MEAS_TYPE_MASK
            debug = bool(type_val & SCI_Commands.MEAS_DEBUG_BIT)
            base_type = type_val + 1 if debug else type_val
            is_1d = (base_type == SCI_Commands.MEAS_DATA_1D)
            is_3d = (base_type == SCI_Commands.MEAS_DATA_3D)
            is_full = (base_type == SCI_Commands.MEAS_DATA_FULL)

            # 1. Generic Block
            raw_status = self.read_s16()
            status_obj = StatusSCI.from_code(raw_status)
            self.d["status"] = status_obj.value
            self.d["status_desc"] = status_obj.to_string()
            
            t_sec = self.read_u32()
            t_usec = self.read_u16()
            self.d["timestamp"] = t_sec + t_usec * 16.0 / 1.0e6

            # 2. FrameConfig Block
            self.d["frame_state"] = self.read_u32()

            px_en_mask = 0
            ch_en_mask = 0
            if not is_1d or debug:
                self.d["dig_int_depth"] = self.read_u16()
                self.d["ana_int_depth"] = self.read_u16()
                self.d["output_power"] = self.read_u16()
                self.d["pixel_gain"] = self.read_u8()
                px_en_mask = self.read_u32()
                self.d["px_en_mask"] = px_en_mask

            if not is_1d:
                ch_en_mask = self.read_u32()
                self.d["ch_en_mask"] = ch_en_mask
            elif debug:
                self.d["bin_msk"] = self.read_u32()
                self.d["sat_msk"] = self.read_u32()

            # 3. RawData Block
            if is_full and debug:
                phase_count = self.read_u8()
                num_raw_ch = self.popcount(px_en_mask) + self.popcount(ch_en_mask)
                raw_samples = []
                for _ in range(num_raw_ch):
                    ch_samples = []
                    for _ in range(phase_count):
                        ch_samples.append(self.read_u24())
                    raw_samples.append(ch_samples)
                self.d["raw_samples"] = raw_samples

            # 4. 3D Matrix Payload Decoding
            if not is_1d:
                footer_len = 2 # CRC + STOP
                if debug: footer_len += 57
                if debug or is_full: footer_len += 14
                if is_full: footer_len += 6
                data_bytes = len(self.rx) - self.cursor - footer_len
                num_pixels = data_bytes // 6
                idx_status = self.cursor
                idx_range = idx_status + num_pixels * 1
                idx_amp = idx_range + num_pixels * 3
                active_px_ids = [n for n in range(32) if (px_en_mask & (1 << n))]
                if num_pixels > len(active_px_ids):
                    active_px_ids.append("Ref")
                
                pixels = []
                for i in range(num_pixels):
                    px_status = self.rx[idx_status + i]
                    r_raw = (self.rx[idx_range + i*3] << 16) | (self.rx[idx_range + i*3 + 1] << 8) | self.rx[idx_range + i*3 + 2]
                    r_val = r_raw - 0x1000000 if r_raw >= 0x800000 else r_raw
                    r_m = r_val / 16384.0
                    a_raw = (self.rx[idx_amp + i*2] << 8) | self.rx[idx_amp + i*2 + 1]
                    a_val = a_raw / 16.0
                    px_id = active_px_ids[i] if i < len(active_px_ids) else f"Ext{i}"
                    pixels.append({"id": px_id, "status": px_status, "range": r_m, "amplitude": a_val})
                self.d["pixels"] = pixels
                self.cursor += num_pixels * 6

            # 5. 1D Block Extraction
            if is_1d or is_full:
                r_val = self.read_s24()
                self.d["range"] = r_val / 16384.0
                self.d["amplitude"] = self.read_u16() / 16.0
                self.d["signal_quality"] = self.read_u8()

            return self.d
        except Exception as e:
            rospy.logwarn(f"Frame parsing exception: {e}")
            return None

class AFBR_S50_UART:
    """
    Broadcom AFBR-S50 UART Communication Layer.
    Supports SCI command framing, byte stuffing, and CRC8 calculations.
    """
    START_BYTE = 0x02
    STOP_BYTE  = 0x03
    ESC_BYTE   = 0x1B
    
    CMD_ACK    = 0x0A
    CMD_NAK    = 0x0B
    
    CMD_SET_OUTPUT_MODE    = 0x41
    CMD_SET_FRAME_TIME     = 0x43
    CMD_START_MEASUREMENT  = 0x11
    CMD_STOP_MEASUREMENT   = 0x12
    
    OUTPUT_MODES = {
        "1d": 0x07, "1d_debug": 0x06, 
        "3d": 0x05, "3d_debug": 0x04, 
        "full": 0x03, "full_debug": 0x02
    }

    def __init__(self, port: str, baudrate: int, timeout: float):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        if self.ser.in_waiting > 0:
            self.ser.read(self.ser.in_waiting)

    @staticmethod
    def calc_crc8_sae_j1850(data: bytes) -> int:
        crc = 0x00
        for byte in data:
            crc ^= byte
            for _ in range(8):
                crc = (crc << 1) ^ 0x1D if crc & 0x80 else crc << 1
            crc &= 0xFF
        return crc

    def _remove_byte_stuffing(self, rx_frame: bytearray) -> bytearray:
        parts = rx_frame.split(bytes([self.ESC_BYTE]))
        clean_data = bytearray(parts[0])
        for part in parts[1:]:
            if len(part) > 0:
                part[0] ^= 0xFF
                clean_data.extend(part)
        return clean_data

    def send_command(self, cmd_id: int, param_bytes: bytes = b"") -> bool:
        raw_payload = bytes([cmd_id]) + param_bytes
        crc = self.calc_crc8_sae_j1850(raw_payload)
        
        # Apply byte stuffing payload assembly
        escaped = bytearray()
        for b in raw_payload + bytes([crc]):
            if b in (self.START_BYTE, self.STOP_BYTE, self.ESC_BYTE):
                escaped.append(self.ESC_BYTE)
                escaped.append(b ^ 0xFF)
            else:
                escaped.append(b)
                
        tx_frame = bytes([self.START_BYTE]) + escaped + bytes([self.STOP_BYTE])
        self.ser.write(tx_frame)
        self.ser.flush()
        return self.__wait_for_ack(cmd_id)

    def __wait_for_ack(self, expected_cmd: int) -> bool:
        start_time = time.time()
        while (time.time() - start_time) < 1.0:
            if self.ser.in_waiting > 0:
                rx = bytearray(self.ser.read_until(bytes([self.STOP_BYTE])))
                if len(rx) == 0 or rx[0] != self.START_BYTE or rx[-1] != self.STOP_BYTE:
                    continue
                rx_clean = self._remove_byte_stuffing(rx)
                if rx_clean[1] == self.CMD_ACK and rx_clean[2] == expected_cmd:
                    return True
                elif rx_clean[1] == self.CMD_NAK and rx_clean[2] == expected_cmd:
                    return False
        return False

    def setup_sensor(self, mode: str, fps: float) -> bool:
        if mode.lower() not in self.OUTPUT_MODES:
            rospy.logerr(f"Invalid mode parameter string: {mode}")
            return False
        if not self.send_command(self.CMD_SET_OUTPUT_MODE, bytes([self.OUTPUT_MODES[mode.lower()]])):
            return False
        # Calculate dynamic frame timing microseconds
        usec = int(1_000_000 / fps)
        if not self.send_command(self.CMD_SET_FRAME_TIME, struct.pack('>I', usec)):
            return False
        return self.send_command(self.CMD_START_MEASUREMENT, b"")

    def stop(self):
        try:
            raw_cmd = bytes([self.CMD_STOP_MEASUREMENT])
            crc = self.calc_crc8_sae_j1850(raw_cmd)
            self.ser.write(bytes([self.START_BYTE, self.CMD_STOP_MEASUREMENT, crc, self.STOP_BYTE]))
            self.ser.flush()
        except: pass
        finally: self.ser.close()

def main():
    rospy.init_node('afbr_s50_driver', anonymous=False)
    node_name = rospy.get_name()

    # 1. Load configuration parameters from ROS master parameter stack
    port = rospy.get_param('~port', '/dev/ttyAMA0')
    baud = rospy.get_param('~baud', 115200)
    timeout = rospy.get_param('~timeout', 1.0)
    fps = rospy.get_param('~fps', 10.0)
    mode = rospy.get_param('~mode', '1d').lower()
    frame_id = rospy.get_param('~frame_id', 'tof_link')

    # 2. Determine topics based on selected target mode architecture
    is_3d_mode = mode.startswith('3d') or mode.startswith('full')
    if is_3d_mode:
        pub = rospy.Publisher('~pointcloud', PointCloud2, queue_size=10)
        rospy.loginfo(f"[{node_name}] Initialized in 3D Mode -> Publishing on ~pointcloud")
    else:
        pub = rospy.Publisher('~range', Range, queue_size=10)
        rospy.loginfo(f"[{node_name}] Initialized in 1D Mode -> Publishing on ~range")

    # Wide FoV Angular Grid Constants (8 Columns x 4 Rows)
    TOTAL_COLS, TOTAL_ROWS = 8, 4
    # H_FOV, V_FOV = math.radians(12.4), math.radians(5.4) # Update to match your exact variant model sheet
    H_FOV, V_FOV = math.radians(2.0), math.radians(2.0)     # S50LV85D
    H_STEP, V_STEP = H_FOV / TOTAL_COLS, V_FOV / TOTAL_ROWS

    # 3. Connection and sensor setup
    try:
        sensor = AFBR_S50_UART(port, baud, timeout)
    except Exception as e:
        rospy.logerr(f"[{node_name}] Failed to establish physical serial link on {port}: {e}")
        return

    if not sensor.setup_sensor(mode, fps):
        rospy.logerr(f"[{node_name}] Hardware rejected requested parameters [Mode: {mode}, FPS: {fps}]")
        sensor.stop()
        return

    # Ultra fast polling loop rate to continuously consume hardware registers
    loop_rate = rospy.Rate(500)

    while not rospy.is_shutdown():
        try:
            if sensor.ser.in_waiting > 0:
                rx = bytearray(sensor.ser.read_until(bytes([sensor.STOP_BYTE])))
                if len(rx) > 0 and rx[0] == sensor.START_BYTE and rx[-1] == sensor.STOP_BYTE:
                    rx_clean = sensor._remove_byte_stuffing(rx)
                    cmd = rx_clean[1]
                    
                    if cmd in SCI_Commands.RX_MEAS_ALL:
                        data = FrameParser(rx_clean).parse(cmd)
                        if not data:
                            continue
                        
                        # --- OUTPUT BRANCH 1: POINT CLOUD (3D MODE) ---
                        if is_3d_mode and "pixels" in data:
                            cloud_points = []
                            for idx, p in enumerate(data['pixels']):
                                if p['status'] != 0: 
                                    continue # Omit tracking errors or saturation noise
                                r = p['range']
                                
                                # Matrix coordinate row-column calculations
                                col = idx % TOTAL_COLS
                                row = idx // TOTAL_COLS
                                
                                # Projection angle extraction relative to optical center
                                azimuth = (col - (TOTAL_COLS - 1) / 2.0) * H_STEP
                                elevation = (row - (TOTAL_ROWS - 1) / 2.0) * V_STEP
                                
                                # Spherical coordinates to ROS FLU Cartesian Space Projection
                                x = r * math.cos(elevation) * math.cos(azimuth)
                                y = r * math.cos(elevation) * math.sin(azimuth)
                                z = r * math.sin(elevation)
                                cloud_points.append([x, y, z])
                            
                            header = std_msgs.msg.Header()
                            header.stamp = rospy.Time.now()
                            header.frame_id = frame_id
                            cloud_msg = pc2.create_cloud_xyz32(header, cloud_points)
                            pub.publish(cloud_msg)

                        # --- OUTPUT BRANCH 2: RANGE MESSAGE (1D MODE) ---
                        elif not is_3d_mode:
                            msg = Range()
                            msg.header.stamp = rospy.Time.now()
                            msg.header.frame_id = frame_id
                            msg.radiation_type = Range.INFRARED
                            msg.field_of_view = H_FOV
                            msg.min_range = 0.05
                            # msg.max_range = 15.0
                            msg.max_range = 30.0    # S50LV85D
                            # Extract singular range tracker reading safely
                            msg.range = data.get('range', 0.0)
                            pub.publish(msg)

            loop_rate.sleep()
        except rospy.ROSInterruptException:
            break
        except Exception as loop_err:
            rospy.logerr(f"[{node_name}] Active processing fault: {loop_err}")

    sensor.stop()

if __name__ == '__main__':
    main()