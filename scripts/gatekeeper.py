#!/usr/bin/env python3
import os
os.environ['MAVLINK20'] = '1'

import rospy
from pymavlink import mavutil
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler
import sys
import time
import math

# Color Codes
GREEN = '\033[92m'
YELLOW = '\033[93m'
RED = '\033[91m'
BLUE = '\033[94m'
CYAN = '\033[96m'
BOLD = '\033[1m'
ENDC = '\033[0m'

# ==========================================
#  PX4 MAVLINK CONSTANTS (Pure Uppercase)
# ==========================================
# You calculate these manually once using your bitshift rules:
PX4_OFFBOARD = 393216          # (6 << 16)
PX4_AUTO_MISSION = 67371008    # (4 << 16) | (4 << 24)
PX4_AUTO_LOITER = 50593792     # (4 << 16) | (3 << 24)

class MissionItem:
    def __init__(self):
        self.seq = None
        self.command = None
        self.frame = None

        self.param1 = None
        self.param2 = None
        self.param3 = None
        self.param4 = None

        self.x = 0
        self.y = 0
        self.z = 0
    
    def __str__(self):

        cmd_map = {
            16: "WAYPOINT",
            20: "RTL",
            21: "LAND",
            22: "TAKEOFF"
        }

        frame_map = {
            0: "GLOBAL",
            2: "MISSION",
            3: "GLOBAL_RELATIVE_ALT",
            8: "BODY_NED"
        }

        cmd_name = cmd_map.get(self.command, f"UNKNOWN({self.command})")
        frame_name = frame_map.get(self.frame, f"FRAME({self.frame})")

        return (
            f"MissionItem(\n"
            f"  seq      = {self.seq}\n"
            f"  command  = {cmd_name}\n"
            f"  frame    = {frame_name}\n"
            f"  x        = {self.x:.6f}\n"
            f"  y        = {self.y:.6f}\n"
            f"  z        = {self.z:.2f}\n"
            f"  param1   = {self.param1}\n"
            f"  param2   = {self.param2}\n"
            f"  param3   = {self.param3}\n"
            f"  param4   = {self.param4}\n"
            f")"
        )

class MasterGatekeeper:
    def __init__(self):
        # ROS Node and Publisher/Subscriber
        rospy.init_node('master_gatekeeper')
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        
        # State Variables
        self.local_position = None  # (x, y, z) in meters relative to home
        self.local_position_yaw = None
        self.home_lat = None        # Global Position Latitude
        self.home_lon = None        # Global Position Longitude
        self.home_alt = None        # Global Position Relative Altitude in meters
        self.current_mode = "Initial"
        self.planner_goal = None
        
        # Mission Stats
        self.mission = []
        self.mission_seq = 0
        self.mission_total = 65535
        self.holding = False
        self.mission_hold_time = rospy.Time.now()

        self.fence = []
        self.fence_total = 0
        
        self.rally = []
        self.rally_total = 0

        try:
            # 14550 = Drone
            self.px4_conn = mavutil.mavlink_connection('udpin:127.0.0.1:14550')
            self.px4_conn.mav.srcSystem = 255
            self.px4_conn.mav.srcComponent = 190
            # 14590 = QGC
            self.qgc_conn = mavutil.mavlink_connection('udpin:127.0.0.1:14590')
            self.qgc_conn.mav.srcSystem = 1
            self.qgc_conn.mav.srcComponent = 1
            
            print(f"{GREEN}--- Gatekeeper Initialize: PX4 <--> QGC ---{ENDC}\n")
        except Exception as e:
            rospy.logerr(f"Link Error: {e}"); sys.exit(1)
    
    # -------------------------------------------------------------------
    #                         Helper Functions
    # -------------------------------------------------------------------
    
    def get_local_coords(self, target_lat, target_lon, target_alt):
        """Converts GPS to Local Meters"""
        if self.home_lat is None:
            return None
        
        # Y = North, X = East
        y = (target_lat - self.home_lat) * 111320.0
        x = (target_lon - self.home_lon) * 111320.0 * math.cos(math.radians(self.home_lat))
        z = target_alt - self.home_alt
        return x, y, z

    def get_px4_mode_names(self, custom_mode):
        # Bit-shift to get the bytes
        main_mode = (custom_mode >> 16) & 0xFF
        sub_mode = (custom_mode >> 24) & 0xFF

        # PX4 Main Mode Constants
        main_modes = {
            1: "MANUAL",
            2: "ALTCTL",
            3: "POSCTL",
            4: "AUTO",
            6: "OFFBOARD",
            7: "STABILIZED",
            8: "RATTITUDE"
        }

        # PX4 Auto Sub-Mode Constants (Only relevant if main_mode == 4)
        sub_modes = {
            1: "READY",
            2: "TAKEOFF",
            3: "LOITER",
            4: "MISSION",
            5: "RTL",
            6: "LAND",
            7: "FOLLOW_ME"
        }

        main_name = main_modes.get(main_mode, f"UNKNOWN({main_mode})")
        
        if main_mode == 4: # If it's an AUTO mode
            sub_name = sub_modes.get(sub_mode, f"SUB({sub_mode})")
            return f"AUTO.{sub_name}"
        
        return main_name
    
    # -------------------------------------------------------------------
    #                       Fake Message Functions
    # -------------------------------------------------------------------
    
    def send_fake_ack(self, command):
        self.qgc_conn.mav.command_ack_send(
            command,            # The command being acknowledged (e.g., 192)
            0,                  # Result: MAV_RESULT_ACCEPTED (0)
            0,                  # progress: 0 (putting 0 is fine since QGC doesn't use it for COMMAND_ACK)
            0,                  # result_param2 (optional, can be 0)
            255,                # target_system (echo back to QGC)
            190                 # target_component (echo back to QGC)
        )
    
    def send_fake_mission_request_int(self, seq=0, mission_type=0):
        self.qgc_conn.mav.mission_request_int_send(
            255,                # target_system (echo back to QGC)
            190,                # target_component (echo back to QGC)
            seq,                # seq of the requested mission item
            mission_type        # mission_type (0 for main mission items)
        )
        print(f"{GREEN}MISSION_REQUEST_INT [target_system : 255, target_component : 190, seq : {seq}, mission_type : {mission_type}]{ENDC}")
    
    def send_fake_mission_ack(self, mission_type=0):
        self.qgc_conn.mav.mission_ack_send(
            255,                # target_system (echo back to QGC)
            190,                # target_component (echo back to QGC)
            0,                  # result: MAV_MISSION_ACCEPTED (0) or other error codes
            mission_type        # mission_type (0 for main mission items)
        )
        print(f"{GREEN}MISSION_ACK [target_system : 255, target_component : 190, type : 0, mission_type : {mission_type}]{ENDC}")

    # -------------------------------------------------------------------
    #                         Planner Functions
    # -------------------------------------------------------------------

    def planner_finish(self):
        # Check if the drone is within the accepted radius of the target
        if self.local_position is None:
            return False
        
        dx = self.local_position[0] - self.planner_goal[0]
        dy = self.local_position[1] - self.planner_goal[1]
        dz = self.local_position[2] - self.planner_goal[2]
        distance = math.sqrt(dx**2 + dy**2 + dz**2)

        if distance < 0.5:
            return True
        else:
            return False
    
    def call_planner(self, goal_x, goal_y, goal_z, yaw=0.0, sending_ack=False, command=None):
        # Implementation for calling the planner
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "world"
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.position.z = goal_z
        goal.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, yaw))
        self.goal_pub.publish(goal)
        if sending_ack:
            self.send_fake_ack(command)

    # -------------------------------------------------------------------
    #                         Mission Functions
    # -------------------------------------------------------------------
    
    def do_mission(self):
        goal = self.mission[self.mission_seq]
        command = goal.command

        if command in [16, 20]:
            if not math.isnan(goal.param4):
                self.call_planner(goal.x, goal.y, goal.z, math.radians(goal.param4))
            else:
                self.call_planner(goal.x, goal.y, goal.z)
        
        elif command == 22:
            if not math.isnan(goal.param4):
                self.call_planner(goal.x, goal.y, goal.z, math.radians(goal.param4))
            else:
                self.call_planner(goal.x, goal.y, goal.z)
    
    def mission_finish(self):
        # Check if the drone is within the accepted radius of the target
        if self.local_position is None:
            return False
        
        dx = self.local_position[0] - self.mission[self.mission_seq].x
        dy = self.local_position[1] - self.mission[self.mission_seq].y
        dz = self.local_position[2] - self.mission[self.mission_seq].z
        distance = math.sqrt(dx**2 + dy**2 + dz**2)

        range_detected = False
        accept_radius = self.mission[self.mission_seq].param2
        if accept_radius == 0.0 or math.isnan(accept_radius):
            if distance < 0.5:
                range_detected = True
            else:
                return False
        else:
            if distance < accept_radius:
                range_detected = True
            else:
                return False

        yaw_detected = True
        if not math.isnan(self.mission[self.mission_seq].param4):
            target_yaw = math.radians(self.mission[self.mission_seq].param4)
            yaw_error = abs(target_yaw - self.local_position_yaw)
            if yaw_error > math.pi:
                yaw_error = 2 * math.pi - yaw_error
            if yaw_error > 0.2:
                yaw_detected = False

        if range_detected and yaw_detected:
            return True
        return False
    
    def handle_mission_message(self, msg):
        q_type = msg.get_type()

        # 1) MISSION_COUNT
        if q_type == "MISSION_COUNT":
            if msg.mission_type == 0:
                rospy.loginfo(f"{GREEN}[MISSION] Count received: {msg.count}{ENDC}")
                if msg.count > 0:
                    self.mission_total = msg.count
                    self.send_fake_mission_request_int(0)
                else:
                    self.mission_total = 65535
                    self.send_fake_mission_ack()
                self.mission = []   # Clear previous mission items when a new MISSION_COUNT is received
                self.mission_seq = 0
            
            elif msg.mission_type == 1:
                rospy.loginfo(f"{GREEN}[FENCE] Count received: {msg.count}{ENDC}")
                if msg.count > 0:
                    self.fence_total = msg.count
                    self.send_fake_mission_request_int(0, mission_type=1)
                else:
                    self.fence_total = 0
                    self.send_fake_mission_ack(mission_type=1)
                self.fence = []   # Clear previous mission items when a new MISSION_COUNT is received
            
            elif msg.mission_type == 2:
                rospy.loginfo(f"{GREEN}[Rally] Count received: {msg.count}{ENDC}")
                if msg.count > 0:
                    self.rally_total = msg.count
                    self.send_fake_mission_request_int(0, mission_type=2)
                else:
                    self.rally_total = 0
                    self.send_fake_mission_ack(mission_type=2)
                self.rally = []   # Clear previous mission items when a new MISSION_COUNT is received

        # 2) MISSION_ITEM_INT
        elif q_type == "MISSION_ITEM_INT":

            item = MissionItem()

            item.seq = msg.seq
            item.command = msg.command
            item.frame = msg.frame

            if msg.command == 20:
                item.x = 0.0
                item.y = 0.0
                item.z = 5.0
                item.param1 = 0.0
                item.param2 = 0.0
                item.param3 = 0.0
                item.param4 = float('nan')

            else:
                item.param1 = msg.param1
                item.param2 = msg.param2
                item.param3 = msg.param3
                item.param4 = float('nan') # ego-planner cannot support yaw control for now, so we set it to NaN to indicate "ignore yaw" in the planner

                # MAVLink INT format → meters conversion
                if msg.frame == 3:  # If it's a global frame, convert to local meters
                    item.x, item.y, item.z = self.get_local_coords(msg.x / 1e7, msg.y / 1e7, msg.z + self.home_alt)
                else:
                    item.x = msg.x
                    item.y = msg.y
                    item.z = msg.z

            if msg.mission_type == 0:
                self.mission.append(item)
                rospy.loginfo(f"{GREEN}[MISSION] Received item {item.seq}, cmd={item.command}{ENDC}")
                print(item)
                if not item.seq == self.mission_total - 1:
                    self.send_fake_mission_request_int(msg.seq + 1)
                else:
                    self.send_fake_mission_ack()

            elif msg.mission_type == 1:
                self.fence.append(item)
                rospy.loginfo(f"{GREEN}[FENCE] Received item {item.seq}, cmd={item.command}{ENDC}")
                print(item)
                if not item.seq == self.fence_total - 1:
                    self.send_fake_mission_request_int(msg.seq + 1, mission_type=1)
                else:
                    self.send_fake_mission_ack(mission_type=1)
            
            elif msg.mission_type == 2:
                self.rally.append(item)
                rospy.loginfo(f"{GREEN}[Rally] Received item {item.seq}, cmd={item.command}{ENDC}")
                print(item)
                if not item.seq == self.rally_total - 1:
                    self.send_fake_mission_request_int(msg.seq + 1, mission_type=2)
                else:
                    self.send_fake_mission_ack(mission_type=2)
        
        # 3) MISSION_CLEAR_ALL
        elif q_type == "MISSION_CLEAR_ALL":
            if msg.mission_type == 0:
                self.mission = []
                self.mission_seq = 0
                self.mission_total = 65535
                self.send_fake_mission_ack()
            
            elif msg.mission_type == 1:
                self.fence = []
                self.fence_total = 0
                self.send_fake_mission_ack(mission_type=1)
            
            elif msg.mission_type == 2:
                self.rally = []
                self.rally_total = 0
                self.send_fake_mission_ack(mission_type=2)

    # -------------------------------------------------------------------

    def run(self):
        while not rospy.is_shutdown():
            # ************************************************************************************************
            # **                                                                                            **
            # **                                 MISSION/OFFBOARD MODE LOOP                                 **
            # **                                                                                            **
            # ************************************************************************************************
            if "MISSION" in self.current_mode:
                if self.mission_total == 65535:
                    self.px4_conn.mav.command_long_send(
                        1,  # target system
                        1,  # target component
                        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                        0,
                        1,  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                        4,  # PX4_CUSTOM_MAIN_MODE_AUTO
                        3,  # PX4_CUSTOM_SUB_MODE_LOITER
                        0, 0, 0, 0
                    )
                    self.current_mode = self.get_px4_mode_names(PX4_AUTO_LOITER)

                elif self.holding:
                    if rospy.Time.now() > self.mission_hold_time:
                        self.holding = False
                        rospy.loginfo(f"{CYAN}--- Next Mission No.{self.mission_seq} ---{ENDC}")
                        print(self.mission[self.mission_seq])
                        self.do_mission()
                
                elif self.mission_finish():
                    rospy.loginfo(f"{GREEN}--- Mission No.{self.mission_seq} Finished ---{ENDC}")

                    if self.mission_seq  == self.mission_total - 1:
                        self.holding = False
                        self.px4_conn.mav.command_long_send(
                            1,  # target system
                            1,  # target component
                            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                            0,
                            1,  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                            4,  # PX4_CUSTOM_MAIN_MODE_AUTO
                            3,  # PX4_CUSTOM_SUB_MODE_LOITER
                            0, 0, 0, 0
                        )
                        self.current_mode = self.get_px4_mode_names(PX4_AUTO_LOITER)
                        rospy.loginfo(f"{GREEN}--- Mission Finish All ---{ENDC}")
                    
                    else:
                        self.holding = True
                        self.mission_hold_time = rospy.Time.now() + rospy.Duration(self.mission[self.mission_seq].param1)
                        if rospy.Time.now() < self.mission_hold_time:
                            rospy.loginfo(f"{YELLOW}Holding at Mission No.{self.mission_seq} for {self.mission[self.mission_seq].param1} seconds ...{ENDC}")
                        
                        self.qgc_conn.mav.mission_item_reached_send(self.mission_seq)
                        self.mission_seq += 1
            
            elif "OFFBOARD" in self.current_mode and self.planner_goal is not None:
                if self.planner_finish():
                    self.planner_goal = None
                    self.px4_conn.mav.command_long_send(
                            1,  # target system
                            1,  # target component
                            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                            0,
                            1,  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                            4,  # PX4_CUSTOM_MAIN_MODE_AUTO
                            3,  # PX4_CUSTOM_SUB_MODE_LOITER
                            0, 0, 0, 0
                        )
                    self.current_mode = self.get_px4_mode_names(PX4_AUTO_LOITER)

            # ************************************************************************************************
            # **                                                                                            **
            # **                                    FROM PX4 -> TO QGC                                      **
            # **                                                                                            **
            # ************************************************************************************************
            msg_p = self.px4_conn.recv_match(blocking=False)
            if msg_p:
                p_type = msg_p.get_type()
 
                # -------------------------------------------------------------------
                #                         GLOBAL_POSITION_INT                        
                # -------------------------------------------------------------------
                if p_type == 'GLOBAL_POSITION_INT':
                    if self.home_lat is not None:
                        hdg_deg = msg_p.hdg / 100.0
                        hdg_rad = math.radians(hdg_deg)
                        yaw_enu = math.pi/2 - hdg_rad
                        yaw_enu = math.atan2(math.sin(yaw_enu), math.cos(yaw_enu))
                        self.local_position_yaw = yaw_enu

                    else:
                        self.home_lat = msg_p.lat / 1e7
                        self.home_lon = msg_p.lon / 1e7
                        self.home_alt = msg_p.alt / 1e3
                        rospy.loginfo(f"{CYAN}Home Position Set: Lat {self.home_lat:.6f}, Lon {self.home_lon:.6f}, Alt {self.home_alt:.2f}m{ENDC}")
                
                # -------------------------------------------------------------------
                #                           HOME_POSITION                            
                # -------------------------------------------------------------------
                elif p_type == 'HOME_POSITION':
                    self.home_lat = msg_p.latitude / 1e7
                    self.home_lon = msg_p.longitude / 1e7
                    self.home_alt = msg_p.altitude / 1e3
                
                # -------------------------------------------------------------------
                #                         LOCAL_POSITION_NED                            
                # -------------------------------------------------------------------
                elif p_type == 'LOCAL_POSITION_NED':
                    self.local_position = (msg_p.y, msg_p.x, -msg_p.z)

                # -------------------------------------------------------------------
                #                             COMMAND_ACK                            
                # -------------------------------------------------------------------
                elif p_type == 'COMMAND_ACK':
                    # Intercept ACKs from the px4 and prevent them from reaching QGC
                    rospy.loginfo(f"{YELLOW}{msg_p}{ENDC}")

                # -------------------------------------------------------------------
                #                              MISSION                            
                # -------------------------------------------------------------------
                elif p_type == "MISSION_CURRENT":
                    self.qgc_conn.mav.mission_current_send(self.mission_seq, self.mission_total, 0, 0)
                    continue

                # elif "MISSION" in p_type:
                #     rospy.loginfo(f"{YELLOW}Drone ---> Mission Command: {p_type} Received ---> QGC{ENDC}")
                #     print(msg_p)

                # -------------------------------------------------------------------
                #                              HEARTBEAT                            
                # -------------------------------------------------------------------
                elif p_type == "HEARTBEAT":
                    if msg_p.type == 18 and msg_p.autopilot == 8:
                        continue
                    elif msg_p.type == 2:
                        
                        if self.current_mode == "Initial":
                            self.current_mode = self.get_px4_mode_names(msg_p.custom_mode)
                        
                        if msg_p.custom_mode == PX4_OFFBOARD and "MISSION" in self.current_mode:
                            self.qgc_conn.mav.heartbeat_send(
                                type=2,
                                autopilot=12,
                                base_mode=157,
                                custom_mode = PX4_AUTO_MISSION,
                                system_status=4,
                                mavlink_version=3
                            )
                            continue
                
                self.qgc_conn.write(msg_p.get_msgbuf())

            # ************************************************************************************************
            # **                                                                                            **
            # **                                    FROM QGC -> TO PX4                                      **
            # **                                                                                            **
            # ************************************************************************************************
            msg_q = self.qgc_conn.recv_match(blocking=False)
            if msg_q:
                q_type = msg_q.get_type()

                # -------------------------------------------------------------------
                #                            COMMAND_INT                            
                # -------------------------------------------------------------------
                if q_type == 'COMMAND_INT':

                     # Special handling for Go-To commands from QGC
                    if msg_q.command == 192:
                        self.px4_conn.mav.command_long_send(
                            1,  # target system
                            1,  # target component
                            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                            0,
                            1,  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                            6,  # PX4_CUSTOM_MAIN_MODE_OFFBOARD
                            0,
                            0, 0, 0, 0
                        )
                        local_x, local_y, local_z = self.get_local_coords(msg_q.x / 1e7, msg_q.y / 1e7, msg_q.z)
                        rospy.loginfo(f"{GREEN}GO-TO Clicked! Meters from Home -> X (East): {local_x:.2f}m, Y (North): {local_y:.2f}m, Z (Up): {local_z:.2f}m{ENDC}")
                        self.current_mode = self.get_px4_mode_names(PX4_OFFBOARD)
                        if local_z > 3.0:
                            self.call_planner(local_x, local_y, local_z, sending_ack=True, command=192)
                            self.planner_goal = (local_x, local_y, local_z)
                        
                    else:
                        self.px4_conn.write(msg_q.get_msgbuf())

                # -------------------------------------------------------------------
                #                            COMMAND_LONG                            
                # -------------------------------------------------------------------
                elif q_type == 'COMMAND_LONG':
                    
                    self.px4_conn.write(msg_q.get_msgbuf())

                # -------------------------------------------------------------------
                #                              SET_MODE                            
                # -------------------------------------------------------------------
                elif q_type == 'SET_MODE':
                    mode_name = self.get_px4_mode_names(msg_q.custom_mode)
                    rospy.loginfo(f"{CYAN}Set Mode Command received: Mode ID {msg_q.custom_mode} -> {mode_name}{ENDC}")
                    
                    if "RTL" in mode_name:
                        self.px4_conn.mav.command_long_send(
                            1,  # target system
                            1,  # target component
                            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                            0,
                            1,  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                            6,  # PX4_CUSTOM_MAIN_MODE_OFFBOARD
                            0,
                            0, 0, 0, 0
                        )
                        rospy.loginfo(f"{GREEN}--- RTL Mode Activated ---{ENDC}")
                        self.call_planner(0.0, 0.0, 5.0, sending_ack=True, command=176)
                        self.planner_goal = (0.0, 0.0, 5.0)
                        self.current_mode = self.get_px4_mode_names(PX4_OFFBOARD)
                        continue
                        
                    elif "MISSION" in mode_name:
                        if not self.mission_total == 65535:
                            self.px4_conn.mav.command_long_send(
                                1,  # target system
                                1,  # target component
                                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                0,
                                1,  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                                6,  # PX4_CUSTOM_MAIN_MODE_OFFBOARD
                                0,
                                0, 0, 0, 0
                            )
                            self.do_mission()
                            rospy.loginfo(f"{GREEN}--- Mission Mode Started ---{ENDC}")
                            self.current_mode = mode_name
                            continue

                    # Other Modes that should stop the planner
                    else:
                        if "LAND" in mode_name:
                            rospy.loginfo(f"{RED}--- Landing Mode Activated ---{ENDC}")
                        
                        elif "POSCTL" in mode_name or "STABILIZED" in mode_name:
                            if "MISSION" in self.current_mode or "OFFBOARD" in self.current_mode:
                                self.call_planner(self.local_position[0], self.local_position[1], self.local_position[2] + 0.3)
                    
                    self.current_mode = mode_name
                    print(msg_q)
                    self.px4_conn.write(msg_q.get_msgbuf())
                
                # -------------------------------------------------------------------
                #                              MISSION                            
                # -------------------------------------------------------------------
                elif q_type in ["MISSION_COUNT", "MISSION_ITEM_INT", "MISSION_CLEAR_ALL"]:
                    self.handle_mission_message(msg_q)

                # elif "MISSION" in q_type:
                #     rospy.loginfo(f"{YELLOW}QGC ---> Mission Command: {q_type} Received ---> Drone{ENDC}")
                #     print(msg_p)

                # -------------------------------------------------------------------
                #                               ELSE                            
                # -------------------------------------------------------------------
                else:
                    self.px4_conn.write(msg_q.get_msgbuf())

            # Small sleep to prevent 100% CPU usage
            time.sleep(0.001)

if __name__ == '__main__':
    keeper = MasterGatekeeper()
    keeper.run()