#!/usr/bin/env python3
import rospy
import math
import numpy as np
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from sensor_msgs.msg import LaserScan
import tf2_ros

class LocalCostmapToMavros:
    def __init__(self):
        rospy.init_node('costmap_to_mavros_obstacles', anonymous=True)
        
        self.num_bins = 72 
        self.max_detection_dist = 10.0 
        self.angle_increment = (2 * math.pi) / self.num_bins
        self.angle_shift = self.angle_increment / 2.0 
        
        # Storage for the persistent global costmap state
        self.grid = None
        self.map_info = None
        self.map_header = None

        # TF Buffer to look up the drone's heading (yaw) and exact position
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Subscribers for BOTH full map and incremental updates
        self.sub_full = rospy.Subscriber('/costmap_node/costmap/costmap', OccupancyGrid, self.full_map_callback, queue_size=1)
        self.sub_update = rospy.Subscriber('/costmap_node/costmap/costmap_updates', OccupancyGridUpdate, self.map_update_callback, queue_size=10)
        
        self.pub = rospy.Publisher('/mavros/obstacle/send', LaserScan, queue_size=10)
        
        # Timer callback executing at 10 Hz to guarantee steady MAVROS stream
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        rospy.loginfo("Costmap to MAVROS node started.")

    def full_map_callback(self, msg):
        """Receives the initial map and complete maps if re-published."""
        self.map_header = msg.header
        self.map_info = msg.info
        self.grid = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))

    def map_update_callback(self, msg):
        """Slices incremental bounding box updates directly into our stored grid."""
        if self.grid is None:
            return # Wait until we have a base map
            
        # Update header timestamp to keep track of fresh data
        self.map_header.stamp = msg.header.stamp
        
        # Reshape the incoming mini-patch of data
        update_data = np.array(msg.data, dtype=np.int8).reshape((msg.height, msg.width))
        
        # Splice the patch into the master grid using the given bounding box coordinates
        self.grid[msg.y : msg.y + msg.height, msg.x : msg.x + msg.width] = update_data

    def timer_callback(self, event):
        if self.grid is None or self.map_info is None:
            return

        # 1. Look up drone heading (yaw) relative to the map frame
        try:
            # We look up base_link relative to the costmap frame (e.g., odom or map)
            t = self.tf_buffer.lookup_transform(self.map_header.frame_id, 'base_link', rospy.Time(0))
            q = t.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            drone_yaw = math.atan2(siny_cosp, cosy_cosp)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn_throttle(2.0, "Waiting for TF transform to find drone heading...")
            return

        res = self.map_info.resolution
        width = self.map_info.width
        height = self.map_info.height
        
        # 2. Extract active obstacle pixels
        y_indices, x_indices = np.where(self.grid > 50)
        
        if len(x_indices) == 0:
            self.publish_empty_scan()
            return

        # 3. Calculate distance and angles relative to the map frame (with drone at center)
        center_x = width // 2
        center_y = height // 2
        
        dx = (x_indices - center_x) * res
        dy = (y_indices - center_y) * res
        
        distances = np.hypot(dx, dy)
        angles_map = np.arctan2(dy, dx)
        
        # Filter range
        valid_mask = (distances < self.max_detection_dist) & (distances > 0.1)
        distances = distances[valid_mask]
        angles_map = angles_map[valid_mask]
        
        if len(distances) == 0:
            self.publish_empty_scan()
            return

        # 4. Correct for drone heading (Map Angles -> Drone Body Angles)
        # Subtracting yaw rotates the map space into the drone's local coordinate system.
        angles_body = angles_map - drone_yaw
        
        # 5. Shift by 2.5 deg and normalize to [0, 2*pi] so Bin 0 centers forward
        shifted_angles = (-angles_body) + self.angle_shift
        shifted_angles = np.mod(shifted_angles, 2 * math.pi)
        
        # Determine bin indices [0 to 71]
        bin_indices = (shifted_angles / self.angle_increment).astype(int)
        bin_indices = np.clip(bin_indices, 0, self.num_bins - 1)

        # 6. Build and send LaserScan
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = "base_link_stable_frd"
        scan.angle_min = 0
        scan.angle_increment = self.angle_increment
        scan.range_min = 0.1
        scan.range_max = self.max_detection_dist
        
        ranges = np.full(self.num_bins, self.max_detection_dist + 1.0)
        
        for b_idx in np.unique(bin_indices):
            bin_distances = distances[bin_indices == b_idx]
            ranges[b_idx] = np.min(bin_distances)
            
        scan.ranges = ranges.tolist()
        self.pub.publish(scan)

    def publish_empty_scan(self):
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = "base_link_stable_frd"
        scan.angle_min = 0
        scan.angle_increment = self.angle_increment
        scan.range_min = 0.1
        scan.range_max = self.max_detection_dist
        scan.ranges = [self.max_detection_dist + 1.0] * self.num_bins
        self.pub.publish(scan)

if __name__ == '__main__':
    try:
        LocalCostmapToMavros()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass