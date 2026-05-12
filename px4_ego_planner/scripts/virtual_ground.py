#!/usr/bin/env python3
import rospy
import numpy as np
import tf2_ros
import tf_conversions
import struct
from sensor_msgs.msg import PointCloud2, Range, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

class VirtualGroundGenerator:
    def __init__(self):
        rospy.init_node('virtual_ground_generator')

        # --- Parameters ---
        # Grid size in meters (e.g., 8x8 meter square)
        self.grid_size = rospy.get_param('~grid_size', 8.0) 
        # Resolution (0.5m means a point every 50cm)
        self.res = rospy.get_param('~resolution', 0.5)
        # The frame we are publishing in (FLU stable frame)
        self.target_frame = "base_link_stable"

        # --- Pre-calculate the Grid (Optimization) ---
        # This prevents running expensive loops inside the callback
        x = np.arange(-self.grid_size/2, self.grid_size/2, self.res)
        y = np.arange(-self.grid_size/2, self.grid_size/2, self.res)
        gx, gy = np.meshgrid(x, y)
        self.grid_x = gx.flatten().astype(np.float32)
        self.grid_y = gy.flatten().astype(np.float32)
        self.num_points = self.grid_x.size

        # --- PACK COLOR (rgb8) ---
        r, g, b = 0, 100, 0
        # Bit-pack then convert to float32 to match datatype 7
        self.rgb_val = struct.unpack('f', struct.pack('I', (r << 16 | g << 8 | b)))[0]

        # --- TF Listener for Tilt Correction ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # --- Pubs/Subs ---
        # Subscribe to your rangefinder (change topic name if needed)
        self.range_sub = rospy.Subscriber('/range_finder/data', Range, self.range_callback)
        self.cloud_pub = rospy.Publisher('/pointcloud/ground', PointCloud2, queue_size=1)

        rospy.loginfo(f"Virtual Ground active. Frame: {self.target_frame}, Points: {self.num_points}")

    def range_callback(self, msg):
        try:
            # 1. Get current drone tilt (Roll/Pitch) to correct rangefinder
            # We look up the transform from odom to the real (tilting) base_link
            t = self.tf_buffer.lookup_transform('odom', 'base_link', rospy.Time(0))
            q = [t.transform.rotation.x, t.transform.rotation.y, 
                 t.transform.rotation.z, t.transform.rotation.w]
            roll, pitch, _ = tf_conversions.transformations.euler_from_quaternion(q)

            # 2. Calculate True Vertical Altitude
            # Rangefinder measures distance along its axis; we need the vertical component
            # If the drone tilts 20 degrees, true_z = range * cos(20)
            true_z = -msg.range * np.cos(roll) * np.cos(pitch)

           # 3. Build the data with the 4-byte GAP (Offset 12 to 16)
            # We use 32 bytes per point (standard for aligned PointXYZRGB)
            # Layout: [X(4), Y(4), Z(4), PAD(4), RGB(4), UNUSED(12)]
            buffer_dtype = np.dtype({
                'names': ['x', 'y', 'z', 'rgb'],
                'formats': ['f4', 'f4', 'f4', 'f4'],
                'offsets': [0, 4, 8, 16], # Explicitly skip 12-15
                'itemsize': 32            # Force the 32-byte step
            })

            data = np.zeros(self.num_points, dtype=buffer_dtype)
            data['x'] = self.grid_x
            data['y'] = self.grid_y
            data['z'] = true_z
            data['rgb'] = self.rgb_val
            
            # 4. Define fields with the correct offsets
            fields = [
                PointField(name="x", offset=0, datatype=7, count=1),
                PointField(name="y", offset=4, datatype=7, count=1),
                PointField(name="z", offset=8, datatype=7, count=1),
                PointField(name="rgb", offset=16, datatype=7, count=1)
            ]

            header = Header(stamp=rospy.Time.now(), frame_id=self.target_frame)
            cloud_msg = PointCloud2()
            cloud_msg.header = header
            cloud_msg.height = 1
            cloud_msg.width = self.num_points
            cloud_msg.is_dense = False
            cloud_msg.is_bigendian = False
            cloud_msg.fields = fields
            cloud_msg.point_step = 32
            cloud_msg.row_step = 32 * self.num_points
            cloud_msg.data = data.tobytes()
            self.cloud_pub.publish(cloud_msg)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn_throttle(10, "Waiting for TF to calculate tilt correction...")

if __name__ == '__main__':
    try:
        node = VirtualGroundGenerator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass