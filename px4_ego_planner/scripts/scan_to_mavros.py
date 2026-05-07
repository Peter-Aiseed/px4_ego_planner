#!/usr/bin/env python3
import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan

def callback(msg):
    # 1. Shift the array to the North for 0 degree start (Rear-to-Front vs Front-to-Rear)
    raw_ranges = np.array(msg.ranges)
    target_size = 72
    indices = np.linspace(0, len(raw_ranges) - 1, target_size).astype(int)
    downsampled = raw_ranges[indices]
    shift = int(len(downsampled) * 0.75)
    fixed_ranges = np.concatenate((downsampled[shift:], downsampled[:shift]))

    # 2. Apply the fixed data to the message
    msg.ranges = fixed_ranges
    msg.angle_min = 0.0
    msg.angle_max = 2.0 * math.pi
    msg.angle_increment = math.pi / 36
    msg.header.frame_id = "base_link_frd"
    msg.header.stamp = rospy.Time.now()

    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('laser_to_mavros')

    # Input: The raw scan from pointcloud_to_laserscan
    # Output: The topic MAVROS is listening to
    pub = rospy.Publisher('/mavros/obstacle/send', LaserScan, queue_size=10)
    sub = rospy.Subscriber('/obstacles_laserscan', LaserScan, callback)

    rospy.loginfo("Laser Scan Fixer Started: Reversing array and forcing base_link_frd")
    rospy.spin()