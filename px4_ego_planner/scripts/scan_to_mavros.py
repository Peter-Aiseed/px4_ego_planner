#!/usr/bin/env python3
import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan

def callback(msg):
    # 1. Shift the array to the North for 0 degree start (Rear-to-Front vs Front-to-Rear)
    raw_ranges = np.array(msg.ranges)
    shift = int(len(raw_ranges) * 0.5)
    turned_ranges = np.concatenate((raw_ranges[shift:], raw_ranges[:shift]))
    target_size = 72
    bin_size = len(turned_ranges) // target_size
    downsampled = np.min(turned_ranges[:target_size * bin_size].reshape(target_size, bin_size), axis=1)

    # 2. Apply the fixed data to the message
    msg.ranges = downsampled
    msg.angle_min = 0.0
    msg.angle_increment = math.pi / 36
    msg.header.frame_id = "base_link_stable_frd"
    msg.header.stamp = rospy.Time.now()

    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('laser_to_mavros')

    # Input: The raw scan from pointcloud_to_laserscan
    # Output: The topic MAVROS is listening to
    pub = rospy.Publisher('/mavros/obstacle/send', LaserScan, queue_size=10)
    sub = rospy.Subscriber('/obstacles_laserscan', LaserScan, callback)

    rospy.loginfo("Laser Scan Fixer Started: Fixing array and changing frame_id to base_link_stable_frd")
    rospy.spin()