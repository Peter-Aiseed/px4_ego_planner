#!/usr/bin/env python3
import rospy
import math
import tf2_ros
import tf_conversions
import geometry_msgs.msg

def broadcast_stable_frames():
    rospy.init_node('stable_tf_broadcaster')
    
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    br = tf2_ros.TransformBroadcaster()
    
    last_published_stamp = rospy.Time(0)
    rate = rospy.Rate(50.0)

    while not rospy.is_shutdown():
        try:
            # Get current drone transform (odom -> base_link_frd)
            t = tfBuffer.lookup_transform('odom', 'base_link_frd', rospy.Time(0))

            if t.header.stamp <= last_published_stamp:
                rate.sleep()
                continue
            last_published_stamp = t.header.stamp

            # Extract Position and Orientation
            trans = t.transform.translation
            q = [t.transform.rotation.x, t.transform.rotation.y, 
                 t.transform.rotation.z, t.transform.rotation.w]
            
            # Extract Roll, Pitch, Yaw
            # Since the source is FRD, Roll=0 Pitch=0 means Z is pointing DOWN.
            _, _, yaw = tf_conversions.transformations.euler_from_quaternion(q)

            # --- BROADCAST FLU FRAME (base_link_stable) ---
            # Standard ROS: Z-Up, Y-Left. We remove Roll and Pitch.
            # We set Roll to 0 to keep Z-Up relative to Odom.
            flu_t = geometry_msgs.msg.TransformStamped()
            flu_t.header.stamp = t.header.stamp  # Use the same timestamp as the source transform
            flu_t.header.frame_id = "odom"
            flu_t.child_frame_id = "base_link_stable"
            flu_t.transform.translation = trans
            
            # Roll 0, Pitch 0, Yaw = Yaw
            q_flu = tf_conversions.transformations.quaternion_from_euler(0, 0, yaw)
            flu_t.transform.rotation.x = q_flu[0]
            flu_t.transform.rotation.y = q_flu[1]
            flu_t.transform.rotation.z = q_flu[2]
            flu_t.transform.rotation.w = q_flu[3]
            br.sendTransform(flu_t)

            # --- BROADCAST FRD FRAME (base_link_stable_frd) ---
            # Standard PX4: Z-Down, Y-Right.
            # We add math.pi (180 deg) to roll to flip the Z axis down.
            frd_t = geometry_msgs.msg.TransformStamped()
            frd_t.header.stamp = t.header.stamp  # Use the same timestamp as the source transform
            frd_t.header.frame_id = "odom"
            frd_t.child_frame_id = "base_link_stable_frd"
            frd_t.transform.translation = trans
            
            # Roll PI, Pitch 0, Yaw = Yaw
            q_frd = tf_conversions.transformations.quaternion_from_euler(math.pi, 0, yaw)
            frd_t.transform.rotation.x = q_frd[0]
            frd_t.transform.rotation.y = q_frd[1]
            frd_t.transform.rotation.z = q_frd[2]
            frd_t.transform.rotation.w = q_frd[3]
            br.sendTransform(frd_t)

            # --- BROADCAST ALTITUDE ORIGIN FRAME (odom_drone_z) ---
            # Stays at XY = (0,0) but matches drone's altitude. Zero rotation.
            z_t = geometry_msgs.msg.TransformStamped()
            z_t.header.stamp = t.header.stamp
            z_t.header.frame_id = "odom"
            z_t.child_frame_id = "odom_drone_z"
            
            # Position: (0, 0, z_drone)
            z_t.transform.translation.x = 0.0
            z_t.transform.translation.y = 0.0
            z_t.transform.translation.z = trans.z
            
            # Orientation: Identity Quaternion (Roll=0, Pitch=0, Yaw=0)
            z_t.transform.rotation.x = 0.0
            z_t.transform.rotation.y = 0.0
            z_t.transform.rotation.z = 0.0
            z_t.transform.rotation.w = 1.0
            br.sendTransform(z_t)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue
        rate.sleep()

if __name__ == '__main__':
    broadcast_stable_frames()