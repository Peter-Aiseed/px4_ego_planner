#!/usr/bin/env python3
import rospy
import tf2_ros
import tf_conversions
import geometry_msgs.msg

def broadcast_leveled_frame():
    rospy.init_node('base_link_stable_tf_broadcaster')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    br = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(50.0) # 50Hz is plenty

    while not rospy.is_shutdown():
        try:
            # Get current drone position relative to odom
            t = tfBuffer.lookup_transform('odom', 'base_link_frd', rospy.Time(0))
            
            # Create a new transform
            leveled_t = geometry_msgs.msg.TransformStamped()
            leveled_t.header.stamp = rospy.Time.now()
            leveled_t.header.frame_id = "odom"
            leveled_t.child_frame_id = "base_link_stable"

            # Copy Position
            leveled_t.transform.translation = t.transform.translation
            
            # Extract only the YAW (Heading)
            q = [t.transform.rotation.x, t.transform.rotation.y, 
                 t.transform.rotation.z, t.transform.rotation.w]
            roll, pitch, yaw = tf_conversions.transformations.euler_from_quaternion(q)
            
            # Create new rotation with 0 Roll and 0 Pitch
            q_new = tf_conversions.transformations.quaternion_from_euler(0, 0, yaw)
            leveled_t.transform.rotation.x = q_new[0]
            leveled_t.transform.rotation.y = q_new[1]
            leveled_t.transform.rotation.z = q_new[2]
            leveled_t.transform.rotation.w = q_new[3]

            br.sendTransform(leveled_t)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue
        rate.sleep()

if __name__ == '__main__':
    broadcast_leveled_frame()