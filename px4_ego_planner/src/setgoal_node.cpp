#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "setgoal_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  double point_x, point_y, point_z;
  double pub_hz;
  std::string frame_id;
  std::string topic;
  bool latch;

  pnh.param("point_x", point_x, 15.0);
  pnh.param("point_y", point_y, 0.0);
  pnh.param("point_z", point_z, 1.5);
  pnh.param("pub_hz", pub_hz, 1.0);                 // 默认 1Hz（按需改）
  pnh.param("frame_id", frame_id, std::string("map"));
  pnh.param("topic", topic, std::string("/move_base_simple/goal"));
  pnh.param("latch", latch, false);

  ros::Publisher sp_pub = nh.advertise<geometry_msgs::PoseStamped>(topic, 10, latch);

  geometry_msgs::PoseStamped desired_point;
  desired_point.header.frame_id = frame_id;
  desired_point.pose.orientation.w = 1.0;  // 无旋转

  ros::Rate rate(pub_hz);
  while (ros::ok())
  {
    desired_point.header.stamp = ros::Time::now();
    desired_point.pose.position.x = point_x;
    desired_point.pose.position.y = point_y;
    desired_point.pose.position.z = point_z;

    sp_pub.publish(desired_point);
    ROS_INFO("Send a goal point: [%.1f, %.1f, %.1f]", point_x, point_y, point_z);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
