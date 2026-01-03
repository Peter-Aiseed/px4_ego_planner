#include "ros/ros.h"
#include <iostream>
#include <string.h>
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
#include "nav_msgs/Odometry.h"

using namespace std;

static string mesh_resource;
visualization_msgs::Marker robot;
ros::Publisher robot_pub;

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Mesh model                                                  
    robot.header.frame_id = "world";
    robot.header.stamp = msg->header.stamp; 

    robot.ns = "mesh";
    robot.id = 0;
    
    robot.type = visualization_msgs::Marker::MESH_RESOURCE;
    robot.action = visualization_msgs::Marker::ADD;

    // robot's pose
    robot.pose.position.x = msg->pose.pose.position.x;
    robot.pose.position.y = msg->pose.pose.position.y;
    robot.pose.position.z = msg->pose.pose.position.z;
    robot.pose.orientation.w = msg->pose.pose.orientation.w;
    robot.pose.orientation.x = msg->pose.pose.orientation.x;
    robot.pose.orientation.y = msg->pose.pose.orientation.y;
    robot.pose.orientation.z = msg->pose.pose.orientation.z;

    robot.scale.x = 2.0;
    robot.scale.y = 2.0;
    robot.scale.z = 2.0;

    // robot's color
    robot.color.a = 1.0;
    robot.color.r = 1.0;
    robot.color.g = 0.0;
    robot.color.b = 0.0;

    robot.mesh_resource = mesh_resource;
    robot_pub.publish(robot);
}

void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // Mesh model
    robot.header.frame_id = "world";
    robot.header.stamp = msg->header.stamp;

    robot.ns = "mesh";
    robot.id = 0;

    robot.type = visualization_msgs::Marker::MESH_RESOURCE;
    robot.action = visualization_msgs::Marker::ADD;

    // robot's pose
    robot.pose.position.x = msg->pose.position.x;
    robot.pose.position.y = msg->pose.position.y;
    robot.pose.position.z = msg->pose.position.z;
    robot.pose.orientation.w = msg->pose.orientation.w;
    robot.pose.orientation.x = msg->pose.orientation.x;
    robot.pose.orientation.y = msg->pose.orientation.y;
    robot.pose.orientation.z = msg->pose.orientation.z;

    robot.scale.x = 2.0;
    robot.scale.y = 2.0;
    robot.scale.z = 2.0;

    // robot's color
    robot.color.a = 1.0;
    robot.color.r = 1.0;
    robot.color.g = 0.0;
    robot.color.b = 0.0;

    robot.mesh_resource = mesh_resource;
    robot_pub.publish(robot);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualization");
  ros::NodeHandle n("~");

  std::string odom_topic;
  bool use_odom = false;

  n.param("pose_topic", odom_topic, string("/mavros/local_position/odom"));
  n.param("mesh_resource", mesh_resource, string("package://px4_ego_planner/resource/meshes/quadrotor.mesh"));
  n.param("use_odom", use_odom, false);
  
  if(use_odom)
  	ros::Subscriber sub_odom = n.subscribe(pose_topic, 10, odom_callback);
  else:
	ros::Subscriber sub_odom = n.subscribe(pose_topic, 10, pose_callback);
  
  robot_pub = n.advertise<visualization_msgs::Marker>("robot", 10, true);  

  ros::spin();

  return 0;
}
