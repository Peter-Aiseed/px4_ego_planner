/*
    The camera orientation is defined as the LUF frame.
    what mavros expects is the position in ENU, and the orientation defined as FLU relative to ENU.
*/
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>

using namespace std;

const Eigen::Matrix3d R_LUF_FLU = (Eigen::Matrix3d() <<
    0, 0, 1,
    1, 0, 0,
    0, 1, 0).finished();
    
Eigen::Matrix3d R_imu;

geometry_msgs::PoseStamped local_pose;
nav_msgs::Path path;

bool vins_received = false;
bool imu_received = false;

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    Eigen::Quaterniond q_imu(
        msg->orientation.w,
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z
    );

    R_imu = q_imu.toRotationMatrix();
	
    imu_received = true;
}

void vins_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if(imu_received) {
        Eigen::Vector3d P_vins(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z
        );

        Eigen::Quaterniond q_vins(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z
        );

        Eigen::Matrix3d R_vins = q_vins.toRotationMatrix();
		Eigen::Vector3d P_tran = R_imu * R_LUF_FLU * R_vins.transpose() * P_vins;
        Eigen::Quaterniond q_tran(R_imu);
        q_tran.normalize();

        // construct the pose that will be sent to PX4
        local_pose.pose.position.x = P_tran.x();
        local_pose.pose.position.y = P_tran.y();
        local_pose.pose.position.z = P_tran.z();

        local_pose.pose.orientation.w = q_tran.w();
        local_pose.pose.orientation.x = q_tran.x();
        local_pose.pose.orientation.y = q_tran.y();
        local_pose.pose.orientation.z = q_tran.z();
		
		// ROS_INFO("camera position relative to ENU: x=%.2f, y=%.2f, z=%.2f", P_tran.x(), P_tran.y(), P_tran.z());
		vins_received = true;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vins_transfer");
    ros::NodeHandle nh;

    ros::Subscriber sub_vins = nh.subscribe("/vins_estimator/imu_propagate", 10, vins_callback);
    /*
        imu/data: orientation in quaternion form computed by FCU (PX4).
        imu/data_raw: only raw imu data without quaternion.
    */
    ros::Subscriber sub_imu = nh.subscribe("/mavros/imu/data", 10, imu_callback);
    ros::Publisher pub_vio = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
	ros::Publisher pub_path = nh.advertise<nav_msgs::Path>("path", 10);

    ros::Rate rate(50);

    local_pose.header.frame_id = "map";
	path.header.stamp = ros::Time::now();
	path.header.frame_id = "map";

    while(ros::ok())
    {
        if(vins_received)
        {
            local_pose.header.stamp = ros::Time::now();
            pub_vio.publish(local_pose);
			path.poses.push_back(local_pose);
			pub_path.publish(path);
            vins_received = false;
        }

        ros::spinOnce();    // process callbacks in the queue once.
        rate.sleep();
    }

    return 0;
}
