/*
    The camera orientation is defined as the RDF  frame.
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

static const Eigen::Matrix3d R_RDF_FLU = (Eigen::Matrix3d() <<
    0, 0, 1,
    -1, 0, 0,
    0, -1, 0).finished();

static Eigen::Vector3d P_vins;
static Eigen::Matrix3d R_vins;

static geometry_msgs::PoseStamped local_pose;
static nav_msgs::Path path;

static bool vins_received = false;

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    Eigen::Quaterniond q_imu(
        msg->orientation.w,
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z
    );

    R_imu = q_imu.toRotationMatrix();

    if(vins_received) {
        // you have to consider the timestamp of the coordinates,
        // as some coordinates are static while others change over time.
        Eigen::Vector3d P_tran = R_imu * R_RDF_FLU * R_vins.transpose() * P_vins;
        local_pose.pose.position.x = P_tran.x();
        local_pose.pose.position.y = P_tran.y();
        local_pose.pose.position.z = P_tran.z();
        local_pose.pose.orientation.w = q_imu.w();
        local_pose.pose.orientation.x = q_imu.x();
        local_pose.pose.orientation.y = q_imu.y();
        local_pose.pose.orientation.z = q_imu.z();
        
        // publish visual odometry to px4
        local_pose.header.stamp = ros::Time::now();
        pub_vio.publish(local_pose);
        path.poses.push_back(local_pose);
        pub_path.publish(path);

        vins_received = false;
    }
}

void vins_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    P_vins << msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z;

    Eigen::Quaterniond q_vins(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z
    );

    R_vins = q_vins.toRotationMatrix();

    if(!vins_received) {
        path.header.stamp = ros::Time::now();
        vins_received = true;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vins_transfer");
    ros::NodeHandle nh("~");

    local_pose.header.frame_id = "map";
	path.header.frame_id = "map";
    /*
        imu/data: orientation in quaternion form computed by FCU (PX4).
        imu/data_raw: only raw imu data without quaternion.
    */
    ros::Subscriber sub_vins = nh.subscribe("/vins_estimator/imu_propagate", 10, vins_callback);
    ros::Subscriber sub_imu = nh.subscribe("/mavros/imu/data", 10, imu_callback);
    ros::Publisher pub_vio = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
	ros::Publisher pub_path = nh.advertise<nav_msgs::Path>("path", 10);

    ros::spin();
    return 0;
}
