#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <iostream>

using namespace std;

static mavros_msgs::State current_state;
static geometry_msgs::PoseStamped current_pose;
static vector<Eigen::Vector3d> g_wps;
static Eigen::Matrix3d R_FLU_ENU;
static bool first_receive = true;

void state_callback(const mavros_msgs::State::ConstPtr& msg)
{
	current_state = *msg;
}

void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	current_pose = *msg;

	if(current_state.mode == "OFFBOARD" && first_receive) {
		Eigen::Quaterniond q_imu(
			msg->pose.orientation.w,
			msg->pose.orientation.x,
			msg->pose.orientation.y,
			msg->pose.orientation.z
		);

		R_FLU_ENU = q_imu.toRotationMatrix();

		first_receive = false;
	}
}

static double distTo(const geometry_msgs::PoseStamped& target)
{
	const auto& c = current_pose.pose.position;
	const auto& t = target.pose.position;
	const double dx = c.x - t.x;
	const double dy = c.y - t.y;
	const double dz = c.z - t.z;
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

static bool loadWaypoints(ros::NodeHandle& pnh)
{
	int point_num = 0;
	pnh.getParam("point_num", point_num);

	if (point_num <= 0)
	{
		ROS_ERROR("'~point_num' must be > 0, got %d", point_num);
		return false;
	}

	g_wps.clear();
	g_wps.reserve(static_cast<size_t>(point_num));

	for (int i = 0; i < point_num; ++i)
	{
		double x = 0.0, y = 0.0, z = 0.0;
		const std::string base = "point" + std::to_string(i);

		const bool ok =
		pnh.getParam(base + "_x", x) &&
		pnh.getParam(base + "_y", y) &&
		pnh.getParam(base + "_z", z);

		if (!ok)
		{
			ROS_ERROR("Missing waypoint params for %s: need ~%s_x, ~%s_y, ~%s_z",
			base.c_str(), base.c_str(), base.c_str(), base.c_str());
			return false;
		}

		Eigen::Vector3d P(x, y, z);

		g_wps.push_back(P);
	}

	ROS_INFO("Loaded %zu waypoints from parameters.", g_wps.size());
	return true;
}

static geometry_msgs::PoseStamped set_target(size_t idx)
{
    geometry_msgs::PoseStamped target;

    if (idx < g_wps.size())
    {
        Eigen::Vector3d P_desired = R_FLU_ENU * g_wps[idx];
        target.pose.position.x = P_desired.x();
        target.pose.position.y = P_desired.y();
        target.pose.position.z = P_desired.z();

        const double dx = target.pose.position.x - current_pose.pose.position.x;
        const double dy = target.pose.position.y - current_pose.pose.position.y;
        const double dist2 = dx * dx + dy * dy;

		// please note that, here, all rotation is relative to ENU frame.
        if (dist2 > 0.1)
        {
			const double yaw = std::atan2(dy, dx);
			ROS_INFO("desired yaw is: %.2f", yaw * 180.0 / M_PI);
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, yaw);   // roll, pitch, yaw
            target.pose.orientation = tf2::toMsg(q);
        }
        else
        {
            target.pose.orientation = current_pose.pose.orientation;
        }
    }
    else
    {
        target = current_pose;
    }

    return target;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "setgoal_node");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	if (ros::console::set_logger_level(
		ROSCONSOLE_DEFAULT_NAME,
		ros::console::levels::Info))
	{
		ros::console::notifyLoggerLevelsChanged();
	}  

	if (!loadWaypoints(pnh))
	{
		ROS_ERROR("Failed to load waypoints. Exiting.");
		return 1;
	}

	ros::Publisher sp_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_callback);
	ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pose_callback);

	const double kReachTol = 0.10;  // 10 cm
	size_t wp_idx = 0;
	geometry_msgs::PoseStamped target_pose;

	ros::Rate rate(1);
	while (ros::ok())
	{
		if(current_state.mode == "OFFBOARD" && !first_receive) {
			if(wp_idx == 0)	target_pose = set_target(wp_idx);

			if(distTo(target_pose) < kReachTol) {
				wp_idx ++;
				target_pose = set_target(wp_idx);
			}
			target_pose.header.frame_id = "world";
			target_pose.header.stamp = ros::Time::now();
			sp_pub.publish(target_pose);
		}

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
