#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <cmath>
#include <string>
#include <vector>

using namespace std;

// ---------------- Global state ----------------
static ros::Publisher pub_sp;
static mavros_msgs::State g_state;
static geometry_msgs::PoseStamped g_vision_pose;
static ros::Time g_last_vision_time(0);
static ros::Time g_last_traj_time(0);

// ---------------- Callback functions ----------------
static void state_callback(const mavros_msgs::State::ConstPtr& msg)
{ 
    g_state = *msg;
}

static void vision_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  g_vision_pose = *msg;
  g_last_vision_time = ros::Time::now();
}

static void  traj_callback(const mavros_msgs::PositionTarget::ConstPtr& /*msg*/)
{
  g_last_traj_time = ros::Time::now();
}

static void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  const auto& p = msg->pose.position;

  ROS_INFO("Goal position x=%.3f y=%.3f z=%.3f", p.x, p.y, p.z);
}

// ---------------- Others functions ----------------
static bool vision_check(const double max_sec)
{
  if (g_last_vision_time.isZero()) return false;
  const double age = (ros::Time::now() - g_last_vision_time).toSec();
  if (age > max_sec) return false;

  const auto& p = g_vision_pose.pose.position;
  if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) return false;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return false;
  return true;
}

static bool traj_check(const double max_sec)
{
  if (g_last_traj_time.isZero()) return false;
  const double age = (ros::Time::now() - g_last_traj_time).toSec();
  if(age < max_sec) return true;
  return false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "real_flight_mode_manager");
  ros::NodeHandle nh("~");

  // Make sure this node can print all information
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
    ros::console::notifyLoggerLevelsChanged();

  // Register subscriber and publisher
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_callback);
  ros::Subscriber vision_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10, vision_callback);
  ros::Subscriber traj_sub = nh.subscribe<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10, traj_callback);
  ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, goal_callback);
  pub_sp = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

  ros::Rate rate(25);

  while (ros::ok())
  {
    ros::spinOnce();

    const bool connected = g_state.connected;
    const bool v_ok = vision_check(0.5);
    const double altitude = g_vision_pose.pose.position.z;
    const bool alt_ok = (altitude > 0.5);
    const bool traj_ok = traj_check(0.5);
    const bool offboard_ok = (g_state.mode == "OFFBOARD");

    if (!offboard_ok)
    {
      if (connected && v_ok)
      {
        if (alt_ok)
        {
          ROS_INFO_THROTTLE(2.0, "OFFBOARD ready. Please switch mode via RC.");
          // send some pionts to assit in activating offboard mode.
          pub_sp.publish(g_vision_pose);
        }
        else
        {
          ROS_WARN_THROTTLE(2.0, "Altitude too low (z=%.2f <= 0.5m).", altitude);
        }
      }
      else
      {
        if (!connected) ROS_WARN_THROTTLE(2.0, "Not connected to FCU.");
        else if (!v_ok) ROS_WARN_THROTTLE(2.0, "No valid vision pose yet.");
      }
    }
    else if(offboard_ok && !traj_ok) {
      pub_sp.publish(g_vision_pose);
    }

    rate.sleep();
  }

  return 0;
}
