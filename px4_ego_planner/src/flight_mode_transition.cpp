#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>

static mavros_msgs::State g_state;

// local odom gate
static bool g_odom_received = false;
static ros::Time g_last_odom_time(0);
static nav_msgs::Odometry g_odom;

// raw setpoint gate
static bool g_raw_received = false;
static ros::Time g_last_raw_time(0);

static void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  g_state = *msg;
}

static void odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
  g_odom = *msg;
  g_last_odom_time = ros::Time::now();
  g_odom_received = true;
}

static void raw_sp_cb(const mavros_msgs::PositionTarget::ConstPtr& /*msg*/)
{
  g_last_raw_time = ros::Time::now();
  g_raw_received = true;
}

static bool isOdomValid(double timeout_s)
{
  if (!g_odom_received) return false;
  return (ros::Time::now() - g_last_odom_time) < ros::Duration(timeout_s);
}

static bool isRawActive(double timeout_s)
{
  if (!g_raw_received) return false;
  return (ros::Time::now() - g_last_raw_time) < ros::Duration(timeout_s);
}

static bool setMode(ros::ServiceClient& set_mode_client, const std::string& mode)
{
  mavros_msgs::SetMode sm;
  sm.request.custom_mode = mode;
  if (set_mode_client.call(sm) && sm.response.mode_sent)
  {
    ROS_INFO("Mode request sent: %s", mode.c_str());
    return true;
  }
  ROS_WARN("Mode request failed: %s", mode.c_str());
  return false;
}

static bool arm(ros::ServiceClient& arming_client, bool value)
{
  mavros_msgs::CommandBool cmd;
  cmd.request.value = value;
  if (arming_client.call(cmd) && cmd.response.success)
  {
    ROS_INFO("%s success.", value ? "Arming" : "Disarming");
    return true;
  }
  ROS_WARN("%s failed.", value ? "Arming" : "Disarming");
  return false;
}

enum class Stage
{
  WAIT_ODOM = 0,
  ARMING,
  TAKEOFF_OFFBOARD, // use OFFBOARD + local position setpoint (relative height)
  HOLD
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_mode_manager");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // takeoff_height: relative climb height (meters)
  double takeoff_height = 1.5;
  double altitude_tol = 0.10;
  double odom_timeout = 0.5;
  double raw_timeout = 0.3;
  double cmd_interval = 1.0;
  std::string hold_mode = "AUTO.LOITER"; // adjust if your PX4 exposes "HOLD"

  pnh.param<double>("takeoff_height", takeoff_height, 1.5);
  pnh.param<double>("altitude_tol", altitude_tol, 0.10);
  pnh.param<double>("odom_timeout", odom_timeout, 0.5);
  pnh.param<double>("raw_timeout", raw_timeout, 0.3);
  pnh.param<double>("cmd_interval", cmd_interval, 1.0);
  pnh.param<std::string>("hold_mode", hold_mode, std::string("AUTO.LOITER"));

  ros::Subscriber state_sub = nh.subscribe("/mavros/state", 10, state_cb);
  ros::Subscriber odom_sub  = nh.subscribe("/mavros/local_position/odom", 10, odom_cb);
  ros::Subscriber raw_sub   = nh.subscribe("/mavros/setpoint_raw/local", 10, raw_sp_cb);

  ros::Publisher sp_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

  ros::ServiceClient arming_client   = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

  ros::Rate rate(20.0);

  // wait for FCU
  while (ros::ok() && !g_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("FCU connected.");

  Stage stage = Stage::WAIT_ODOM;
  ros::Time last_cmd_time = ros::Time::now() - ros::Duration(cmd_interval);

  // takeoff target (relative)
  bool takeoff_target_inited = false;
  double takeoff_start_z = 0.0;
  double takeoff_target_z = 0.0;

  while (ros::ok())
  {
    ros::spinOnce();

    const bool odom_ok = isOdomValid(odom_timeout);
    const bool raw_ok  = isRawActive(raw_timeout);

    // HOLD 阶段：根据 /mavros/setpoint_raw/local 有无数据在 HOLD <-> OFFBOARD 之间切换
    if (stage == Stage::HOLD)
    {
      if (raw_ok && g_state.mode != "OFFBOARD" &&
          (ros::Time::now() - last_cmd_time) > ros::Duration(0.2))
      {
        setMode(set_mode_client, "OFFBOARD");
        last_cmd_time = ros::Time::now();
      }
      else if (!raw_ok && g_state.mode == "OFFBOARD" &&
               (ros::Time::now() - last_cmd_time) > ros::Duration(0.2))
      {
        setMode(set_mode_client, hold_mode);
        last_cmd_time = ros::Time::now();
      }

      // 不发布 setpoint，避免与外部 raw setpoint 竞争
      rate.sleep();
      continue;
    }

    switch (stage)
    {
      case Stage::WAIT_ODOM:
      {
        if (odom_ok)
        {
          ROS_INFO("Local odom valid. Start arming.");
          stage = Stage::ARMING;
          last_cmd_time = ros::Time::now() - ros::Duration(cmd_interval);
        }
        else
        {
          static ros::Time last_warn(0);
          if ((ros::Time::now() - last_warn) > ros::Duration(1.0))
          {
            ROS_WARN("Waiting for valid /mavros/local_position/odom ...");
            last_warn = ros::Time::now();
          }
        }
        break;
      }

      case Stage::ARMING:
      {
        if (!odom_ok)
        {
          ROS_WARN("Odom lost. Back to WAIT_ODOM.");
          stage = Stage::WAIT_ODOM;
          break;
        }

        if (!g_state.armed && (ros::Time::now() - last_cmd_time) > ros::Duration(cmd_interval))
        {
          arm(arming_client, true);
          last_cmd_time = ros::Time::now();
        }

        if (g_state.armed)
        {
          ROS_INFO("Armed. Start OFFBOARD takeoff (relative height).");
          stage = Stage::TAKEOFF_OFFBOARD;
          last_cmd_time = ros::Time::now() - ros::Duration(cmd_interval);
          takeoff_target_inited = false;
        }
        break;
      }

      case Stage::TAKEOFF_OFFBOARD:
      {
        if (!odom_ok)
        {
          ROS_WARN("Odom lost during takeoff. Switch to HOLD.");
          setMode(set_mode_client, hold_mode);
          stage = Stage::HOLD;
          last_cmd_time = ros::Time::now();
          break;
        }

        // init relative takeoff target once
        if (!takeoff_target_inited)
        {
          takeoff_start_z = g_odom.pose.pose.position.z;
          takeoff_target_z = takeoff_start_z + takeoff_height;
          takeoff_target_inited = true;

          // OFFBOARD 需要先发一段 setpoint 再切模式（避免拒绝）
          geometry_msgs::PoseStamped sp;
          sp.header.frame_id = "map";
          sp.pose.position.x = g_odom.pose.pose.position.x;
          sp.pose.position.y = g_odom.pose.pose.position.y;
          sp.pose.position.z = takeoff_target_z;
          sp.pose.orientation.w = 1.0;

          for (int i = 0; ros::ok() && i < 20; ++i) // 1s @ 20Hz
          {
            sp.header.stamp = ros::Time::now();
            sp_pub.publish(sp);
            ros::spinOnce();
            rate.sleep();
          }
          setMode(set_mode_client, "OFFBOARD");
          last_cmd_time = ros::Time::now();
          ROS_INFO("Takeoff target z: start=%.2f target=%.2f", takeoff_start_z, takeoff_target_z);
        }

        // keep publishing takeoff setpoint while taking off
        geometry_msgs::PoseStamped sp;
        sp.header.frame_id = "map";
        sp.header.stamp = ros::Time::now();
        sp.pose.position.x = g_odom.pose.pose.position.x; // hold current x/y
        sp.pose.position.y = g_odom.pose.pose.position.y;
        sp.pose.position.z = takeoff_target_z;
        sp.pose.orientation.w = 1.0;
        sp_pub.publish(sp);

        // reached target altitude -> HOLD
        const double z = g_odom.pose.pose.position.z;
        if (z >= takeoff_target_z - altitude_tol)
        {
          ROS_INFO("Reached takeoff altitude z=%.2f (target %.2f). Switch to HOLD(%s).",
                   z, takeoff_target_z, hold_mode.c_str());
          setMode(set_mode_client, hold_mode);
          stage = Stage::HOLD;
          last_cmd_time = ros::Time::now();
        }
        break;
      }

      case Stage::HOLD:
        // handled above
        break;
    }

    rate.sleep();
  }

  return 0;
}
