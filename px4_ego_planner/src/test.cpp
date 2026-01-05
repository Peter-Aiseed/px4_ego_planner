/**
 * offboard_mission_simple.cpp
 *
 * ROS1 (Noetic) + MAVROS + PX4 OFFBOARD mission example:
 * - Wait for FCU connection
 * - Validate /mavros/vision_pose/pose freshness
 * - Continuously publish setpoints (current pose) to allow RC switching into OFFBOARD
 * - After OFFBOARD is entered (by RC), fly through waypoints
 * - Yaw always points toward the current target waypoint
 * - Hover at each waypoint for 5 seconds, then proceed
 *
 * Parameters (private, i.e. under "~"):
 *   point_num (int)
 *   point{i}_x, point{i}_y, point{i}_z (double), i = 0..point_num-1
 *
 * Topics:
 *   Sub: /mavros/state (mavros_msgs/State)
 *   Sub: /mavros/vision_pose/pose (geometry_msgs/PoseStamped)
 *   Pub: /mavros/setpoint_position/local (geometry_msgs/PoseStamped)
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>

#include <cmath>
#include <string>
#include <vector>

// ---------------- Global state ----------------
static mavros_msgs::State g_state;
static geometry_msgs::PoseStamped g_vision_pose;
static ros::Time g_last_vision_time(0);

static ros::Publisher g_setpoint_pub;

static bool g_in_offboard = false;

static std::vector<geometry_msgs::PoseStamped> g_wps;
static size_t g_wp_idx = 0;

// hover control
static bool g_hovering = false;
static ros::Time g_hover_start(0);

// ---------------- Callbacks ----------------
static void stateCb(const mavros_msgs::State::ConstPtr& msg) { g_state = *msg; }

static void visionPoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  g_vision_pose = *msg;
  // Use ROS time for freshness; for sim-time you can switch to msg->header.stamp if appropriate.
  g_last_vision_time = ros::Time::now();
}

// ---------------- Helpers ----------------
static inline double normalizeYaw(double yaw)
{
  while (yaw > M_PI) yaw -= 2.0 * M_PI;
  while (yaw < -M_PI) yaw += 2.0 * M_PI;
  return yaw;
}

// Make a quaternion from yaw (roll=pitch=0)
static geometry_msgs::Quaternion yawToQuat(double yaw_rad)
{
  geometry_msgs::Quaternion q;
  const double half = 0.5 * yaw_rad;
  q.x = 0.0;
  q.y = 0.0;
  q.z = 0.0;
  q.w = 1.0;
  // q.z = std::sin(half);
  // q.w = std::cos(half);
  return q;
}

static bool visionValid(const double max_age_sec)
{
  if (g_last_vision_time.isZero()) return false;
  const double age = (ros::Time::now() - g_last_vision_time).toSec();
  if (age > max_age_sec) return false;

  const auto& p = g_vision_pose.pose.position;
  if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) return false;
  return true;
}

static double distTo(const geometry_msgs::PoseStamped& target)
{
  const auto& c = g_vision_pose.pose.position;
  const auto& t = target.pose.position;
  const double dx = c.x - t.x;
  const double dy = c.y - t.y;
  const double dz = c.z - t.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

static double computeYawToTarget(const geometry_msgs::PoseStamped& target)
{
  const auto& c = g_vision_pose.pose.position;
  const auto& t = target.pose.position;

  const double dx = t.x - c.x;
  const double dy = t.y - c.y;

  // If target is extremely close in XY, keep yaw unchanged by returning NaN sentinel.
  if (std::hypot(dx, dy) < 1e-4) return std::numeric_limits<double>::quiet_NaN();

  return normalizeYaw(std::atan2(dy, dx));
}

static void publishPoseWithYaw(const geometry_msgs::PoseStamped& pose, const double yaw_rad)
{
  geometry_msgs::PoseStamped sp = pose;
  sp.header.stamp = ros::Time::now();
  sp.pose.orientation = yawToQuat(yaw_rad);
  g_setpoint_pub.publish(sp);
}

// ---------------- Mission state machine ----------------
static void runMissionStep(double& yaw_cmd)
{
  // No waypoints loaded: hold current position
  if (g_wps.empty())
  {
    ROS_WARN_THROTTLE(2.0, "No waypoints loaded. Holding current position.");
    publishPoseWithYaw(g_vision_pose, yaw_cmd);
    return;
  }

  // Mission finished: hold last waypoint
  if (g_wp_idx >= g_wps.size())
  {
    publishPoseWithYaw(g_wps.back(), yaw_cmd);
    return;
  }

  const auto& target = g_wps[g_wp_idx];

  // Update yaw command to face the current target (if target is not degenerate)
  const double yaw_to_target = computeYawToTarget(target);
  if (!std::isnan(yaw_to_target)) yaw_cmd = yaw_to_target;

  // Always publish target setpoint with yaw pointing to target
  publishPoseWithYaw(target, yaw_cmd);

  // Check reach
  constexpr double kReachTol = 0.05;  // 5 cm
  const double d = distTo(target);

  if (!g_hovering)
  {
    if (d < kReachTol)
    {
      g_hovering = true;
      g_hover_start = ros::Time::now();
      ROS_INFO("Reached waypoint %zu/%zu (%.2f, %.2f, %.2f). Hover 5s.",
               g_wp_idx + 1,
               g_wps.size(),
               target.pose.position.x,
               target.pose.position.y,
               target.pose.position.z);
    }
    return;
  }

  // Hovering
  const double hover_t = (ros::Time::now() - g_hover_start).toSec();
  if (hover_t >= 5.0)
  {
    g_hovering = false;
    g_wp_idx++;

    if (g_wp_idx < g_wps.size())
    {
      ROS_INFO("Proceed to waypoint %zu/%zu.", g_wp_idx + 1, g_wps.size());
    }
    else
    {
      ROS_INFO("Mission completed. Holding last waypoint.");
    }
  }
}

// ---------------- Parameter loading ----------------
static bool loadWaypoints(ros::NodeHandle& pnh)
{
  int point_num = 0;
  if (!pnh.getParam("point_num", point_num))
  {
    ROS_ERROR("Required param '~point_num' not set.");
    return false;
  }
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

    geometry_msgs::PoseStamped wp;
    wp.header.frame_id = "map";  // Ensure this matches your estimator/local frame usage
    wp.pose.position.x = x;
    wp.pose.position.y = y;
    wp.pose.position.z = z;
    // Orientation will be overwritten at publish time by publishPoseWithYaw()
    wp.pose.orientation.w = 1.0;

    g_wps.push_back(wp);
  }

  ROS_INFO("Loaded %zu waypoints from parameters.", g_wps.size());
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "offboard_mission_simple");

  ros::NodeHandle nh;        // for topics
  ros::NodeHandle pnh("~");  // for params

  if (ros::console::set_logger_level(
        ROSCONSOLE_DEFAULT_NAME,
        ros::console::levels::Info))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  // Load waypoints
  if (!loadWaypoints(pnh))
  {
    ROS_ERROR("Failed to load waypoints. Exiting.");
    return 1;
  }

  // ROS I/O
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, stateCb);
  ros::Subscriber vision_sub =
      nh.subscribe<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10, visionPoseCb);

  g_setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

  // Control loop
  constexpr double kLoopHz = 20.0;
  ros::Rate rate(kLoopHz);

  // Yaw command state: keep continuity across frames
  double yaw_cmd = 0.0;

  // For OFFBOARD readiness log throttling
  ROS_INFO("Node started. Waiting for FCU connection and valid vision pose...");

  while (ros::ok())
  {
    ros::spinOnce();

    const bool connected = g_state.connected;
    const bool v_ok = visionValid(0.5);  // max pose age 0.5s
    const double z = g_vision_pose.pose.position.z;
    const bool alt_ok = (z > 0.1);

    // Always publish something once we have a valid pose, to keep setpoint stream alive.
    // Before OFFBOARD, we publish current pose to avoid unexpected jumps at mode switch.
    if (!g_in_offboard)
    {
      if (connected && v_ok)
      {
        // Maintain a continuous yaw_cmd based on the first waypoint direction (optional).
        // This helps yaw behave deterministically even before mission starts.
        const double yaw_to_first = computeYawToTarget(g_wps.front());
        if (!std::isnan(yaw_to_first)) yaw_cmd = yaw_to_first;

        publishPoseWithYaw(g_vision_pose, yaw_cmd);

        if (alt_ok)
        {
          ROS_INFO_THROTTLE(2.0, "OFFBOARD ready. Please switch mode via RC.");
        }
        else
        {
          ROS_WARN_THROTTLE(2.0, "Altitude too low (z=%.3f <= 0.1m). Take off first.", z);
        }
      }
      else
      {
        if (!connected) ROS_WARN_THROTTLE(2.0, "Not connected to FCU.");
        if (!v_ok)      ROS_WARN_THROTTLE(2.0, "No valid vision pose yet.");
      }

      // Detect OFFBOARD entry
      if (g_state.mode == "OFFBOARD")
      {
        g_in_offboard = true;
        g_wp_idx = 0;
        g_hovering = false;
        ROS_INFO("OFFBOARD entered. Start mission.");
      }
    }
    else
    {
      // In OFFBOARD: require continuous valid pose; otherwise hold current position
      if (!connected)
      {
        ROS_ERROR_THROTTLE(1.0, "FCU disconnected while in OFFBOARD. Holding.");
        publishPoseWithYaw(g_vision_pose, yaw_cmd);
      }
      else if (!v_ok)
      {
        ROS_ERROR_THROTTLE(1.0, "Vision pose invalid while in OFFBOARD. Holding.");
        publishPoseWithYaw(g_vision_pose, yaw_cmd);
      }
      else
      {
        runMissionStep(yaw_cmd);
      }
    }

    rate.sleep();
  }

  return 0;
}

