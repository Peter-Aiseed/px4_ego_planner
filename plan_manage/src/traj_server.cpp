#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "traj_utils/Bspline.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
#include <tf/tf.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher px4_pos_cmd_pub;
mavros_msgs::PositionTarget px4_cmd;

double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using ego_planner::UniformBspline;

bool in_mission_ = false;
vector<UniformBspline> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_;

// Setpoint Control
Eigen::Vector3d current_pos_ = Eigen::Vector3d::Zero();
double current_yaw_ = 0.0;

void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    current_pos_(0) = msg->pose.position.x;
    current_pos_(1) = msg->pose.position.y;
    current_pos_(2) = msg->pose.position.z;

    tf::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);

    tf::Matrix3x3 m(q);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    current_yaw_ = yaw;
}

void bsplineCallback(traj_utils::BsplineConstPtr msg)
{
  // parse pos traj

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  // parse yaw traj

  // Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  // for (int i = 0; i < msg->yaw_pts.size(); ++i) {
  //   yaw_pts(i, 0) = msg->yaw_pts[i];
  // }

  //UniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());

  traj_duration_ = traj_[0].getTimeSum();

  in_mission_ = true;
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last)
{
  constexpr double PI = 3.1415926;
  constexpr double YAW_DOT_MAX_PER_SEC = PI;
  // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw = 0;
  double yawdot = 0;

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
  double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).toSec();
  if (yaw_temp - last_yaw_ > PI)
  {
    if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }
  else if (yaw_temp - last_yaw_ < -PI)
  {
    if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }
  else
  {
    if (yaw_temp - last_yaw_ < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else if (yaw_temp - last_yaw_ > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }

  if (fabs(yaw - last_yaw_) <= max_yaw_change)
    yaw = 0.5 * last_yaw_ + 0.5 * yaw; // nieve LPF
  yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
  last_yaw_ = yaw;
  last_yaw_dot_ = yawdot;

  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;
}

void cmdCallback(const ros::TimerEvent &e)
{
  ros::Time time_now = ros::Time::now();
  static ros::Time time_last = ros::Time::now();

  /* Publishing own position to keep track on the drone */
  if (!in_mission_)
  {
    time_last = time_now;
    px4_cmd.header.stamp = time_now;

    px4_cmd.coordinate_frame = 1;
    px4_cmd.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                        mavros_msgs::PositionTarget::IGNORE_VY |
                        mavros_msgs::PositionTarget::IGNORE_VZ |
                        mavros_msgs::PositionTarget::IGNORE_AFX |
                        mavros_msgs::PositionTarget::IGNORE_AFY |
                        mavros_msgs::PositionTarget::IGNORE_AFZ |
                        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
                        
    px4_cmd.position.x = current_pos_(0);
    px4_cmd.position.y = current_pos_(1);
    px4_cmd.position.z = current_pos_(2);

    px4_cmd.velocity.x = 0;
    px4_cmd.velocity.y = 0;
    px4_cmd.velocity.z = 0;

    px4_cmd.acceleration_or_force.x = 0;
    px4_cmd.acceleration_or_force.y = 0;
    px4_cmd.acceleration_or_force.z = 0;

    px4_cmd.yaw = current_yaw_;
    last_yaw_ = current_yaw_;

    px4_cmd.yaw_rate = 0;

    px4_pos_cmd_pub.publish(px4_cmd);

    return;
  }

  double t_cur = (time_now - start_time_).toSec();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);

  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);

    /*** calculate yaw ***/
    yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);
    /*** calculate yaw ***/

    double tf = min(traj_duration_, t_cur + 2.0);
    pos_f = traj_[0].evaluateDeBoorT(tf);
  }
  else if (t_cur >= traj_duration_)
  {
    /* hover when finish traj_ */
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();

    yaw_yawdot.first = last_yaw_;
    yaw_yawdot.second = 0;

    pos_f = pos;

    /* Calculate the distance between current position and goal*/
    double dist = (current_pos_ - pos).norm();

    /*Mission invalidation or Trajectory ownership timeout or Change to Pilot Control*/ 
    if (dist > 1.0) 
    {
      in_mission_ = false;
    }
  }
  else
  {
    cout << "[Traj server]: invalid time." << endl;
    time_last = time_now;
    return;
  }
  time_last = time_now;

  px4_cmd.header.stamp = time_now;
  px4_cmd.coordinate_frame = 1;
  px4_cmd.type_mask = 0;

  px4_cmd.position.x = pos(0);
  px4_cmd.position.y = pos(1);
  px4_cmd.position.z = pos(2);

  px4_cmd.velocity.x = vel(0);
  px4_cmd.velocity.y = vel(1);
  px4_cmd.velocity.z = vel(2);

  px4_cmd.acceleration_or_force.x = acc(0);
  px4_cmd.acceleration_or_force.y = acc(1);
  px4_cmd.acceleration_or_force.z = acc(2);

  px4_cmd.yaw = yaw_yawdot.first;
  px4_cmd.yaw_rate = yaw_yawdot.second;

  px4_pos_cmd_pub.publish(px4_cmd);

  last_yaw_ = px4_cmd.yaw;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  // ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Subscriber bspline_sub = nh.subscribe("planning/bspline", 10, bsplineCallback);

  ros::Subscriber pose_sub = nh.subscribe("/mavros/local_position/pose", 10, poseCallback);

  px4_pos_cmd_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 20);

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.05), cmdCallback);

  nh.param("traj_server/time_forward", time_forward_, -1.0);
  last_yaw_ = 0.0;
  last_yaw_dot_ = 0.0;

  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}