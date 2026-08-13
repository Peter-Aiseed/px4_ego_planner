#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <Eigen/Eigen>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <plan_env/raycast.h>

#define logit(x) (log((x) / (1.0 - (x))))

using namespace std;

struct MappingParameters {
  /* Ring Buffer Window */
  Eigen::Vector3i local_map_size_id_;  // Ring buffer window grid dimensions
  Eigen::Vector3i current_center_id_;  // Ring buffer center voxel ID

  /* Map boundaries & resolution */
  Eigen::Vector3d map_origin_, map_size_;
  Eigen::Vector3d map_min_boundary_, map_max_boundary_;
  Eigen::Vector3i map_voxel_num_;
  Eigen::Vector3d local_update_range_;
  double resolution_, resolution_inv_;
  double obstacles_inflation_;
  string frame_id_;

  /* Depth camera parameters for clearing */
  double cx_, cy_, fx_, fy_;
  double depth_filter_maxdist_, depth_filter_mindist_;
  double k_depth_scaling_factor_;
  int skip_pixel_;

  /* Raycasting & Probabilities */
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_;
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_, min_occupancy_log_;
  double max_ray_length_;

  /* Visualization & Limits */
  double visualization_truncate_height_, virtual_ceil_height_, ground_height_;
  double odom_depth_timeout_;

  // PointCloud Distance Limits
  double pointcloud_maxdist_;
  double pointcloud_mindist_;
};

struct MappingData {
  std::vector<double> occupancy_buffer_;
  std::vector<char> occupancy_buffer_inflate_;

  /* Pose & Transforms */
  Eigen::Vector3d camera_pos_;
  Eigen::Matrix3d camera_r_m_;
  Eigen::Matrix4d cam2body_;

  /* Incoming Data Caches */
  cv::Mat depth_image_;
  pcl::PointCloud<pcl::PointXYZ> cloud_points_;

  /* Raycasting & Integration Caches */
  vector<short> count_hit_, count_hit_and_miss_;
  vector<char> flag_traverse_, flag_rayend_;
  char raycast_num_;
  queue<Eigen::Vector3i> cache_voxel_;

  /* Local Bounds */
  Eigen::Vector3i local_bound_min_, local_bound_max_;

  /* Status Flags */
  bool has_odom_, has_cloud_, has_depth_;
  bool local_updated_;
  ros::Time last_occ_update_time_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class GridMap {
public:
  GridMap() {}
  ~GridMap() {}

  void initMap(ros::NodeHandle& nh);

  /* Ring Buffer Address Translation */
  inline int toBufferAddress(const Eigen::Vector3i& id);
  inline void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
  inline void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
  inline bool isInMap(const Eigen::Vector3d& pos);
  inline bool isInMap(const Eigen::Vector3i& idx);

  /* Query States */
  inline int getOccupancy(Eigen::Vector3d pos);
  inline int getOccupancy(Eigen::Vector3i id);
  inline int getInflateOccupancy(Eigen::Vector3d pos);
  inline bool isUnknown(const Eigen::Vector3i& id);
  inline bool isKnownFree(const Eigen::Vector3i& id);
  inline bool isKnownOccupied(const Eigen::Vector3i& id);

  void publishMap();
  void publishMapInflate(bool all_info = false);

  inline double getResolution() { return mp_.resolution_; }
  Eigen::Vector3d getOrigin() { return mp_.map_origin_; }
  void getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size);
  bool odomValid() { return md_.has_odom_; }

  typedef std::shared_ptr<GridMap> Ptr;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Missing getter needed by ego_replan_fsm.cpp
  inline bool getOdomDepthTimeout() {
    if (!md_.has_odom_) return true;
    if ((ros::Time::now() - md_.last_occ_update_time_).toSec() > mp_.odom_depth_timeout_) {
      return true;
    }
    return false;
  }

private:
  /* Callbacks */
  void odomCallback(const nav_msgs::OdometryConstPtr& odom);
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void depthCallback(const sensor_msgs::ImageConstPtr& img);

  void updateOccupancyCallback(const ros::TimerEvent&);
  void visCallback(const ros::TimerEvent&);

  /* Mapping Operations */
  void processPointCloudInput();
  void processDepthClearing();
  void applyCacheToBuffer();
  void clearAndInflateLocalMap();
  void updateWindow(const Eigen::Vector3i& new_center_id);

  inline void inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts);
  int setCacheOccupancy(Eigen::Vector3d pos, int occ);

  ros::NodeHandle node_;
  ros::Subscriber odom_sub_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber depth_sub_;

  ros::Publisher map_pub_, map_inf_pub_;
  ros::Timer occ_timer_, vis_timer_;

  MappingParameters mp_;
  MappingData md_;
};

/* ============================== Inline Functions ============================== */

inline int GridMap::toBufferAddress(const Eigen::Vector3i& id) {
  int rx = (id(0) % mp_.local_map_size_id_(0) + mp_.local_map_size_id_(0)) % mp_.local_map_size_id_(0);
  int ry = (id(1) % mp_.local_map_size_id_(1) + mp_.local_map_size_id_(1)) % mp_.local_map_size_id_(1);
  int rz = (id(2) % mp_.local_map_size_id_(2) + mp_.local_map_size_id_(2)) % mp_.local_map_size_id_(2);

  return rx * mp_.local_map_size_id_(1) * mp_.local_map_size_id_(2) +
         ry * mp_.local_map_size_id_(2) + rz;
}

inline void GridMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
  for (int i = 0; i < 3; ++i) id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
}

inline void GridMap::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
  for (int i = 0; i < 3; ++i) pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
}

inline bool GridMap::isInMap(const Eigen::Vector3i& idx) {
  Eigen::Vector3i diff = idx - mp_.current_center_id_;
  return (std::abs(diff(0)) < mp_.local_map_size_id_(0) / 2) &&
         (std::abs(diff(1)) < mp_.local_map_size_id_(1) / 2) &&
         (std::abs(diff(2)) < mp_.local_map_size_id_(2) / 2);
}

inline bool GridMap::isInMap(const Eigen::Vector3d& pos) {
  Eigen::Vector3i idx;
  posToIndex(pos, idx);
  return isInMap(idx);
}

inline bool GridMap::isUnknown(const Eigen::Vector3i& id) {
  if (!isInMap(id)) return true;
  return md_.occupancy_buffer_[toBufferAddress(id)] < mp_.clamp_min_log_ - 1e-3;
}

inline bool GridMap::isKnownFree(const Eigen::Vector3i& id) {
  if (!isInMap(id)) return false;
  int adr = toBufferAddress(id);
  return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ && md_.occupancy_buffer_inflate_[adr] == 0;
}

inline bool GridMap::isKnownOccupied(const Eigen::Vector3i& id) {
  if (!isInMap(id)) return false;
  return md_.occupancy_buffer_inflate_[toBufferAddress(id)] == 1;
}

inline int GridMap::getOccupancy(Eigen::Vector3d pos) {
  if (!isInMap(pos)) return -1;
  Eigen::Vector3i id;
  posToIndex(pos, id);
  return md_.occupancy_buffer_[toBufferAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline int GridMap::getOccupancy(Eigen::Vector3i id) {
  if (!isInMap(id)) return -1;
  return md_.occupancy_buffer_[toBufferAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline int GridMap::getInflateOccupancy(Eigen::Vector3d pos) {
  if (!isInMap(pos)) return -1;
  Eigen::Vector3i id;
  posToIndex(pos, id);
  return int(md_.occupancy_buffer_inflate_[toBufferAddress(id)]);
}

inline void GridMap::inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts) {
  int num = 0;
  for (int x = -step; x <= step; ++x)
    for (int y = -step; y <= step; ++y)
      for (int z = -step; z <= step; ++z) {
        pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z);
      }
}

#endif