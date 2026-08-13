#include "plan_env/grid_map.h"

void GridMap::initMap(ros::NodeHandle &nh)
{
  node_ = nh;

  double x_size, y_size, z_size;
  node_.param("grid_map/resolution", mp_.resolution_, 0.1);
  node_.param("grid_map/map_size_x", x_size, 20.0);
  node_.param("grid_map/map_size_y", y_size, 20.0);
  node_.param("grid_map/map_size_z", z_size, 5.0);
  node_.param("grid_map/local_update_range_x", mp_.local_update_range_(0), 5.0);
  node_.param("grid_map/local_update_range_y", mp_.local_update_range_(1), 5.0);
  node_.param("grid_map/local_update_range_z", mp_.local_update_range_(2), 3.0);
  node_.param("grid_map/obstacles_inflation", mp_.obstacles_inflation_, 0.2);

  /* Camera intrinsics for depth clearing */
  node_.param("grid_map/fx", mp_.fx_, 385.0);
  node_.param("grid_map/fy", mp_.fy_, 385.0);
  node_.param("grid_map/cx", mp_.cx_, 320.0);
  node_.param("grid_map/cy", mp_.cy_, 240.0);

  node_.param("grid_map/depth_filter_maxdist", mp_.depth_filter_maxdist_, 5.0);
  node_.param("grid_map/depth_filter_mindist", mp_.depth_filter_mindist_, 0.2);
  node_.param("grid_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_, 1000.0);
  node_.param("grid_map/skip_pixel", mp_.skip_pixel_, 2);

  /* Log-odds probabilities */
  node_.param("grid_map/p_hit", mp_.p_hit_, 0.70);
  node_.param("grid_map/p_miss", mp_.p_miss_, 0.35);
  node_.param("grid_map/p_min", mp_.p_min_, 0.12);
  node_.param("grid_map/p_max", mp_.p_max_, 0.97);
  node_.param("grid_map/p_occ", mp_.p_occ_, 0.80);
  node_.param("grid_map/max_ray_length", mp_.max_ray_length_, 5.0);

  node_.param("grid_map/visualization_truncate_height", mp_.visualization_truncate_height_, 2.5);
  node_.param("grid_map/virtual_ceil_height", mp_.virtual_ceil_height_, 3.0);
  node_.param("grid_map/ground_height", mp_.ground_height_, 0.0);
  node_.param("grid_map/frame_id", mp_.frame_id_, string("world"));

  // Re-add PointCloud min/max distance parameters
  node_.param("grid_map/pointcloud_maxdist", mp_.pointcloud_maxdist_, 5.0);
  node_.param("grid_map/pointcloud_mindist", mp_.pointcloud_mindist_, 0.2);

  // Re-add timeout parameter
  node_.param("grid_map/odom_depth_timeout", mp_.odom_depth_timeout_, 1.0);

  mp_.resolution_inv_ = 1.0 / mp_.resolution_;
  mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);

  for (int i = 0; i < 3; ++i)
    mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

  /* Ring buffer window size setup */
  for (int i = 0; i < 3; ++i) {
    mp_.local_map_size_id_(i) = std::ceil(2.0 * (mp_.local_update_range_(i) + mp_.obstacles_inflation_) * mp_.resolution_inv_) + 2;
  }
  mp_.current_center_id_ = Eigen::Vector3i(0, 0, 0);

  int buffer_size = mp_.local_map_size_id_(0) * mp_.local_map_size_id_(1) * mp_.local_map_size_id_(2);

  md_.occupancy_buffer_ = std::vector<double>(buffer_size, mp_.clamp_min_log_);
  md_.occupancy_buffer_inflate_ = std::vector<char>(buffer_size, 0);

  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_.count_hit_ = vector<short>(buffer_size, 0);
  md_.flag_rayend_ = vector<char>(buffer_size, -1);
  md_.flag_traverse_ = vector<char>(buffer_size, -1);
  md_.raycast_num_ = 0;

  md_.cam2body_ << 0.0, 0.0, 1.0, 0.0,
                  -1.0, 0.0, 0.0, 0.0,
                   0.0, -1.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 1.0;

  /* ROS Subscribers */
  odom_sub_ = node_.subscribe<nav_msgs::Odometry>("grid_map/odom", 50, &GridMap::odomCallback, this);
  cloud_sub_ = node_.subscribe<sensor_msgs::PointCloud2>("grid_map/cloud", 10, &GridMap::cloudCallback, this);
  depth_sub_ = node_.subscribe<sensor_msgs::Image>("grid_map/depth", 10, &GridMap::depthCallback, this);

  /* ROS Publishers and Timers */
  map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy", 10);
  map_inf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy_inflate", 10);

  occ_timer_ = node_.createTimer(ros::Duration(0.05), &GridMap::updateOccupancyCallback, this);
  vis_timer_ = node_.createTimer(ros::Duration(0.10), &GridMap::visCallback, this);

  md_.has_odom_ = false;
  md_.has_cloud_ = false;
  md_.has_depth_ = false;
  md_.local_updated_ = false;
}

void GridMap::odomCallback(const nav_msgs::OdometryConstPtr &odom)
{
  Eigen::Quaterniond body_q(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                            odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
  Eigen::Matrix4d body2world = Eigen::Matrix4d::Identity();
  body2world.block<3, 3>(0, 0) = body_q.toRotationMatrix();
  body2world(0, 3) = odom->pose.pose.position.x;
  body2world(1, 3) = odom->pose.pose.position.y;
  body2world(2, 3) = odom->pose.pose.position.z;

  Eigen::Matrix4d cam_T = body2world * md_.cam2body_;
  md_.camera_pos_ = cam_T.block<3, 1>(0, 3);
  md_.camera_r_m_ = cam_T.block<3, 3>(0, 0);

  Eigen::Vector3i new_center_id;
  posToIndex(md_.camera_pos_, new_center_id);
  updateWindow(new_center_id);

  md_.has_odom_ = true;
}

void GridMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  pcl::PointCloud<pcl::PointXYZ> cloud_raw;
  pcl::fromROSMsg(*msg, cloud_raw);

  md_.cloud_points_.clear();

  if (cloud_raw.empty()) return;

  double min_dist_sq = mp_.pointcloud_mindist_ * mp_.pointcloud_mindist_;
  double max_dist_sq = mp_.pointcloud_maxdist_ * mp_.pointcloud_maxdist_;

  // Use camera/sensor position if available, otherwise default to drone position
  Eigen::Vector3d sensor_pos = md_.has_odom_ ? md_.camera_pos_ : Eigen::Vector3d::Zero();

  for (const auto &pt : cloud_raw.points) {
    // Check for invalid floating point values (NaN/Inf)
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;

    Eigen::Vector3d p(pt.x, pt.y, pt.z);
    double dist_sq = (p - sensor_pos).squaredNorm();

    // Distance Filter: Only keep points inside [mindist, maxdist]
    if (dist_sq >= min_dist_sq && dist_sq <= max_dist_sq) {
      md_.cloud_points_.push_back(pt);
    }
  }

  md_.has_cloud_ = !md_.cloud_points_.empty();
  md_.last_occ_update_time_ = ros::Time::now();
}

void GridMap::depthCallback(const sensor_msgs::ImageConstPtr &img)
{
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);
  md_.has_depth_ = true;
  md_.last_occ_update_time_ = ros::Time::now();
}

int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
  if (occ != 1 && occ != 0) return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  if (!isInMap(id)) return -1;

  int idx_ctns = toBufferAddress(id);
  md_.count_hit_and_miss_[idx_ctns] += 1;
  if (md_.count_hit_and_miss_[idx_ctns] == 1) {
    md_.cache_voxel_.push(id);
  }

  if (occ == 1) md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

void GridMap::processPointCloudInput()
{
  if (!md_.has_cloud_ || md_.cloud_points_.empty()) return;

  for (const auto& pt : md_.cloud_points_.points) {
    Eigen::Vector3d pt_w(pt.x, pt.y, pt.z);
    if (isInMap(pt_w)) {
      setCacheOccupancy(pt_w, 1);  // Mark as occupied hit
    }
  }
}

void GridMap::processDepthClearing()
{
  if (!md_.has_depth_ || md_.depth_image_.empty()) return;

  md_.raycast_num_ += 1;
  uint16_t *row_ptr;
  int cols = md_.depth_image_.cols;
  int rows = md_.depth_image_.rows;
  int skip_pix = mp_.skip_pixel_;
  double depth;

  RayCaster raycaster;
  Eigen::Vector3d half(0.5, 0.5, 0.5);
  Eigen::Vector3d ray_pt, pt_w, pt_cam;

  const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

  for (int v = 0; v < rows; v += skip_pix) {
    row_ptr = md_.depth_image_.ptr<uint16_t>(v);
    for (int u = 0; u < cols; u += skip_pix) {
      depth = (*row_ptr) * inv_factor;
      row_ptr += skip_pix;

      if (depth < mp_.depth_filter_mindist_) continue;

      if (depth > mp_.depth_filter_maxdist_ || depth == 0) {
        depth = mp_.max_ray_length_;
      }

      pt_cam(0) = (u - mp_.cx_) * depth / mp_.fx_;
      pt_cam(1) = (v - mp_.cy_) * depth / mp_.fy_;
      pt_cam(2) = depth;

      pt_w = md_.camera_r_m_ * pt_cam + md_.camera_pos_;

      /* Raycast free space along light path */
      raycaster.setInput(md_.camera_pos_ / mp_.resolution_, pt_w / mp_.resolution_);

      while (raycaster.step(ray_pt)) {
        Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;
        if (!isInMap(tmp)) break;

        int vox_idx = setCacheOccupancy(tmp, 0);  // Mark as free miss
        if (vox_idx != -1) {
          if (md_.flag_traverse_[vox_idx] == md_.raycast_num_) break;
          md_.flag_traverse_[vox_idx] = md_.raycast_num_;
        }
      }
    }
  }
}

void GridMap::applyCacheToBuffer()
{
  /* Flush whatever is currently queued in cache_voxel_ into the log-odds
     buffer. Called once after clearing and once after adding, so a hit and
     a miss on the same voxel in the same cycle never compete for the same
     update - whichever phase ran second simply wins for that voxel. */
  while (!md_.cache_voxel_.empty()) {
    Eigen::Vector3i idx = md_.cache_voxel_.front();
    md_.cache_voxel_.pop();

    int addr = toBufferAddress(idx);

    double log_odds_update = (md_.count_hit_[addr] > 0) ? mp_.prob_hit_log_ : mp_.prob_miss_log_;
    md_.count_hit_[addr] = 0;
    md_.count_hit_and_miss_[addr] = 0;

    md_.occupancy_buffer_[addr] = std::min(
        std::max(md_.occupancy_buffer_[addr] + log_odds_update, mp_.clamp_min_log_),
        mp_.clamp_max_log_);
  }
}

void GridMap::updateOccupancyCallback(const ros::TimerEvent &)
{
  if (!md_.has_odom_) return;

  /* Clear phase: depth-image raycasting misses go in and are flushed first,
     so free-space evidence is applied before any obstacle hits this cycle. */
  processDepthClearing();
  applyCacheToBuffer();

  /* Add phase: point-cloud hits go in and are flushed second, so a fresh
     detection always has the final say over a voxel that was also cleared
     this same cycle. */
  processPointCloudInput();
  applyCacheToBuffer();

  /* Apply local bounding limits */
  Eigen::Vector3d local_range_min = md_.camera_pos_ - mp_.local_update_range_;
  Eigen::Vector3d local_range_max = md_.camera_pos_ + mp_.local_update_range_;

  posToIndex(local_range_max, md_.local_bound_max_);
  posToIndex(local_range_min, md_.local_bound_min_);

  clearAndInflateLocalMap();
  md_.local_updated_ = true;
}

void GridMap::updateWindow(const Eigen::Vector3i& new_center_id) {
  Eigen::Vector3i delta = new_center_id - mp_.current_center_id_;
  if (delta.squaredNorm() == 0) return;

  Eigen::Vector3i min_new = new_center_id - mp_.local_map_size_id_ / 2;
  Eigen::Vector3i max_new = new_center_id + mp_.local_map_size_id_ / 2;

  Eigen::Vector3i min_old = mp_.current_center_id_ - mp_.local_map_size_id_ / 2;
  Eigen::Vector3i max_old = mp_.current_center_id_ + mp_.local_map_size_id_ / 2;

  /* Reset out-of-bound voxels standard for sliding window map */
  for (int x = min_new(0); x <= max_new(0); ++x) {
    for (int y = min_new(1); y <= max_new(1); ++y) {
      for (int z = min_new(2); z <= max_new(2); ++z) {
        if (x < min_old(0) || x > max_old(0) ||
            y < min_old(1) || y > max_old(1) ||
            z < min_old(2) || z > max_old(2)) {

          Eigen::Vector3i vox(x, y, z);
          int addr = toBufferAddress(vox);
          md_.occupancy_buffer_[addr] = mp_.clamp_min_log_;
          md_.occupancy_buffer_inflate_[addr] = 0;
        }
      }
    }
  }

  mp_.current_center_id_ = new_center_id;
}

void GridMap::clearAndInflateLocalMap()
{
  Eigen::Vector3d local_range_min = md_.camera_pos_ - mp_.local_update_range_;
  Eigen::Vector3d local_range_max = md_.camera_pos_ + mp_.local_update_range_;

  posToIndex(local_range_max, md_.local_bound_max_);
  posToIndex(local_range_min, md_.local_bound_min_);

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));

  // 2. Clear all inflated voxels in the local window
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z) {
        md_.occupancy_buffer_inflate_[toBufferAddress(Eigen::Vector3i(x, y, z))] = 0;
      }

  // 3. Re-inflate active obstacles
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z) {
        Eigen::Vector3i pt(x, y, z);
        if (md_.occupancy_buffer_[toBufferAddress(pt)] > mp_.min_occupancy_log_) {
          inflatePoint(pt, inf_step, inf_pts);

          for (const auto& inf_pt : inf_pts) {
            if (!isInMap(inf_pt)) continue;
            md_.occupancy_buffer_inflate_[toBufferAddress(inf_pt)] = 1;
          }
        }
      }
}

void GridMap::publishMap()
{
  if (map_pub_.getNumSubscribers() <= 0) return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z) {
        Eigen::Vector3i idx(x, y, z);
        if (md_.occupancy_buffer_[toBufferAddress(idx)] < mp_.min_occupancy_log_) continue;

        Eigen::Vector3d pos;
        indexToPos(idx, pos);
        if (pos(2) > mp_.visualization_truncate_height_) continue;

        pt.x = pos(0); pt.y = pos(1); pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_.publish(cloud_msg);
}

void GridMap::publishMapInflate(bool)
{
  if (map_inf_pub_.getNumSubscribers() <= 0) return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z) {
        Eigen::Vector3i idx(x, y, z);
        if (md_.occupancy_buffer_inflate_[toBufferAddress(idx)] == 0) continue;

        Eigen::Vector3d pos;
        indexToPos(idx, pos);
        if (pos(2) > mp_.visualization_truncate_height_) continue;

        pt.x = pos(0); pt.y = pos(1); pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  map_inf_pub_.publish(cloud_msg);
}

void GridMap::visCallback(const ros::TimerEvent &)
{
  publishMapInflate(true);
  publishMap();
}

void GridMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size)
{
  ori = mp_.map_origin_;
  size = mp_.map_size_;
}