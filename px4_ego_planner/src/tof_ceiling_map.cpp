// tof_ceiling_map_node.cpp
//
// Subscribes to an upward-facing ToF pointcloud (may be in a different frame
// than odom -- tf2-looked-up and transformed per message), maintains a
// drone-centered, world-anchored 11x11 grid @ 0.1m resolution (1.1m x 1.1m).
//
// Per cell, three flat arrays instead of a full sample window:
//  - committed_[cell]: the trusted value currently used by CPV/visualization.
//  - temporal_[cell]:  smallest reading seen during the current streak of
//                       samples that exceed committed_ by more than
//                       divergence_threshold_ (i.e. suggest MORE clearance).
//  - counter_[cell]:   how many consecutive such samples support temporal_.
//
// Asymmetric by design: a reading indicating a closer obstacle (z <= committed)
// is trusted immediately, no confirmation -- becoming more conservative is
// always safe to act on right away. A reading indicating more clearance
// (z > committed + threshold) only replaces committed_ after confirm_count_
// consecutive such readings, each compared against the fixed committed_
// baseline (not against each other), so sensor noise between candidate
// samples can't reset the streak. Cells never touched keep their committed_
// value indefinitely (no time decay).
//
// Publishes:
//  - full_grid: all 121 committed cells (x,y,z,range) as PointCloud2, for RViz/debug.
//  - /mavros/distance_sensor/ceiling_up: sensor_msgs/Range, min range over the
//    center 5x5 committed cells, for MAVROS to relay as MAVLink DISTANCE_SENSOR to PX4/CPV.
//
// Assumes ENU (z-up): range = cell_z - drone_z, positive = clearance above.

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <array>
#include <memory>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

class TofCeilingMap {
public:
    TofCeilingMap(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : half_((SIZE - 1) / 2),
          origin_x_(0.0f), origin_y_(0.0f),
          origin_set_(false),
          have_pose_(false),
          drone_x_(0.0f), drone_y_(0.0f), drone_z_(0.0f),
          have_raw_point_(false),
          raw_point_z_(0.0f)
    {
        pnh.param("resolution", resolution_, 0.1f);
        pnh.param("min_range", min_range_, 0.05f);
        pnh.param("max_range", max_range_, 15.0f);
        pnh.param("field_of_view", field_of_view_, 0.05f);
        pnh.param<std::string>("range_frame_id", range_frame_id_, "tof_up_link");
        pnh.param<std::string>("target_frame_id", target_frame_id_, "odom");
        pnh.param("divergence_threshold", divergence_threshold_, 1.0f); // meters
        pnh.param("confirm_count", confirm_count_, 10); // consecutive diverging samples needed to commit

        buildRings();
        tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));

        committed_.fill(std::numeric_limits<float>::quiet_NaN());
        temporal_.fill(std::numeric_limits<float>::quiet_NaN());
        counter_.fill(0);

        cloud_sub_ = nh.subscribe("/tof_sensor_up/pointcloud", 5, &TofCeilingMap::cloudCb, this);
        pose_sub_  = nh.subscribe("/mavros/local_position/pose", 20, &TofCeilingMap::poseCb, this);

        center_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud/tof_ceiling_map", 1);

        // Consumed by MAVROS's distance_sensor plugin, which forwards this as
        // a MAVLink DISTANCE_SENSOR message to PX4 (for CPV_UP_DIST / CPV_GO_NO_DATA).
        // Must match a "subscriber" entry (topic + id + orientation) in your
        // mavros px4_config.yaml under distance_sensor:
        range_pub_ = nh.advertise<sensor_msgs::Range>("/mavros/distance_sensor/upward_distance_sub", 1);

        ROS_INFO("tof_ceiling_map_node: %dx%d grid @ %.2fm res, confirm_count=%d, thresh=%.2fm",
                  SIZE, SIZE, resolution_, confirm_count_, divergence_threshold_);
    }

private:
    static constexpr int SIZE = 31;        // 31x31 -> 4.65m x 4.65m @ 0.15m res
    static constexpr int CENTER_SPAN = 5;  // 5x5 = 25 cells published

    static constexpr float RAW_MIN_Z = 1.0f;
    static constexpr float RAW_MAX_Z = 20.0f;

    const int half_;
    float resolution_;
    float min_range_, max_range_, field_of_view_;
    std::string range_frame_id_;
    std::string target_frame_id_;
    float divergence_threshold_;
    int confirm_count_;

    tf2_ros::Buffer tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    std::array<float, SIZE * SIZE> committed_; // trusted value, used by publishers
    std::array<float, SIZE * SIZE> temporal_;  // candidate value during a diverging streak
    std::array<int, SIZE * SIZE> counter_;     // consecutive diverging samples supporting temporal_

    std::vector<std::vector<int>> rings_;

    float origin_x_, origin_y_;
    bool origin_set_;

    bool have_pose_;
    float drone_x_, drone_y_, drone_z_;

    bool have_raw_point_;
    float raw_point_z_;

    ros::Subscriber cloud_sub_, pose_sub_;
    ros::Publisher center_pub_;
    ros::Publisher range_pub_;

    inline int idx(int ix, int iy) const { return iy * SIZE + ix; }

    inline float snap(float v) const {
        return roundf(v / resolution_) * resolution_;
    }

    void buildRings()
    {
        rings_.resize(half_ + 1);

        for (int iy = 0; iy < SIZE; ++iy) {
            for (int ix = 0; ix < SIZE; ++ix) {

                int dx = ix - half_;
                int dy = iy - half_;

                int ring = std::max(std::abs(dx), std::abs(dy));

                rings_[ring].push_back(idx(ix, iy));
            }
        }
    }

    void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        drone_x_ = msg->pose.position.x;
        drone_y_ = msg->pose.position.y;
        drone_z_ = msg->pose.position.z;
        have_pose_ = true;
        
        maybeShiftGrid();
    }

    // Re-origin the grid to stay centered on the drone; preserve overlapping cells.
    void maybeShiftGrid() {
        float new_origin_x = snap(drone_x_);
        float new_origin_y = snap(drone_y_);

        if (!origin_set_) {
            origin_x_ = new_origin_x;
            origin_y_ = new_origin_y;
            origin_set_ = true;
            return;
        }

        int dx = static_cast<int>(roundf((new_origin_x - origin_x_) / resolution_));
        int dy = static_cast<int>(roundf((new_origin_y - origin_y_) / resolution_));
        if (dx == 0 && dy == 0) return; // still in same center cell, no shift

        std::array<float, SIZE * SIZE> shifted_committed;
        std::array<float, SIZE * SIZE> shifted_temporal;
        std::array<int, SIZE * SIZE> shifted_counter;
        shifted_committed.fill(std::numeric_limits<float>::quiet_NaN());
        shifted_temporal.fill(std::numeric_limits<float>::quiet_NaN());
        shifted_counter.fill(0);

        for (int iy = 0; iy < SIZE; ++iy) {
            for (int ix = 0; ix < SIZE; ++ix) {
                int src_ix = ix + dx;
                int src_iy = iy + dy;
                if (src_ix >= 0 && src_ix < SIZE && src_iy >= 0 && src_iy < SIZE) {
                    int dst = idx(ix, iy);
                    int src = idx(src_ix, src_iy);
                    shifted_committed[dst] = committed_[src];
                    shifted_temporal[dst] = temporal_[src];
                    shifted_counter[dst] = counter_[src];
                }
            }
        }

        committed_ = shifted_committed;
        temporal_ = shifted_temporal;
        counter_ = shifted_counter;
        origin_x_ = new_origin_x;
        origin_y_ = new_origin_y;
    }

    void insertSample(int cell, float z) {
        float& committed = committed_[cell];
        float& temporal = temporal_[cell];
        int& counter = counter_[cell];

        // ROS_INFO_THROTTLE(0.2, "UPDATE cell=%d z=%.2f committed=%.2f diff=%.2f counter=%d", cell, z, committed, std::isnan(committed) ? 0.0f : (z - committed), counter);

        if (std::isnan(committed)) {
            // No trusted value yet: trust the first reading immediately.
            committed = z;
            temporal = std::numeric_limits<float>::quiet_NaN();
            counter = 0;
            return;
        }

        if (z <= committed) {
            // Reading indicates an equal-or-closer obstacle: trust immediately,
            // no confirmation needed. Becoming more conservative is always safe
            // to act on right away.
            committed = z;
            temporal = std::numeric_limits<float>::quiet_NaN();
            counter = 0;
            return;
        }

        // z > committed: reading suggests more clearance than currently trusted.
        // This is the direction that needs confirmation before we trust it,
        // since a false "more clearance" reading is the dangerous case for CPV.
        if ((z - committed) <= divergence_threshold_) {
            // Within noise tolerance of committed -- not a real divergence, ignore.
            return;
        }

        // Genuine divergence. Compare every new candidate against the fixed
        // committed baseline (not against the previous candidate) so that
        // noise between successive divergent samples can't reset the streak --
        // each sample only needs to still be far from committed, not close to
        // the last one.
        if (std::isnan(temporal)) {
            temporal = z;
            counter = 1;
        } else {
            temporal = std::min(temporal, z); // keep the most conservative candidate
            counter++;
        }

        if (counter >= confirm_count_) {
            committed = temporal;
            temporal = std::numeric_limits<float>::quiet_NaN();
            counter = 0;
        }
    }

    void insertPoint(float x, float y, float z) {

        // ROS_INFO_THROTTLE(0.2,
        //     "MAP POINT: x=%.3f y=%.3f z=%.3f | "
        //     "origin=(%.3f, %.3f) | dx=%.3f dy=%.3f",
        //     x, y, z,
        //     origin_x_, origin_y_,
        //     x - origin_x_, y - origin_y_);

        int ix = static_cast<int>(roundf((x - origin_x_) / resolution_)) + half_;
        int iy = static_cast<int>(roundf((y - origin_y_) / resolution_)) + half_;
        if (ix < 0 || ix >= SIZE || iy < 0 || iy >= SIZE) return; // outside grid, drop

        insertSample(idx(ix, iy), z);
    }

    void cloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        if (!have_pose_) return;

        sensor_msgs::PointCloud2 cloud_in_target;

        if (msg->header.frame_id == target_frame_id_) {
            // Already in the target frame, skip the lookup/transform cost.
            cloud_in_target = *msg;
        } else {
            geometry_msgs::TransformStamped tf_stamped;
            try {
                // Use the cloud's own stamp; falls back to latest available if
                // the exact stamp isn't in the tf buffer yet (small extrapolation
                // tolerance recommended in your tf2 buffer setup upstream).
                tf_stamped = tf_buffer_.lookupTransform(
                    target_frame_id_, msg->header.frame_id, msg->header.stamp,
                    ros::Duration(0.05));
            } catch (const tf2::TransformException& ex) {
                ROS_WARN_THROTTLE(2.0, "tof_ceiling_map: tf lookup %s -> %s failed: %s",
                                   msg->header.frame_id.c_str(), target_frame_id_.c_str(), ex.what());
                return; // drop this cloud, don't insert stale/garbage points
            }

            tf2::doTransform(*msg, cloud_in_target, tf_stamped);
        }

        sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud_in_target, "x");
        sensor_msgs::PointCloud2ConstIterator<float> it_y(cloud_in_target, "y");
        sensor_msgs::PointCloud2ConstIterator<float> it_z(cloud_in_target, "z");

        have_raw_point_ = false;

        for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
            if (std::isnan(*it_x) || std::isnan(*it_y) || std::isnan(*it_z)) continue;

            // Reject invalid raw ToF range
            if (!std::isfinite(*it_x) || !std::isfinite(*it_y) || !std::isfinite(*it_z)) continue;
            
            float raw_range = *it_z - drone_z_;
            if (raw_range < RAW_MIN_Z || raw_range > RAW_MAX_Z) continue;

            if (!have_raw_point_) {
                raw_point_z_ = *it_z;
                have_raw_point_ = true;
            }

            insertPoint(*it_x, *it_y, *it_z);
        }

        publishFullMap(msg->header.stamp);
        publishMinRange(msg->header.stamp);
    }

    // Smallest vertical clearance among the 25 center committed cells ->
    // sensor_msgs/Range for MAVROS to relay as DISTANCE_SENSOR to PX4.
    void publishMinRange(const ros::Time& stamp)
    {
        sensor_msgs::Range range_msg;
        range_msg.header.stamp = stamp;
        range_msg.header.frame_id = range_frame_id_;
        range_msg.radiation_type = sensor_msgs::Range::INFRARED;
        range_msg.field_of_view = field_of_view_;
        range_msg.min_range = min_range_;
        range_msg.max_range = max_range_;

        if (!have_pose_) {
            range_msg.range = max_range_ + 1.0f;
            range_pub_.publish(range_msg);
            return;
        }

        // ============================================================
        // Layer 1:
        // Search the center 5x5 map area.
        // ============================================================

        int cell = findCenterCell();
        if (cell >= 0) {
            float z = committed_[cell];
            float range = z - drone_z_;

            if (range < 0.0f)
                range = 0.0f;

            range_msg.range = std::min(range, max_range_);
            range_pub_.publish(range_msg);
            return;
        }

        // ============================================================
        // Layer 2:
        // Center 5x5 is empty.
        // Search the rest of the map from closest ring outward.
        // ============================================================

        cell = findClosestOuterMapCell();
        if (cell >= 0) {
            float z = committed_[cell];
            float range = z - drone_z_;

            if (range < 0.0f)
                range = 0.0f;

            range_msg.range = std::min(range, max_range_);
            range_pub_.publish(range_msg);
            return;
        }

        // ============================================================
        // Layer 3:
        // Map has no valid points.
        // Use the latest raw PointCloud measurement.
        // ============================================================

        if (have_raw_point_) {
            float range = raw_point_z_ - drone_z_;

            if (range < 0.0f)
                range = 0.0f;

            range_msg.range = std::min(range, max_range_);
            range_pub_.publish(range_msg);
            return;
        }

        // ============================================================
        // No map point and no raw PointCloud point.
        // Clear / out of sensing range.
        // ============================================================

        range_msg.range = max_range_ + 1.0f;
        range_pub_.publish(range_msg);
    }

    void publishFullMap(const ros::Time& stamp) {
        sensor_msgs::PointCloud2 out;
        out.header.stamp = stamp;
        out.header.frame_id = target_frame_id_;

        sensor_msgs::PointCloud2Modifier mod(out);
        mod.setPointCloud2Fields(4,
            "x", 1, sensor_msgs::PointField::FLOAT32,
            "y", 1, sensor_msgs::PointField::FLOAT32,
            "z", 1, sensor_msgs::PointField::FLOAT32,
            "range", 1, sensor_msgs::PointField::FLOAT32);
        mod.resize(SIZE * SIZE);

        sensor_msgs::PointCloud2Iterator<float> out_x(out, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(out, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(out, "z");
        sensor_msgs::PointCloud2Iterator<float> out_range(out, "range");

        int written = 0;

        for (int iy = 0; iy < SIZE; ++iy) {
            for (int ix = 0; ix < SIZE; ++ix) {
                float z = committed_[idx(ix, iy)];
                float wx = origin_x_ + (ix - half_) * resolution_;
                float wy = origin_y_ + (iy - half_) * resolution_;

                *out_x = wx;
                *out_y = wy;

                if (std::isnan(z) || !have_pose_) {
                    *out_z = std::numeric_limits<float>::quiet_NaN();
                    *out_range = std::numeric_limits<float>::quiet_NaN();
                } else {
                    *out_z = z;
                    *out_range = z - drone_z_; // ENU: positive = clearance above drone
                }

                ++out_x; ++out_y; ++out_z; ++out_range;
                ++written;
            }
        }

        out.width = written;
        out.height = 1;
        out.is_dense = false; // may contain NaNs for unmapped cells

        center_pub_.publish(out);
    }

    int findCenterCell()
    {
        const int center_ring = CENTER_SPAN / 2;
        int best_cell = -1;
        float best_z = std::numeric_limits<float>::infinity();

        for (int ring = 0; ring <= center_ring; ++ring) {
            for (int cell : rings_[ring]) {
                const float z = committed_[cell];

                if (std::isnan(z))
                    continue;

                if (z < best_z) {
                    best_z = z;
                    best_cell = cell;
                }
            }
        }
        return best_cell;
    }

    int findClosestOuterMapCell()
    {
        const int first_outer_ring = CENTER_SPAN / 2 + 1;

        for (int ring = first_outer_ring; ring <= half_; ++ring) {
            for (int cell : rings_[ring]) {
                if (std::isnan(committed_[cell]))
                    continue;

                return cell;
            }
        }
        return -1;
    }


};

int main(int argc, char** argv) {
    ros::init(argc, argv, "tof_ceiling_map");
    ros::NodeHandle nh, pnh("~");
    TofCeilingMap node(nh, pnh);
    ros::spin();
    return 0;
}