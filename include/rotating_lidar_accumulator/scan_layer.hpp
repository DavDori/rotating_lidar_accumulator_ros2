#ifndef SCAN_LAYER_H
#define SCAN_LAYER_H

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <memory>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rotating_lidar_accumulator/utils/conversion.hpp"

class ScanLayer {
   public:
    ScanLayer(const sensor_msgs::msg::LaserScan& scan_msg,
              const Eigen::Transform<float, 3, Eigen::Affine>& lidar_offset,
              const Eigen::Vector3f& rotation_axis, float angle_rad,
              double time_increment, int ring_id, int num_scan);
    ScanLayer(const sensor_msgs::msg::LaserScan& scan_msg,
              const Eigen::Transform<float, 3, Eigen::Affine>& lidar_pose,
              double time_increment, int ring_id, int num_scan);
    ScanLayer(const ScanLayer& other);
    ScanLayer& operator=(const ScanLayer& other);
    ScanLayer(ScanLayer&& other) noexcept;
    ScanLayer& operator=(ScanLayer&& other) noexcept;
    ~ScanLayer() = default;

    pcl::PointCloud<PointXYZIRT>::Ptr getPointCloud() { return pc_; }
    rclcpp::Time getTime() const { return timestamp_; }
    size_t getNumPoints() const { return pc_->size(); }

   private:
    rclcpp::Time timestamp_;
    pcl::PointCloud<PointXYZIRT>::Ptr pc_;
};

#endif