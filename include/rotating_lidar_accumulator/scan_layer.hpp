#pragma once

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rotating_lidar_accumulator/core/custom_point_type.hpp"
#include "rotating_lidar_accumulator/utils/conversion_ros2.hpp"

namespace rla {
class ScanLayer {
   public:
    ScanLayer(const sensor_msgs::msg::LaserScan& scan_msg,
              const Eigen::Transform<float, 3, Eigen::Affine>& lidar_pose,
              const utils::ScanInfo& info);
    ScanLayer(const sensor_msgs::msg::LaserScan& scan_msg,
              const Eigen::Transform<float, 3, Eigen::Affine>& lidar_offset,
              const Eigen::Vector3f& rotation_axis, float angle_rad,
              const utils::ScanInfo& info);
    ScanLayer(const ScanLayer& other);
    ScanLayer& operator=(const ScanLayer& other);
    ScanLayer(ScanLayer&& other) noexcept;
    ScanLayer& operator=(ScanLayer&& other) noexcept;
    ~ScanLayer() = default;

    [[nodiscard]] pcl::PointCloud<core::PointXYZIRT>::Ptr getPointCloud() {
        return pc_;
    }
    [[nodiscard]] rclcpp::Time getTime() const { return timestamp_; }
    [[nodiscard]] size_t getNumPoints() const { return pc_->size(); }

   private:
    rclcpp::Time timestamp_;
    pcl::PointCloud<core::PointXYZIRT>::Ptr pc_;
};
}  // namespace rla