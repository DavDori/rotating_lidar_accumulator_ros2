#include "rotating_lidar_accumulator/scan_layer.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include "rotating_lidar_accumulator/utils/conversion_ros2.hpp"

namespace rla {

namespace {

Eigen::Affine3f makeLidarPose(const Eigen::Affine3f& lidar_offset,
                              const Eigen::Vector3f& rotation_axis,
                              float angle_rad) {
    return lidar_offset *
           Eigen::AngleAxisf(angle_rad, rotation_axis.normalized());
}

}  // namespace

ScanLayer::ScanLayer(
    const sensor_msgs::msg::LaserScan& scan_msg,
    const Eigen::Transform<float, 3, Eigen::Affine>& lidar_pose,
    const utils::ScanInfo& info)
    : timestamp_(scan_msg.header.stamp) {
    auto cloud = utils::convertLaserScanToPointCloud(scan_msg, info);
    if (!cloud) {
        throw std::runtime_error("Could not convert scan into point cloud.");
    }
    pc_ = std::make_shared<pcl::PointCloud<core::PointXYZIRT>>(*cloud);

    // Transform the lidar scan with a single transformation that already takes
    // into account for the rotation of the lidar.
    pcl::transformPointCloud(*pc_, *pc_, lidar_pose);
}

ScanLayer::ScanLayer(
    const sensor_msgs::msg::LaserScan& scan_msg,
    const Eigen::Transform<float, 3, Eigen::Affine>& lidar_offset,
    const Eigen::Vector3f& rotation_axis, float angle_rad,
    const utils::ScanInfo& info)
    : ScanLayer(scan_msg, makeLidarPose(lidar_offset, rotation_axis, angle_rad),
                info) {}

ScanLayer::ScanLayer(const ScanLayer& other)
    : timestamp_(other.timestamp_),
      pc_(std::make_shared<pcl::PointCloud<core::PointXYZIRT>>(*other.pc_)) {}

ScanLayer& ScanLayer::operator=(const ScanLayer& other) {
    if (this != &other) {
        // Deep copy of the point cloud
        timestamp_ = other.timestamp_;
        pc_ = std::make_shared<pcl::PointCloud<core::PointXYZIRT>>(*other.pc_);
    }
    return *this;
}

ScanLayer::ScanLayer(ScanLayer&& other) noexcept
    : timestamp_(std::move(other.timestamp_)),
      pc_(std::move(other.pc_))  // Transfers ownership of the point cloud
{
    // The "other" object is now empty or in a valid but unspecified state.
}

ScanLayer& ScanLayer::operator=(ScanLayer&& other) noexcept {
    if (this != &other) {  // Check for self-assignment
        timestamp_ = std::move(other.timestamp_);
        pc_ = std::move(other.pc_);  // Transfers ownership
    }
    return *this;
}
}  // namespace rla