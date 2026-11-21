#include "rotating_lidar_accumulator/point_cloud_buffer.hpp"

namespace rla {

PointCloudBuffer::PointCloudBuffer()
    : PointCloudBuffer(Eigen::Transform<float, 3, Eigen::Affine>::Identity(),
                       Eigen::Vector3f(0.0f, 1.0f, 0.0f)) {
    reset();
}

PointCloudBuffer::PointCloudBuffer(
    const Eigen::Transform<float, 3, Eigen::Affine>& lidar_offset,
    const Eigen::Vector3f& turret_axis)
    : lidar_offset_(lidar_offset), turret_axis_(turret_axis) {
    reset();
}

void PointCloudBuffer::addScan(const sensor_msgs::msg::LaserScan& msg,
                               uint32_t scan_duration, float angle_rad) {
    utils::ScanInfo info;
    info.rel_scan_start = scan_duration * static_cast<uint32_t>(idx_);
    info.ring_id = static_cast<uint16_t>(idx_);
    info.scan_duration = scan_duration;
    ScanLayer scan(msg, lidar_offset_, turret_axis_, angle_rad, info);
    scans_.emplace_back(std::move(scan));
    ++idx_;
}

void PointCloudBuffer::addScan(
    const sensor_msgs::msg::LaserScan& msg, uint32_t scan_duration,
    Eigen::Transform<float, 3, Eigen::Affine>& lidar_pose) {
    utils::ScanInfo info;
    info.rel_scan_start = scan_duration * static_cast<uint32_t>(idx_);
    info.ring_id = static_cast<uint16_t>(idx_);
    info.scan_duration = scan_duration;
    ScanLayer scan(msg, lidar_pose, info);
    scans_.emplace_back(std::move(scan));
    ++idx_;
}

/*
Clear the buffer of all accumulated scans
*/
void PointCloudBuffer::reset() {
    scans_.clear();
    idx_ = 0;
}

void PointCloudBuffer::enableOrganizedPointcloud(
    const utils::PointCloudOrganizationParams& p) {
    organized_params_ = p;
    enable_organize_ = true;
}

void PointCloudBuffer::disableOrganizedPointcloud() {
    enable_organize_ = false;
}

// GETTERS
// ---------------------------------------------------------------------------------

/*
Merge all pointclouds stored in the buffer so far
*/
pcl::PointCloud<core::PointXYZIRT>::Ptr PointCloudBuffer::getTotalPointcloud() {
    auto merged_pointcloud =
        std::make_shared<pcl::PointCloud<core::PointXYZIRT>>();

    // Estimate total size and reserve space to optimize performance
    size_t total_points = 0;
    for (const auto& scan_layer : scans_) {
        total_points += scan_layer.getNumPoints();
    }
    merged_pointcloud->reserve(total_points);

    // Merge all point clouds (scans) from the buffer
    for (auto& scan_layer : scans_) {
        *merged_pointcloud += *(scan_layer.getPointCloud());
    }
    return merged_pointcloud;
}

/*
Merge all pointclouds and returns a pointcloud2 message for ros2. Depending on
the settings, the pointcloud can be organized or not.
*/
sensor_msgs::msg::PointCloud2 PointCloudBuffer::getTotalPointcloudROS(
    std::string frame_id) {
    pcl::PointCloud<core::PointXYZIRT>::Ptr pc = getTotalPointcloud();
    sensor_msgs::msg::PointCloud2 cloud_msg;

    if (enable_organize_) {
        cloud_msg = organizePointCloud2(pc, organized_params_);
    } else {
        pcl::toROSMsg(*pc, cloud_msg);
    }
    cloud_msg.header.stamp = getScanTime(0);
    cloud_msg.header.frame_id = frame_id;
    return cloud_msg;
}
}  // namespace rla