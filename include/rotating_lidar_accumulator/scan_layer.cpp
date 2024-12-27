#include "scan_layer.h"


ScanLayer::ScanLayer(
    const sensor_msgs::msg::LaserScan& scan_msg,
    const Eigen::Transform<float, 3, Eigen::Affine>& lidar_offset,
    const Eigen::Vector3f& rotation_axis,
    float angle_rad
    ) : timestamp_(scan_msg.header.stamp), angle_rad_(angle_rad)
{
    pc_ = convertLaserScanToPointCloud(scan_msg);
    if(!pc_ || pc_->empty()) 
    {
        throw std::invalid_argument("Input LaserScan resulted in an empty PointCloud.");
    }
    // First rotation corresponding to the offsets
    pcl::transformPointCloud(* pc_, * pc_, lidar_offset);

    Eigen::Quaternionf rotation_quaternion(
        Eigen::AngleAxisf(angle_rad, rotation_axis.normalized()));
    pcl::transformPointCloud(* pc_, * pc_, Eigen::Vector3f::Zero() , rotation_quaternion);
}

ScanLayer::ScanLayer(const ScanLayer& other)
    : timestamp_(other.timestamp_), angle_rad_(other.angle_rad_),
    pc_(std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*other.pc_))
{}

ScanLayer& ScanLayer::operator=(const ScanLayer& other) 
{
    if(this != &other) {
        // Deep copy of the point cloud
        timestamp_ = other.timestamp_;
        pc_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*other.pc_);
    }
    return *this;
}

ScanLayer::ScanLayer(ScanLayer&& other) noexcept
    : timestamp_(std::move(other.timestamp_)),
      angle_rad_(std::move(other.angle_rad_)),
      pc_(std::move(other.pc_)) // Transfers ownership of the point cloud
{
    // The "other" object is now empty or in a valid but unspecified state.
}

ScanLayer& ScanLayer::operator=(ScanLayer&& other) noexcept
{
    if (this != &other) { // Check for self-assignment
        timestamp_ = std::move(other.timestamp_);
        angle_rad_ = std::move(other.angle_rad_);
        pc_ = std::move(other.pc_); // Transfers ownership
    }
    return *this;
}