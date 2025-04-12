#include "point_cloud_buffer.h"


PointCloudBuffer::PointCloudBuffer()
    : PointCloudBuffer(
        Eigen::Transform<float, 3, Eigen::Affine>::Identity(),
        Eigen::Vector3f(0.0f,1.0f,0.0f))
{
}

PointCloudBuffer::PointCloudBuffer(
        const Eigen::Transform<float, 3, Eigen::Affine>& lidar_offset, 
        const Eigen::Vector3f& turret_axis)
        : lidar_offset_(lidar_offset),
          turret_axis_(turret_axis)
{
    reset();
}

void PointCloudBuffer::addScan(
    const sensor_msgs::msg::LaserScan& msg,
    float angle_rad)
{
    if(dir_ < 0 && ring_ == 0) // skip the scan because ring_ must be positive or 0
    {
        return;
    }
    ScanLayer scan(msg, lidar_offset_, turret_axis_, angle_rad, ring_, idx_);
    scans_.emplace_back(std::move(scan));
    idx_++;
    ring_ += dir_;
}

/*
Clear the buffer of all accumulated scans
*/
void PointCloudBuffer::reset()
{
    scans_.clear();
    idx_ = 0;
    ring_ = 0;
    dir_ = 1;
}

/*
When the turret is rotated, the scans are added in the opposite direction.
*/
void PointCloudBuffer::newSweep()
{
    scans_.clear();
    idx_ = 0;
    dir_ = -1 * dir_;
}

void PointCloudBuffer::enableOrganizedPointcloud(const PointCloudOrganizationParams& p)
{
    organized_params_ = p;
    enable_organize_ = true;
}

void PointCloudBuffer::disableOrganizedPointcloud()
{
    enable_organize_ = false;
}

// GETTERS ---------------------------------------------------------------------------------

/*
Merge all pointclouds stored in the buffer so far
*/
pcl::PointCloud<PointXYZIRT>::Ptr PointCloudBuffer::getTotalPointcloud()
{
    auto merged_pointcloud = std::make_shared<pcl::PointCloud<PointXYZIRT>>();

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
sensor_msgs::msg::PointCloud2 PointCloudBuffer::getTotalPointcloudROS()
{
    pcl::PointCloud<PointXYZIRT>::Ptr pc = getTotalPointcloud();
    sensor_msgs::msg::PointCloud2 cloud_msg;

    if(enable_organize_)
    {
        cloud_msg = organizePointCloud2(pc, organized_params_);
    }
    else
    {
        pcl::toROSMsg(*pc, cloud_msg);  
    }
    return cloud_msg;
}

float PointCloudBuffer::getMaxLayerAngleRad() const
{
    float current_candidate = -M_PI;
    for (size_t i = 0; i < scans_.size(); i++)
    {
        if(current_candidate < scans_.at(i).getAngleRad())
        {
            current_candidate = scans_.at(i).getAngleRad();
        }
    }
    return current_candidate;
}

float PointCloudBuffer::getMinLayerAngleRad() const
{
    float current_candidate = M_PI;
    for (size_t i = 0; i < scans_.size(); i++)
    {
        if(current_candidate > scans_.at(i).getAngleRad())
        {
            current_candidate = scans_.at(i).getAngleRad();
        }
    }
    return current_candidate;
}

pcl::PointCloud<PointXYZIRT>::Ptr PointCloudBuffer::getScan(int index)
{
    return scans_.at(index).getPointCloud();
}
