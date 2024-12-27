#include "point_cloud_buffer.h"


PointCloudBuffer::PointCloudBuffer()
    : PointCloudBuffer(
        1, 
        Eigen::Transform<float, 3, Eigen::Affine>::Identity(),
        Eigen::Vector3f(0.0f,1.0f,0.0f))
{
}

PointCloudBuffer::PointCloudBuffer(unsigned int num_layers)
    : PointCloudBuffer(
        num_layers, 
        Eigen::Transform<float, 3, Eigen::Affine>::Identity(),
        Eigen::Vector3f(0.0f,1.0f,0.0f))
{
}

PointCloudBuffer::PointCloudBuffer(
        unsigned int num_layers, 
        const Eigen::Transform<float, 3, Eigen::Affine>& lidar_offset, 
        const Eigen::Vector3f& turret_axis)
        : num_layers_(num_layers),
          lidar_offset_(lidar_offset),
          turret_axis_(turret_axis),
          idx_(0)
{
    float n_channels = 32.0;
    organized_params_.h_fov_rad = 2.0 * M_PI;
    organized_params_.h_res_rad = organized_params_.h_fov_rad / 360.0;
    organized_params_.h_min_rad = -organized_params_.h_fov_rad / 2.0;
    organized_params_.v_fov_rad = (30.0 / 180.0 * M_PI);
    organized_params_.v_res_rad = organized_params_.v_fov_rad / n_channels;
    organized_params_.v_min_rad = -organized_params_.v_fov_rad / 2.0;
    reset();
}


void PointCloudBuffer::addScan(
    const sensor_msgs::msg::LaserScan& scan,
    float angle_rad)
{
    addScan(ScanLayer(scan, lidar_offset_, turret_axis_, angle_rad));
}

void PointCloudBuffer::addScan(ScanLayer&& scan)
{
    if(scans_.size() == num_layers_) // at least one scan has been completed
    {
        scans_.at(idx_) = std::move(scan);
    }
    else
    {
        scans_.emplace_back(std::move(scan));
    }
    idx_ = (idx_+1) % num_layers_;
}


void PointCloudBuffer::reset()
{
    scans_.clear();
    idx_ = 0;
}

// GETTERS ---------------------------------------------------------------------------------

/*
Merge all pointclouds stored in the buffer so far
*/
pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudBuffer::getTotalPointcloud()
{
    auto merged_pointcloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

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
Merge all pointclouds and returns a pointcloud2 message for ros2
*/
sensor_msgs::msg::PointCloud2 PointCloudBuffer::getTotalPointcloudROS()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc = getTotalPointcloud();
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*pc, cloud_msg);  

    return cloud_msg;
}

/*
Merge all pointclouds after the corresponding transformation
has been applied into a organized pointcloud message for ros2.
*/
sensor_msgs::msg::PointCloud2 PointCloudBuffer::getTotalOrganizedPointcloudROS()
{

    pcl::PointCloud<pcl::PointXYZI>::Ptr unorganized =  getTotalPointcloud();
    sensor_msgs::msg::PointCloud2 organized = organizePointCloud2(
        unorganized, organized_params_);
    return organized;
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

pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudBuffer::getScan(int index)
{
    return scans_.at(index).getPointCloud();
}
