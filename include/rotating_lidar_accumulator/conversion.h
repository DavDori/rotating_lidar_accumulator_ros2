#ifndef CONVERSION_H
#define CONVERSION_H

#include "rclcpp/rclcpp.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

struct PointCloudOrganizationParams {
    float azim_fov_rad = 0.0f;
    float azim_res_rad = 0.0f;
    float azim_min_rad = 0.0f;
    float elev_fov_rad = 0.0f;
    float elev_res_rad = 0.0f;
    float elev_min_rad = 0.0f;
};

pcl::PointCloud<pcl::PointXYZI>::Ptr convertLaserScanToPointCloud(
    const sensor_msgs::msg::LaserScan& scan_msg
    );

pcl::PointXYZI convertScanRayToPoint(float range, float angle, float intensity);

bool isRangeInvalid(float range, float max_range);

sensor_msgs::msg::PointCloud2 organizePointCloud2(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& unorganized_cloud,
    const PointCloudOrganizationParams& params);

int calculatePointCol(const pcl::PointXYZI& point,  
    const PointCloudOrganizationParams& params);

int calculatePointRow(const pcl::PointXYZI& point,  
    const PointCloudOrganizationParams& params);

std::vector<float> convToFloat(const std::vector<double>& input);

#endif
