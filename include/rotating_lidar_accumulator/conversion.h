#ifndef CONVERSION_H
#define CONVERSION_H

#include "rclcpp/rclcpp.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

struct PointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

struct PointCloudOrganizationParams {
    float azim_fov_rad = 0.0f;
    float azim_res_rad = 0.0f;
    float azim_min_rad = 0.0f;
    float elev_fov_rad = 0.0f;
    float elev_res_rad = 0.0f;
    float elev_min_rad = 0.0f;
};

pcl::PointCloud<PointXYZIRT>::Ptr convertLaserScanToPointCloud(
    const sensor_msgs::msg::LaserScan& scan_msg,
    int ring_id,
    int num_scan);

inline PointXYZIRT convertScanRayToPoint(float range, float angle, float intensity);

inline bool isRangeInvalid(float range, float max_range);

sensor_msgs::msg::PointCloud2 organizePointCloud2(
    const pcl::PointCloud<PointXYZIRT>::Ptr& unorganized_cloud,
    const PointCloudOrganizationParams& params);

inline int calculatePointCol(const PointXYZIRT& point,  
    const PointCloudOrganizationParams& params);

inline int calculatePointRow(const PointXYZIRT& point,  
    const PointCloudOrganizationParams& params);

std::vector<float> convToFloat(const std::vector<double>& input);

#endif
