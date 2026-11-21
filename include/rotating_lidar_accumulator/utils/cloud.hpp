#pragma once

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "rotating_lidar_accumulator/core/custom_point_type.hpp"

namespace rla::utils {

struct PointCloudOrganizationParams {
    float azim_fov_rad = 0.0f;
    float azim_res_rad = 0.0f;
    float azim_min_rad = 0.0f;
    float elev_fov_rad = 0.0f;
    float elev_res_rad = 0.0f;
    float elev_min_rad = 0.0f;
};

[[nodiscard]] sensor_msgs::msg::PointCloud2 organizePointCloud2(
    const pcl::PointCloud<core::PointXYZIRT>::Ptr& unorganized_cloud,
    const PointCloudOrganizationParams& params);

[[nodiscard]] std::vector<float> convToFloat(const std::vector<double>& input);

}  // namespace rla::utils
