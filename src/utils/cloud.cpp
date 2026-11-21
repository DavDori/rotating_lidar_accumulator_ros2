#include "rotating_lidar_accumulator/utils/cloud.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace rla::utils {
namespace {
int calculatePointColumn(const core::PointXYZIRT& point,
                         const PointCloudOrganizationParams& params) {
    // Calculate azimuth angle in rad and corresponding column index
    float azim_rad = std::atan2(point.y, point.x);
    float azim_delta_rad = azim_rad - params.azim_min_rad;

    int col =
        static_cast<int>(std::floor(azim_delta_rad / params.azim_res_rad));
    return col;
}

int calculatePointRow(const core::PointXYZIRT& point,
                      const PointCloudOrganizationParams& params) {
    // Calculate corresponding row index
    float elev_rad =
        std::asin(point.z / std::sqrt(point.x * point.x + point.y * point.y +
                                      point.z * point.z));
    float elev_delta_rad = elev_rad - params.elev_min_rad;

    int row =
        static_cast<int>(std::floor(elev_delta_rad / params.elev_res_rad));
    return row;
}
}  // namespace

sensor_msgs::msg::PointCloud2 organizePointCloud2(
    const pcl::PointCloud<core::PointXYZIRT>::Ptr& unorganized_cloud,
    const PointCloudOrganizationParams& params) {
    // Define organized cloud dimensions
    int width =
        static_cast<int>(std::round(params.azim_fov_rad / params.azim_res_rad));
    int height =
        static_cast<int>(std::round(params.elev_fov_rad / params.elev_res_rad));

    // Initialize the organized PointCloud2 message
    sensor_msgs::msg::PointCloud2 organized_cloud;
    organized_cloud.width = width;
    organized_cloud.height = height;
    organized_cloud.is_dense = false;
    organized_cloud.is_bigendian = false;
    organized_cloud.point_step =
        21;  // 5 fields of float32 (x, y, z, intensity, time) + ring of uint8
    organized_cloud.row_step = organized_cloud.point_step * width;

    // Define fields for XYZ and Intensity
    sensor_msgs::PointCloud2Modifier modifier(organized_cloud);
    modifier.setPointCloud2Fields(
        4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
        sensor_msgs::msg::PointField::FLOAT32, "z", 1,
        sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
        sensor_msgs::msg::PointField::FLOAT32, "ring", 1,
        sensor_msgs::msg::PointField::UINT8, "time", 1,
        sensor_msgs::msg::PointField::FLOAT32);

    // Resize the cloud to accommodate width * height points
    modifier.resize(width * height);

    // Use iterators to fill in the data for organized_cloud
    sensor_msgs::PointCloud2Iterator<float> iter_x(organized_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(organized_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(organized_cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(organized_cloud,
                                                           "intensity");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_ring(organized_cloud,
                                                        "ring");
    sensor_msgs::PointCloud2Iterator<float> iter_time(organized_cloud, "time");

    // Initialize to NaN
    for (int i = 0; i < width * height;
         ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
        *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
        *iter_intensity = std::numeric_limits<float>::quiet_NaN();
        *iter_ring = std::numeric_limits<uint8_t>::quiet_NaN();
        *iter_time = std::numeric_limits<float>::quiet_NaN();
    }
    // Fill with values
    for (const auto& point : unorganized_cloud->points) {
        int col = calculatePointColumn(point, params);
        if (col < 0 || col >= width) {
            continue;
        }
        int row = calculatePointRow(point, params);
        if (row < 0 || row >= height) {
            continue;
        }
        int idx = row * width + col;

        // Move iterators to the correct index
        iter_x =
            sensor_msgs::PointCloud2Iterator<float>(organized_cloud, "x") + idx;
        iter_y =
            sensor_msgs::PointCloud2Iterator<float>(organized_cloud, "y") + idx;
        iter_z =
            sensor_msgs::PointCloud2Iterator<float>(organized_cloud, "z") + idx;
        iter_intensity = sensor_msgs::PointCloud2Iterator<float>(
                             organized_cloud, "intensity") +
                         idx;

        *iter_x = point.x;
        *iter_y = point.y;
        *iter_z = point.z;
        *iter_intensity = point.intensity;
        *iter_ring = point.ring;
        *iter_time = point.t;
    }
    return organized_cloud;
}

}  // namespace rla::utils
