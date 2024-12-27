#include "conversion.h"

pcl::PointCloud<pcl::PointXYZI>::Ptr convertLaserScanToPointCloud(
    const sensor_msgs::msg::LaserScan& scan_msg)
{
    // Extract LaserScan data
    float angle_min = scan_msg.angle_min;
    float angle_increment = scan_msg.angle_increment;

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    // Iterate through LaserScan data and populate PCL PointCloud
    for (size_t i = 0; i < scan_msg.ranges.size(); i++)
    {
        float range = scan_msg.ranges[i];
        float angle = angle_min + i * angle_increment;
        float intensity = scan_msg.intensities[i];
        pcl::PointXYZI point = convertScanRayToPoint(range, angle, intensity);
        pcl_cloud->push_back(point);
    }
    return pcl_cloud;
}

pcl::PointXYZI convertScanRayToPoint(float range, float angle, float intensity)
{
    pcl::PointXYZI point;
    point.x = range * std::cos(angle);
    point.y = range * std::sin(angle);
    point.z = 0.0;  // scan is considerd on a x-y plane
    point.intensity = intensity;
    return point;
}

bool isRangeInvalid(float range, float max_range)
{
    if(range < 0.0)
    {
        throw "Invalid negative range";
    }
    if(max_range <= 0.0)
    {
        throw "Invalid negative or zero max range";
    }
    //treat negative ranges as invalid
    return std::isinf(range) || range > max_range;
}

sensor_msgs::msg::PointCloud2 organizePointCloud2(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& unorganized_cloud,
    const PointCloudOrganizationParams& params)
{

    // Define organized cloud dimensions
    int width = static_cast<int>(std::round(params.h_fov_rad / params.h_res_rad));
    int height = static_cast<int>(std::round(params.v_fov_rad / params.v_res_rad));

    // Initialize the organized PointCloud2 message
    sensor_msgs::msg::PointCloud2 organized_cloud;
    organized_cloud.width = width;
    organized_cloud.height = height;
    organized_cloud.is_dense = false;
    organized_cloud.is_bigendian = false;
    organized_cloud.point_step = 16;  // 4 fields of float32 (x, y, z, intensity)
    organized_cloud.row_step = organized_cloud.point_step * width;

    // Define fields for XYZ and Intensity
    sensor_msgs::PointCloud2Modifier modifier(organized_cloud);
    modifier.setPointCloud2Fields(4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);

    // Resize the cloud to accommodate width * height points
    modifier.resize(width * height);

    // Use iterators to fill in the data for organized_cloud
    sensor_msgs::PointCloud2Iterator<float> iter_x(organized_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(organized_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(organized_cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(organized_cloud, "intensity");


    // Initialize to NaN
    for (int i = 0; i < width * height; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) 
    {
        *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
        *iter_intensity = std::numeric_limits<float>::quiet_NaN();
    }
    // Fill with values
    for (const auto& point : unorganized_cloud->points) 
    {
        int col = calculatePointCol(point, params);
        if (col < 0 || col >= width) {
            continue;
        }
        int row = calculatePointRow(point, params);
        if (row < 0 || row >= height) {
            continue;
        }
        int idx = row * width + col;

        // Move iterators to the correct index
        iter_x = sensor_msgs::PointCloud2Iterator<float>(organized_cloud, "x") + idx;
        iter_y = sensor_msgs::PointCloud2Iterator<float>(organized_cloud, "y") + idx;
        iter_z = sensor_msgs::PointCloud2Iterator<float>(organized_cloud, "z") + idx;
        iter_intensity = sensor_msgs::PointCloud2Iterator<float>(organized_cloud, "intensity") + idx;

        *iter_x = point.x;
        *iter_y = point.y;
        *iter_z = point.z;
        *iter_intensity = point.intensity;
    }
    return organized_cloud;
}

int calculatePointCol(const pcl::PointXYZI& point,  
    const PointCloudOrganizationParams& params)
{
    // Calculate azimuth angle in rad and corresponding column index
    float azimuth_rad = std::atan2(point.y, point.x);
    float azimuth_delta_rad = azimuth_rad - params.h_min_rad;

    int col = static_cast<int>(std::floor(azimuth_delta_rad / params.h_res_rad));
    return col;
}

int calculatePointRow(const pcl::PointXYZI& point,  
    const PointCloudOrganizationParams& params)
{
    // Calculate corresponding row index
    float elevation_rad = std::asin(point.z / std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z));
    float elevation_delta_rad = elevation_rad - params.v_min_rad;

    int row = static_cast<int>(std::floor(elevation_delta_rad / params.v_res_rad));
    return row;
}


std::vector<float> convToFloat(const std::vector<double>& input)
{
    return std::vector<float>(input.begin(), input.end());
}