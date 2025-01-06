#include "scan_layer.h"

/**
 * @brief Converts a raw LiDAR scan message into a transformed point cloud, 
 *        applying both a static offset and a varying rotation based on the 
 *        additional rotation of a moving body.
 * 
 * This constructor processes a ROS2 LaserScan message to generate a point cloud 
 * and applies transformations to account for the following:
 * 
 * 1. **Lidar Offset Transformation**:
 *    The LiDAR is mounted on a rotor of an additional rotating body, and 
 *    `lidar_offset` represents the fixed transformation between the LiDAR's 
 *    coordinate frame and the rotor's frame. This transformation is applied 
 *    uniformly to all points in the scan.
 * 
 * 2. **Dynamic Rotations**:
 *    As the additional rotating body spins, each point in the scan requires 
 *    an additional rotation to map it into the global coordinate frame. The 
 *    rotation axis is defined by `rotation_axis`, and the angle increment 
 *    for subsequent points is determined by `delta_psi_rad`.
 * 
 * @param scan_msg A `sensor_msgs::msg::LaserScan` containing raw LiDAR scan data.
 * @param lidar_offset An `Eigen::Transform` representing the fixed offset between 
 *        the LiDAR and the rotor.
 * @param rotation_axis An `Eigen::Vector3f` defining the axis of rotation for the 
 *        additional rotating body (should be normalized).
 * @param initial_psi_rad A float representing the initial angle (in radians) at which 
 *        the scan started.
 * @param delta_psi_rad A float representing the rotation angle increment (in radians) 
 *        between two consecutive points in the scan.
 * 
 * @throws std::invalid_argument If the input scan results in an empty point cloud.
 */
ScanLayer::ScanLayer(
    const sensor_msgs::msg::LaserScan& scan_msg,
    const Eigen::Transform<float, 3, Eigen::Affine>& lidar_offset,
    const Eigen::Vector3f& rotation_axis,
    float initial_psi_rad,
    float delta_psi_rad
    ) : timestamp_(scan_msg.header.stamp), angle_rad_(initial_psi_rad)
{
    pc_ = convertLaserScanToPointCloud(scan_msg);
    if(!pc_ || pc_->empty()) 
    {
        throw std::invalid_argument("Input LaserScan resulted in an empty PointCloud.");
    }
    // First rotation corresponding to the offsets
    pcl::transformPointCloud(* pc_, * pc_, lidar_offset);

    float psi = initial_psi_rad;
    // Iterate through each point and apply the rotation
    for (auto& point : pc_->points)
    {
        // Create a rotation quaternion based on the current angle
        Eigen::Quaternionf rotation_quaternion(
            Eigen::AngleAxisf(psi, rotation_axis));
        
        Eigen::Vector3f point_vector(point.x, point.y, point.z);
        point_vector = rotation_quaternion * point_vector;

        // Update the point with the rotated coordinates
        point.x = point_vector.x();
        point.y = point_vector.y();
        point.z = point_vector.z();

        // Increment the angle for the next point
        psi += delta_psi_rad;
    }
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