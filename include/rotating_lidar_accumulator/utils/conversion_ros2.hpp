#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <cstdint>
#include <optional>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "rotating_lidar_accumulator/core/custom_point_type.hpp"
#include "rotating_lidar_accumulator/core/duration.hpp"
#include "rotating_lidar_accumulator/core/timestamp.hpp"

namespace rla::utils {

/**
 * @brief Collection of informations needed for the conversion that are not in
 * the laser_scan message.
 */
struct ScanInfo {
    uint32_t rel_scan_start;  //!< Relative time start of the laser scan (micro
                              //!< seconds). Since the purpose is accumulation,
                              //!< this should be wrt the first scan added.
    uint32_t
        scan_duration;  //!< Time duration of the laser scan (micro seconds).
    uint16_t ring_id;   //!< Assign this ring id to the points.
};

/**
 * @brief Converts a ROS2 Header message to a Timestamp.
 * @param msg ROS2  builtin_interfaces::msg::Time message.
 * @return Time as Timestamp.
 */
[[nodiscard]] core::Timestamp fromROS2(
    const builtin_interfaces::msg::Time& msg);

/**
 * @brief Converts a ROS2 Header message to a Duration.
 * @param msg ROS2  builtin_interfaces::msg::Time message.
 * @return Time as Duration.
 */
[[nodiscard]] core::Duration fromROS2(
    const builtin_interfaces::msg::Duration& msg);

/**
 * @brief Convert a timestamp to ROS2 message.
 * @param t Time as Timestamp.
 * @return Time as builtin_interfaces::msg::Time .
 */
[[nodiscard]] builtin_interfaces::msg::Time toROS2(const core::Timestamp& t);

/**
 * @brief Convert a laser scan into a point cloud.
 * @note Each point cloud's point has 3d position, intensity, ring, time
 * information.
 * @param scan_msg Laser scan message to convert.
 * @param info Information needed for the conversion that are not in the
 * message.
 * @return Return an optional with the point cloud as the result of the
 * conversion, or a std::nullopt if empty scan or error during conversion.
 */
[[nodiscard]] std::optional<pcl::PointCloud<core::PointXYZIRT>>
convertLaserScanToPointCloud(const sensor_msgs::msg::LaserScan& scan_msg,
                             const ScanInfo& info);

}  // namespace rla::utils