#pragma once

#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include "rotating_lidar_accumulator/core/duration.hpp"
#include "rotating_lidar_accumulator/core/timestamp.hpp"

namespace utils {

/**
 * @brief Converts a ROS2 Header message to a Timestamp.
 * @param msg ROS2 std_msgs::msg::Header message.
 * @return Time as Timestamp.
 */
[[nodiscard]] core::Timestamp fromROS2(
    const builtin_interfaces::msg::Time& msg);

/**
 * @brief Converts a ROS2 Header message to a Duration.
 * @param msg ROS2 std_msgs::msg::Header message.
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

}  // namespace utils