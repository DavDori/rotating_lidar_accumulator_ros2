#include "rotating_lidar_accumulator/utils/conversion_ros2.hpp"

#include <cmath>

namespace rla::utils {

namespace {

core::PointXYZIRT convertScanRayToPoint(float range, float angle,
                                        float intensity) {
    core::PointXYZIRT point{};
    point.x = range * std::cos(angle);
    point.y = range * std::sin(angle);
    point.z = 0.0F;  // scan is considered on a x-y plane
    point.intensity = intensity;
    return point;
}

}  // namespace

core::Timestamp fromROS2(const builtin_interfaces::msg::Time& msg) {
    core::Timestamp t(msg.sec, msg.nanosec);
    return t;
}

core::Duration fromROS2(const builtin_interfaces::msg::Duration& msg) {
    core::Duration t(msg.sec, msg.nanosec);
    return t;
}

builtin_interfaces::msg::Time toROS2(const core::Timestamp& t) {
    builtin_interfaces::msg::Time msg;
    msg.sec = static_cast<int32_t>(t.getSeconds());
    msg.nanosec = static_cast<uint32_t>(t.getNanoseconds());
    return msg;
}

std::optional<pcl::PointCloud<core::PointXYZIRT>> convertLaserScanToPointCloud(
    const sensor_msgs::msg::LaserScan& scan_msg, const ScanInfo& info) {
    if (scan_msg.ranges.empty()) {
        return std::nullopt;
    }
    // Does not work without intensities.
    const size_t n = scan_msg.ranges.size();
    if (scan_msg.intensities.empty() || scan_msg.intensities.size() != n) {
        return std::nullopt;
    }
    // Would overflow.
    if (uint64_t{info.rel_scan_start} + info.scan_duration > UINT32_MAX) {
        return std::nullopt;
    }
    const float angle_min = scan_msg.angle_min;
    const float angle_increment = scan_msg.angle_increment;

    pcl::PointCloud<core::PointXYZIRT> pcl_cloud;
    pcl_cloud.reserve(n);
    // Iterate through LaserScan data and populate PCL PointCloud
    for (uint32_t i = 0; i < static_cast<uint32_t>(n); ++i) {
        if (!(scan_msg.ranges[i] >= scan_msg.range_min &&
              scan_msg.ranges[i] <= scan_msg.range_max)) {
            continue;
        }
        core::PointXYZIRT point = convertScanRayToPoint(
            scan_msg.ranges[i], angle_min + i * angle_increment,
            scan_msg.intensities[i]);
        point.ring = info.ring_id;
        point.t = info.rel_scan_start +
                  static_cast<uint32_t>(static_cast<uint64_t>(i) *
                                        info.scan_duration / n);
        pcl_cloud.push_back(point);
    }
    return pcl_cloud;
}

}  // namespace rla::utils