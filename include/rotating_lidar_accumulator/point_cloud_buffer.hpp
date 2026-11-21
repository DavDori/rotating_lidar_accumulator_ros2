#pragma once

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>

#include "rotating_lidar_accumulator/scan_layer.hpp"
#include "rotating_lidar_accumulator/utils/cloud.hpp"

namespace rla {
class PointCloudBuffer {
   public:
    PointCloudBuffer();
    PointCloudBuffer(
        const Eigen::Transform<float, 3, Eigen::Affine>& lidar_offset,
        const Eigen::Vector3f& turret_axis);
    ~PointCloudBuffer() {}

    void addScan(const sensor_msgs::msg::LaserScan& msg, uint32_t scan_duration,
                 float angle_rad);
    void addScan(const sensor_msgs::msg::LaserScan& msg, uint32_t scan_duration,
                 Eigen::Transform<float, 3, Eigen::Affine>& lidar_pose);
    void reset();
    void enableOrganizedPointcloud(
        const utils::PointCloudOrganizationParams& p);
    void disableOrganizedPointcloud();

    pcl::PointCloud<core::PointXYZIRT>::Ptr getTotalPointcloud();
    sensor_msgs::msg::PointCloud2 getTotalPointcloudROS(std::string frame_id);
    uint8_t getNumLayers() const { return scans_.size(); }
    uint8_t getCurrentIndex() const { return idx_; }
    utils::PointCloudOrganizationParams getOrganizationParams() const {
        return organized_params_;
    }
    rclcpp::Time getFirstTimestamp() const { return scans_.at(0).getTime(); }

   private:
    rclcpp::Time getScanTime(uint8_t idx) const {
        return scans_.at(idx).getTime();
    }

    Eigen::Transform<float, 3, Eigen::Affine> lidar_offset_;
    Eigen::Vector3f turret_axis_;

    std::vector<ScanLayer> scans_;
    utils::PointCloudOrganizationParams organized_params_;
    uint8_t idx_{0};
    bool enable_organize_ = false;
};
}  // namespace rla