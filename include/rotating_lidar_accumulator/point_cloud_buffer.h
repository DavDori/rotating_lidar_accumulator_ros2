#ifndef POINT_CLOUD_BUFFER
#define POINT_CLOUD_BUFFER

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>

#include "scan_layer.h"

class PointCloudBuffer
{
public:
    PointCloudBuffer();
    PointCloudBuffer(
            const Eigen::Transform<float, 3, Eigen::Affine>& lidar_offset, 
            const Eigen::Vector3f& turret_axis
            );
    ~PointCloudBuffer() {}

    void addScan(const sensor_msgs::msg::LaserScan& scan, float angle_rad);
    void reset();
    void enableOrganizedPointcloud(const PointCloudOrganizationParams& p);
    void disableOrganizedPointcloud();
    pcl::PointCloud<PointXYZIRT>::Ptr getTotalPointcloud();
    sensor_msgs::msg::PointCloud2 getTotalPointcloudROS();
    pcl::PointCloud<PointXYZIRT>::Ptr getScan(int index);
    uint8_t getNumLayers() const {return scans_.size();}
    uint8_t getCurrentIndex() const {return idx_;}
    int getRing() const {return ring_;}
    PointCloudOrganizationParams getOrganizationParams() const {return organized_params_;}
    float getMaxLayerAngleRad() const;
    float getMinLayerAngleRad() const;
    void newSweep();

private:
    
    Eigen::Transform<float, 3, Eigen::Affine> lidar_offset_;
    Eigen::Vector3f turret_axis_;
    std::vector<ScanLayer> scans_;
    PointCloudOrganizationParams organized_params_;
    uint8_t idx_ = 0;
    uint8_t ring_ = 0;
    int dir_ = 1;
    bool enable_organize_ = false;
};


#endif