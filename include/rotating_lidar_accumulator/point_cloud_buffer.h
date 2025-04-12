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
    PointCloudBuffer(unsigned int num_layers);
    PointCloudBuffer(
            unsigned int num_layers, 
            const Eigen::Transform<float, 3, Eigen::Affine>& lidar_offset, 
            const Eigen::Vector3f& turret_axis
            );
    ~PointCloudBuffer() {}

    void addScan(const sensor_msgs::msg::LaserScan& scan, float angle_rad);
    void addScan(ScanLayer&& scan);
    void reset();
    void enableOrganizedPointcloud(const PointCloudOrganizationParams& p);
    void disableOrganizedPointcloud();
    pcl::PointCloud<PointXYZIRT>::Ptr getTotalPointcloud();
    sensor_msgs::msg::PointCloud2 getTotalPointcloudROS();
    unsigned int getNumLayers() const {return scans_.size();}
    pcl::PointCloud<PointXYZIRT>::Ptr getScan(int index);
    PointCloudOrganizationParams getOrganizationParams() const {return organized_params_;}
    float getMaxLayerAngleRad() const;
    float getMinLayerAngleRad() const;

private:
    
    unsigned int num_layers_;
    Eigen::Transform<float, 3, Eigen::Affine> lidar_offset_;
    Eigen::Vector3f turret_axis_;
    std::vector<ScanLayer> scans_;
    PointCloudOrganizationParams organized_params_;
    unsigned int idx_;
    bool enable_organize_ = false;
};


#endif