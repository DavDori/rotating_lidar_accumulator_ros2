#include <chrono>
#include <functional>
#include <memory>
#include <string>
// Matrix/vector utils
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>
// Point cloud utils
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// ROS2 utils
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
// Custom
#include "rotating_lidar_accumulator/point_cloud_buffer.h"


bool hasVelocityChanged(double vel_prev, double vel)
{
    return vel_prev * vel < 0.0;
}

class PointCloudAccumulator : public rclcpp::Node
{
    using AffineTransform3f = Eigen::Transform<float, 3, Eigen::Affine>;

public:
    PointCloudAccumulator() : Node("point_cloud_accumulator")
    {
        declare_parameter("mech.lidar_offset_xyz_m", std::vector<float>{0.0f,0.0f,0.0f});
        declare_parameter("mech.lidar_offset_ypr_deg", std::vector<float>{0.0f,0.0f,0.0f});
        declare_parameter("mech.rotation_axis", std::vector<float>{0.0f,1.0f,0.0f});
        declare_parameter("topic.in.scan", "/scan");
        declare_parameter("topic.in.angle", "/turret/angle");
        declare_parameter("topic.out.cloud", "/turret/cloud");

        declare_parameter("pointcloud.enable_organized", false);
        declare_parameter("pointcloud.frame_id", "/lidar_base");
        declare_parameter("pointcloud.max_num_layers", 200);
        declare_parameter("pointcloud.azimuth.res_deg", 0.225);
        declare_parameter("pointcloud.azimuth.fov_deg", 360.0);
        declare_parameter("pointcloud.azimuth.min_deg", -180.0);
        declare_parameter("pointcloud.elevation.res_deg", 1.0);
        declare_parameter("pointcloud.elevation.fov_deg", 30.0);
        declare_parameter("pointcloud.elevation.min_deg", -15.0);


        unsigned int max_layers = get_parameter("pointcloud.max_num_layers").as_int();
        bool en_organzied_pointcloud = get_parameter("pointcloud.enable_organized").as_bool();
        std::string topic_in_scan = get_parameter("topic.in.scan").as_string();
        std::string topic_in_angle = get_parameter("topic.in.angle").as_string();
        std::string topic_out_cloud = get_parameter("topic.out.cloud").as_string();

        frame_id_ = get_parameter("pointcloud.frame_id").as_string();
        // Initialize point cloud buffer
        std::vector<float> rotation_axis_vec = 
            convToFloat(get_parameter("mech.rotation_axis").as_double_array());
        Eigen::Vector3f rotation_axis(rotation_axis_vec.data());

        AffineTransform3f tform_offset = computeOffsetTransformation();
        buff_ = PointCloudBuffer(max_layers, tform_offset, rotation_axis);

        PointCloudOrganizationParams params;
        params.azim_res_rad = (float) 
            DEG2RAD(get_parameter("pointcloud.azimuth.res_deg").as_double());
        params.azim_fov_rad = (float) 
            DEG2RAD(get_parameter("pointcloud.azimuth.fov_deg").as_double());
        params.azim_min_rad = (float)
            DEG2RAD(get_parameter("pointcloud.azimuth.min_deg").as_double());
        params.elev_res_rad = (float) 
            DEG2RAD(get_parameter("pointcloud.elevation.res_deg").as_double());
        params.elev_fov_rad = (float) 
            DEG2RAD(get_parameter("pointcloud.elevation.fov_deg").as_double());
        params.elev_min_rad = (float) 
            DEG2RAD(get_parameter("pointcloud.elevation.min_deg").as_double());

        if(en_organzied_pointcloud)
        {
            buff_.enableOrganizedPointcloud(params);
        }
        // Set Quality Of Service for subscribers as sensor to prioritize the most
        // recent message, and improve reliability + low latency
        auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
        // Initialize subscriptions
        laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            topic_in_scan, 
            sensor_qos, 
            std::bind(&PointCloudAccumulator::laserCallback, this, std::placeholders::_1));

        angle_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            topic_in_angle, 
            sensor_qos,
            std::bind(&PointCloudAccumulator::angleCallback, this, std::placeholders::_1));

        point_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            topic_out_cloud, 5);

        last_angle_time_ = this->get_clock()->now();
        scan_angle_rad_ = 0.0;
        scan_vel_radps_ = 0.0;

        std::vector<double> offsets_xyz = get_parameter("mech.lidar_offset_xyz_m").as_double_array();
        std::vector<double> offsets_ypr = get_parameter("mech.lidar_offset_ypr_deg").as_double_array();

        std::ostringstream general_params;
        general_params << "\n--- Parameters ---\n"
            << "  - LiDAR offset:\n"
            << "      * xyz: [" << offsets_xyz[0] << ", " << offsets_xyz[1] << ", " << offsets_xyz[2] << "] m\n"
            << "      * ypr: [" << offsets_ypr[0] << ", " << offsets_ypr[1] << ", " << offsets_ypr[2] << "] deg\n"
            << "  - Turret rotation axis: [" << rotation_axis_vec[0] << ", " << rotation_axis_vec[1] << ", " << rotation_axis_vec[2] << "]\n"
            << "  - Topic in scan: " << topic_in_scan << "\n"
            << "  - Topic in angle: " << topic_in_angle << "\n"
            << "  - Topic out pointcloud: " << topic_out_cloud << "\n"
            << "  - Pointcloud output frame id: " << frame_id_ << "\n"
            << "  - Max number of layers: " << max_layers << "\n"
            << "  - Using organized pointcloud: " << (en_organzied_pointcloud ? "true" : "false") << "\n";

        RCLCPP_INFO(this->get_logger(), "%s", general_params.str().c_str());

        // Second string for organized pointcloud parameters
        if (en_organzied_pointcloud) {
            std::ostringstream organized_params;
            organized_params << "\n--- Point Cloud Organization Parameters ---\n"
                << "  - Azimuth\n"
                << "      * resolution: " << RAD2DEG(params.azim_res_rad) << " deg\n"
                << "      * FOV: " << RAD2DEG(params.azim_fov_rad) << " deg\n"
                << "      * start angle: " << RAD2DEG(params.azim_min_rad) << " deg\n"
                << "  - Elevation\n"
                << "      * resolution: " << RAD2DEG(params.elev_res_rad) << " deg\n"
                << "      * FOV: " << RAD2DEG(params.elev_fov_rad) << " deg\n"
                << "      * start angle: " << RAD2DEG(params.elev_min_rad) << " deg\n";

            RCLCPP_INFO(this->get_logger(), "%s", organized_params.str().c_str());
        }
        RCLCPP_INFO(this->get_logger(), "PointCloudAccumulator initialized successfully.");
    }

  private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr angle_sub_;
    std::string frame_id_;

    rclcpp::Time last_angle_time_;
    PointCloudBuffer buff_;

    double scan_angle_rad_;
    double scan_vel_radps_;

    bool is_new_sweep_ = false;

//--CALLBACKS------------------------------------------------------------------------
    
    void laserCallback(const sensor_msgs::msg::LaserScan& msg)
    {
        try{
            if(is_new_sweep_)
            {
                publishPointcloud();
                RCLCPP_INFO(
                    this->get_logger(), 
                    "Published pointcloud made of %u layers", 
                    buff_.getNumLayers());
                buff_.reset();
                is_new_sweep_ = false;
            }

            rclcpp::Time current_time = this->get_clock()->now();
            rclcpp::Duration time_difference = current_time - last_angle_time_;
            double dt_s = time_difference.seconds();

            double current_angle_rad = predictScanAngle(dt_s);
            buff_.addScan(msg, current_angle_rad);
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error in scanCallback: %s", e.what());
        }
    }

    void angleCallback(const sensor_msgs::msg::JointState& msg)
    {
        if(msg.position.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received JointState message with no position data");
            return;
        }
        if(msg.velocity.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received JointState message with no velocity data");
            return;
        }
        if(hasVelocityChanged(scan_vel_radps_, msg.velocity[0]))
        {
            is_new_sweep_ = true;
        }
        
        // Update values
        scan_angle_rad_ = msg.position[0];
        scan_vel_radps_ = msg.velocity[0];
        // Use the time of arrival as reference
        last_angle_time_ = this->get_clock()->now();
    }
    
//-----------------------------------------------------------------------------------------------

    AffineTransform3f computeOffsetTransformation()
    {
        std::vector<float> offsets_xyz = convToFloat(get_parameter("mech.lidar_offset_xyz_m").as_double_array());
        std::vector<float> offsets_ypr = convToFloat(get_parameter("mech.lidar_offset_ypr_deg").as_double_array());


        if(offsets_xyz.empty() || offsets_ypr.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received offsets with no data. Returning Identity offset transofmration.");
            return AffineTransform3f::Identity();
        }

        // Angles need to be converted from degress to radiants
        float yaw_rad = offsets_ypr[0] * M_PI / 180.0;
        float pitch_rad = offsets_ypr[1] * M_PI / 180.0;
        float roll_rad = offsets_ypr[2] * M_PI / 180.0;

        // Create a rotation matrix from Euler YPR Sequence
        Eigen::Matrix3f rotation_matrix;
        rotation_matrix = 
            Eigen::AngleAxisf(yaw_rad, Eigen::Vector3f::UnitZ()) *
            Eigen::AngleAxisf(pitch_rad, Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(roll_rad, Eigen::Vector3f::UnitX());
        
        Eigen::Vector3f translation(offsets_xyz.data());
        AffineTransform3f tform_offsets =
            AffineTransform3f::Identity();
        tform_offsets.translate(translation);
        tform_offsets.rotate(rotation_matrix);

        return tform_offsets;
    }


    double predictScanAngle(double dt)
    {
        // With the assumption of constant velocity, calculate the angle by Euler
        // integration.
        if(dt < 0.0)
        {
            RCLCPP_WARN(this->get_logger(),
                "Negative time interval (dt = %f) provided for integration. Returning current scan angle value without integration.", 
                dt);
            return scan_angle_rad_;
        }
        return scan_angle_rad_ + scan_vel_radps_ * dt;
    }

    void publishPointcloud()
    {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        try 
        {
            cloud_msg = buff_.getTotalPointcloudROS();
        
            cloud_msg.header.frame_id = frame_id_;
            cloud_msg.header.stamp = this->get_clock()->now();

            point_cloud_pub_->publish(cloud_msg);
            RCLCPP_DEBUG(this->get_logger(), "Published point cloud with %d points", cloud_msg.width * cloud_msg.height);
        }
        catch (const std::exception& e) 
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to publish point cloud: %s", e.what());
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudAccumulator>());
    rclcpp::shutdown();
    return 0;
}
