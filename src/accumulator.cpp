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
#include "std_msgs/msg/bool.hpp" 
// Custom
#include "rotating_lidar_accumulator/point_cloud_buffer.h"
// Mutex for thread safety
#include <mutex>
// Tf2
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2_eigen/tf2_eigen.hpp>


std::atomic<bool> is_new_sweep;

class PointCloudAccumulator : public rclcpp::Node
{
    using AffineTransform3f = Eigen::Transform<float, 3, Eigen::Affine>;

public:
    PointCloudAccumulator() : Node("point_cloud_accumulator")
    {
        declare_parameter("mech.lidar_frame_id", "/trip/lidar_pitch_link");
        declare_parameter("topic.in.scan", "/scan");
        declare_parameter("topic.in.trigger", "/new_sweep");
        declare_parameter("topic.out.cloud", "/turret/cloud");

        declare_parameter("pointcloud.frame_id", "/lidar_base");
        declare_parameter("pointcloud.max_scans", 4);
        declare_parameter("pointcloud.enable_organized", false);
        declare_parameter("pointcloud.azimuth.res_deg", 0.225);
        declare_parameter("pointcloud.azimuth.fov_deg", 360.0);
        declare_parameter("pointcloud.azimuth.min_deg", -180.0);
        declare_parameter("pointcloud.elevation.res_deg", 1.0);
        declare_parameter("pointcloud.elevation.fov_deg", 30.0);
        declare_parameter("pointcloud.elevation.min_deg", -15.0);


        std::string topic_in_scan = get_parameter("topic.in.scan").as_string();
        std::string topic_in_trigger = get_parameter("topic.in.trigger").as_string();
        std::string topic_out_cloud = get_parameter("topic.out.cloud").as_string();

        const bool en_organzied_pointcloud = get_parameter("pointcloud.enable_organized").as_bool();
        lidar_frame_id_ = get_parameter("mech.lidar_frame_id").as_string();
        base_frame_id_ = get_parameter("pointcloud.frame_id").as_string();

        max_num_scans_ = static_cast<uint8_t>(get_parameter("pointcloud.max_scans").as_int());

        // Initialize point cloud buffer
        buff_ = PointCloudBuffer();

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

        trigger_sub_ = create_subscription<std_msgs::msg::Bool>(
            topic_in_trigger,
            10,
            std::bind(&PointCloudAccumulator::triggerSweepCallback, this, std::placeholders::_1));

        point_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            topic_out_cloud, 5);
        
        tf_buffer_ =
            std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        std::ostringstream general_params;
        general_params << "\n--- Parameters ---\n"
            << "  - Topic in scan: " << topic_in_scan << "\n"
            << "  - Topic in trigger: "         << topic_in_trigger << "\n"
            << "  - Topic out pointcloud: " << topic_out_cloud << "\n"
            << "  - Pointcloud output frame id: " << base_frame_id_ << "\n"
            << "  - Scan input frame id: " << lidar_frame_id_ << "\n"
            << "  - Collecting up to: " << static_cast<int>(max_num_scans_) << " scans" << "\n"
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
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_ { nullptr };
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string base_frame_id_;
    std::string lidar_frame_id_;

    rclcpp::Time last_angle_time_;
    PointCloudBuffer buff_;
    mutable std::mutex mutex_;
    uint8_t max_num_scans_;

//--CALLBACKS------------------------------------------------------------------------
    
    void laserCallback(const sensor_msgs::msg::LaserScan& msg)
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "Received a laser scan message");
        // Look up for the transformation between target_frame and turtle2 frames
        // and send velocity commands for turtle2 to reach target_frame
        AffineTransform3f lidar_tform;
        try 
        {
            geometry_msgs::msg::TransformStamped t = 
                tf_buffer_->lookupTransform( base_frame_id_, lidar_frame_id_, msg.header.stamp, rclcpp::Duration(1,0));
            lidar_tform = tf2::transformToEigen(t).cast<float>();
        } 
        catch (const tf2::TransformException & ex) 
        {
            RCLCPP_INFO(
                this->get_logger(), "Could not transform %s to %s: %s",
                lidar_frame_id_.c_str(), base_frame_id_.c_str(), ex.what()
            );
            return;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(is_new_sweep.load())
            {
                is_new_sweep.store(false);
            }
            buff_.addScan(msg, lidar_tform);

            bool trigger_fired = is_new_sweep.exchange(false);
            bool count_exceeded = buff_.getNumLayers() > max_num_scans_;

            if (trigger_fired || count_exceeded)
            {
                publishPointcloud();
                buff_.reset();
            }
        }
    }

    void triggerSweepCallback(const std_msgs::msg::Bool& msg)
    {
        if (msg.data) 
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "New sweep triggered!");
            is_new_sweep.store(true);
        }
    }

    
    void publishPointcloud()
    {
        try 
        {
            auto cloud_msg = buff_.getTotalPointcloudROS(base_frame_id_);
        
            cloud_msg.header.frame_id = base_frame_id_;

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
    auto node = std::make_shared<PointCloudAccumulator>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}