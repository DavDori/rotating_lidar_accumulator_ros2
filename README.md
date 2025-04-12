# Rotating 2D LiDAR scan accumulator for ROS2

## Overview

This ROS 2 package, processes data from a back-and-forth rotating 2D LiDAR and generates a 3D point cloud. It subscribes to LaserScan and JointState topics, accumulates LiDAR scans as layers, and publishes a consolidated point cloud. It is designed for use with systems that feature a rotating 2D LiDAR.

## Features

- Converts 2D LiDAR scans into 3D point clouds.
- Supports organized and unorganized point cloud outputs.
- Configurable mechanical offsets and rotation parameters.
- Handles scan velocity changes to detect new sweeps.
- Publishes point clouds in the `sensor_msgs/msg/PointCloud2` format.

## Dependencies

This project requires the following libraries:
- [Eigen3](https://eigen.tuxfamily.org): For matrix and vector operations.
- [PCL (Point Cloud Library)](https://pointclouds.org/): For point cloud processing.
- [ROS 2 Humble](https://docs.ros.org/en/humble/index.html): For robot middleware and communication.

## Run the node

The node can be used with many configurations of a rotating back-and-forth LiDAR.

### Launch with `conf0` Mode

In the **conf0** configuration, the LiDAR is tilted of a 15 degrees and rotates along the vertical axis. It generates a point cloud similar to that of a mechanical 3D LiDAR (e.g., Velodyne Puck VLP-32).

<p float="left">
    <img src="./docs/conf0.gif" alt="drawing" width="55%"/>
    <img src="./docs/conf0.png" alt="drawing" width="43.3%"/>
</p>
Launch the node using the parameters in `config/conf0.yaml`:

```bash
ros2 launch rotating_lidar_accumulator accumulator_conf0.launch.py
```

### Launch with `conf1` Mode

In the **conf1** configuration, the LiDAR rotates along the x-axis in a up-and-down motion. It generates a point cloud with a high density of points along the additional rotation axis.

<p float="left">
    <img src="./docs/conf1.gif" alt="drawing" width="55%"/>
    <img src="./docs/conf1.png" alt="drawing" width="43.3%"/>
</p>

To launch the node with the parameters in `config/conf1.yaml`, run:

```bash
ros2 launch rotating_lidar_accumulator accumulator_conf1.launch.py
```

## Parameters

| Parameter Name                   | Description                                         | Default Value                     |
|----------------------------------|-----------------------------------------------------|-----------------------------------|
| `mech.lidar_offset_xyz_m`        | LiDAR positional offset in meters (x, y, z).       | `[0.0, 0.0, 0.0]`|
| `mech.lidar_offset_ypr_deg`      | LiDAR orientation offset in degrees (yaw, pitch, roll). | `[0.0, 0.0, 0.0]`|
| `mech.rotation_axis`             | Axis of rotation for the LiDAR (x, y, z).          | `[0.0, 1.0, 0.0]`|
| `pointcloud.topic.out.cloud`     | Name of the output topic.                           | `/turret/cloud`|
| `pointcloud.topic.in.angle`     | Name of the input topic where the turret angle is published.                           | `/turret/angle`|
| `pointcloud.topic.in.scan`     | Name of the input topic where the LiDAR scan is published                           | `/scan`|
| `pointcloud.frame_id`           | Name of the frame id of the output pointcloud.                           | `lidar_base`|
| `pointcloud.organized`             | Generate organized point clouds (true/false).      | `false`|
| `pointcloud.azimuth.res_deg` | Azimuth angular resolution of the organized pointcloud in degrees.    | `0.225`|
| `pointcloud.azimuth.fov_deg` | Azimuth Field of View (FOV) of the organized pointcloud in degrees.    | `360.0`|
| `pointcloud.azimuth.min_deg` | Starting angle of the Azimuth FOV of the organized pointcloud in degrees.    | `-180.0`|
| `pointcloud.elevation.res_deg` | Elevation angular resolution of the organized pointcloud in degrees.    | `1.0`|
| `pointcloud.elevation.fov_deg` | Elevation Field of View (FOV) of the organized pointcloud in degrees.    | `30.0`|
| `pointcloud.elevation.min_deg` | Starting angle of the Elevation FOV of the organized pointcloud in degrees.    | `-15.0`|

## Topics

### Subscribed Topics

| Topic Name       | Message Type                         | Description                                      |
|------------------|--------------------------------------|--------------------------------------------------|
| `/scan`          | `sensor_msgs/msg/LaserScan`         | LiDAR scan data.                                |
| `/turret/angle`         | `sensor_msgs/msg/JointState`        | Rotational angle and velocity of the LiDAR (in rad).     |

The node uses `rclcpp::SensorDataQoS` for subscribers to prioritize low-latency communication for sensor data.
### Published Topics

The accumulator node publishes a point cloud with points of type XYZIRT, meaning

- XYZ: cartesian coordinates of the point
- I: intensity of the point
- R: ring 
- T: timestamp of the point

| Topic Name       | Message Type                         | Description                                      |
|------------------|--------------------------------------|--------------------------------------------------|
| `/turret/cloud`    | `sensor_msgs/msg/PointCloud2`       | Generated 3D point cloud.                       |

