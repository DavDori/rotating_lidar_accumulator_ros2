import os
from ament_index_python.packages import get_package_share_directory

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Declare a launch argument for the config file name
    config_file_arg = launch.actions.DeclareLaunchArgument(
        'config',
        default_value='conf_custom_home.yaml',
        description='Name of the config YAML file inside the config folder'
    )

    use_sim_time_arg = launch.actions.DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use /clock (e.g. from rosbag2 play --clock) instead of system time'
    )

    # Get the package config directory
    config_dir = os.path.join(
        get_package_share_directory('rotating_lidar_accumulator'),
        'config'
    )

    # Full path to the selected config file
    param_config_path = launch.substitutions.PathJoinSubstitution([
        config_dir,
        LaunchConfiguration('config')
    ])

    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        launch_ros.actions.Node(
            package='rotating_lidar_accumulator',
            executable='accumulator',
            output='screen',
            parameters=[param_config_path, {'use_sim_time': use_sim_time}]  # Load parameters from the selected YAML file
        ),
    ])
