import os
from ament_index_python.packages import get_package_share_directory

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare a launch argument for the config file name
    config_file_arg = launch.actions.DeclareLaunchArgument(
        'config_file',
        default_value='conf_custom_home.yaml',
        description='Name of the config YAML file inside the config folder'
    )

    # Get the package config directory
    config_dir = os.path.join(
        get_package_share_directory('rotating_lidar_accumulator'),
        'config'
    )

    # Full path to the selected config file
    param_config_path = launch.substitutions.PathJoinSubstitution([
        config_dir,
        LaunchConfiguration('config_file')
    ])

    return LaunchDescription([
        config_file_arg,
        launch_ros.actions.Node(
            package='rotating_lidar_accumulator',
            executable='accumulator',
            output='screen',
            parameters=[param_config_path]  # Load parameters from the selected YAML file
        ),
    ])
