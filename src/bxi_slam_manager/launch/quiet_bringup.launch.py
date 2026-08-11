from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    livox_share = get_package_share_directory("livox_ros_driver2")
    config_path = LaunchConfiguration("livox_config_path")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "livox_config_path",
                default_value=os.path.join(
                    livox_share, "config", "MID360s_config.json"
                ),
            ),
            Node(
                package="bxi_slam_manager",
                executable="bxi_slam_manager",
                name="bxi_slam_manager",
                output="screen",
                parameters=[{"livox_config_path": config_path}],
                respawn=True,
                respawn_delay=3.0,
            ),
        ]
    )
