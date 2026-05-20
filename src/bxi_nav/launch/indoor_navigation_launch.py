from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    rviz = LaunchConfiguration('rviz')
    nav_share_dir = get_package_share_directory('nav')
    default_map_file = os.path.join(
        nav_share_dir,
        'maps',
        'maps.yaml'
    )
    default_params_file = os.path.join(
        nav_share_dir,
        'config',
        'nav2_params.yaml'
    )
    nav2_launch = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )
    rviz_config = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=default_map_file,
            description='Full path to the 2D occupancy map yaml'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to the Nav2 params yaml'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically transition Nav2 lifecycle nodes'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Whether to start RViz'
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                params_file,
                {
                    'use_sim_time': use_sim_time,
                    'yaml_filename': map_file,
                }
            ]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': ['map_server'],
            }]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'params_file': params_file,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
            }.items()
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            remappings=[
                ('cloud_in', '/cloud_registered'),
                ('scan', '/scan'),
            ],
            parameters=[{
                'target_frame': 'base_link',

                # 只取一定高度范围内的点，模拟2D雷达
                'min_height': 0.00,
                'max_height': 0.50,

                # 360度扫描
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0087,

                'scan_time': 0.1,
                'range_min': 0.1,
                'range_max': 20.0,

                'use_inf': True,
                'inf_epsilon': 1.0,
            }]
        ),
        Node(
            condition=IfCondition(rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])
