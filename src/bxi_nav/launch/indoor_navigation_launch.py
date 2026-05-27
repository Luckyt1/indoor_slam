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
        # body_raw → base_link：把 Point-LIO 输出的物理帧修正为 REP-103 标准帧。
        # Point-LIO 现在发布 odom → body_raw（倒置朝向）。
        # 绕 x 轴转 180° 对应雷达倒扣安装（roll=π）。
        # 如果修正后方向仍不对，用 ros2 run tf2_ros tf2_echo odom body_raw 确认
        # 当前旋转，然后调整 roll/pitch/yaw 参数。
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='body_raw_to_base_link',
            output='screen',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '3.14159265',
                '--pitch', '0',
                '--yaw', '0',
                '--frame-id', 'body_raw',
                '--child-frame-id', 'base_link',
            ]
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
                'min_height': -1.0,
                'max_height': 0.40,

                # 360度扫描
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0087,

                'scan_time': 0.1,
                'range_min': 1.0,
                'range_max': 10.0,

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
