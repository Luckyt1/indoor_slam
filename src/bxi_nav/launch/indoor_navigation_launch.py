from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    rviz = LaunchConfiguration('rviz')
    params_file = LaunchConfiguration('params_file')
    nav_share_dir = get_package_share_directory('nav')
    default_params_file = os.path.join(
        nav_share_dir, 'config', 'nav2_params.yaml')
    scan_params_file = os.path.join(nav_share_dir, 'config', 'scan_params.yaml')
    collision_monitor_params_file = os.path.join(
        nav_share_dir, 'config', 'collision_monitor_params.yaml')
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
            default_value='',
            description='Required full path to the selected 2D occupancy map yaml'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='false',
            description='Automatically transition Nav2 lifecycle nodes'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Whether to start RViz'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Nav2 parameter file (defaults to the ported indoor_slam tuning)'
        ),
        GroupAction(actions=[
            SetRemap(src='/plan', dst='/debug/plan'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={
                    'params_file': params_file,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'use_composition': 'False',
                }.items()),
        ]),
        # Point-LIO publishes its estimator-native twist in the upside-down
        # body_raw frame. Convert that odometry to canonical base_link axes;
        # no pose differentiation or temporal filtering is applied here.
        Node(
            package='nav',
            executable='nav_odom',
            name='nav_odom',
            output='screen',
            parameters=[{
                'input_topic': '/aft_mapped_to_init',
                'output_topic': '/nav/odom',
                'robot_base_frame': 'base_link',
            }],
        ),
        # indoor_slam's local planner consumes this rolling terrain cloud for
        # footprint/path collision checks.  Keep its tuned parameters while
        # using the corrected /nav/odom integration topic from this workspace.
        Node(
            package='nav',
            executable='terrain_analysis',
            name='terrain_analysis',
            output='screen',
            parameters=[{
                'odometryTopic': '/nav/odom',
                'laserCloudTopic': '/cloud_registered',
                'terrainMapTopic': '/terrain_map',
                'scanVoxelSize': 0.05,
                'decayTime': 1.0,
                'noDecayDis': 0.0,
                'clearingDis': 8.0,
                'useSorting': False,
                'quantileZ': 0.25,
                'considerDrop': True,
                'limitGroundLift': False,
                'maxGroundLift': 0.15,
                'clearDyObs': True,
                'minDyObsDis': 0.0,
                'minDyObsAngle': 0.0,
                'minDyObsRelZ': 0.0,
                'absDyObsRelZThre': -0.7,
                'minDyObsVFOV': -16.0,
                'maxDyObsVFOV': 16.0,
                'minDyObsPointNum': 5,
                'noDataObstacle': False,
                'noDataBlockSkipNum': 0,
                'minBlockPointNum': 5,
                'vehicleHeight': 1.0,
                'voxelPointUpdateThre': 50,
                'voxelTimeUpdateThre': 1.0,
                'minRelZ': -1.0,
                'maxRelZ': 0.4,
                'disRatioZ': 0.2,
            }]
        ),
        # Keep the near-field safety guard after indoor_slam's velocity
        # smoother. It remains the only publisher of /cmd_vel_safe, which the
        # robot gateway accepts, and fails closed when scan data becomes stale.
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            emulate_tty=True,
            parameters=[
                collision_monitor_params_file,
                {'use_sim_time': use_sim_time},
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_collision_monitor',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': ['collision_monitor'],
            }],
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
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
        Node(
            package='nav',
            executable='indoor_nav_goal',
            name='app_nav_gateway',
            output='screen',
            parameters=[{
                'frame_id': 'map',
                'odom_topic': '/nav/odom',
                'robot_base_frame': 'base_link',
            }]
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
            # 该文件同时供 SLAM manager 的独立 scan 节点使用，避免两个入口
            # 的量程悄悄漂移。
            parameters=[scan_params_file]
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
