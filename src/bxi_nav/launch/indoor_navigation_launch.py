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
            package='nav',
            executable='terrain_analysis',
            name='terrain_analysis',
            output='screen',
            parameters=[{
                # 输入的里程计话题，用于获取机器人在 odom/map 中的位置和姿态。
                'odometryTopic': '/aft_mapped_to_init',
                # 输入的点云话题，来自 Point-LIO 配准后的点云。
                'laserCloudTopic': '/cloud_registered',
                # 输出的地形/障碍点云话题，供调试或下游模块使用。
                'terrainMapTopic': '/terrain_map',
                # 对输入点云做体素降采样的尺寸，单位米；越小保留点越多、计算量越大。
                'scanVoxelSize': 0.05,
                # 地形体素的时间衰减，超过该时间未更新的旧点会逐渐失效，单位秒。
                'decayTime': 1.0,
                # 初始化/清图后，机器人移动超过该距离前不启用 no-data 衰减，单位米。
                'noDecayDis': 0.0,
                # 手动清除地形点云时的清除半径，单位米。
                'clearingDis': 8.0,
                # 是否对体素内高度排序；开启后可用分位数估计地面高度。
                'useSorting': False,
                # 使用排序时选取的高度分位数，0.25 表示偏低的地面估计。
                'quantileZ': 0.25,
                # 是否考虑台阶/下落区域，避免把突然降低的区域误判为可通行地面。
                'considerDrop': True,
                # 是否限制地面高度在相邻更新中的上升幅度。
                'limitGroundLift': False,
                # 开启 limitGroundLift 时允许的最大地面抬升高度，单位米。
                'maxGroundLift': 0.15,
                # 是否清除动态障碍；开启后会按高度/视场/点数规则过滤移动物体。
                'clearDyObs': True,
                # 动态障碍判定的最小距离，单位米。
                'minDyObsDis': 0.0,
                # 动态障碍判定的最小角度阈值，单位度。
                'minDyObsAngle': 0.0,
                # 动态障碍相对地面的最小高度，单位米。
                'minDyObsRelZ': 0.0,
                # 动态障碍绝对相对高度阈值，单位米。
                'absDyObsRelZThre': -0.7,
                # 动态障碍判定的垂直视场下限，单位度。
                'minDyObsVFOV': -16.0,
                # 动态障碍判定的垂直视场上限，单位度。
                'maxDyObsVFOV': 16.0,
                # 判定动态障碍所需的最少点数。
                'minDyObsPointNum': 5,
                # 是否把无点云数据的区域视为障碍。
                'noDataObstacle': False,
                # no-data 障碍判定时跳过的空块数量，用于降低误报。
                'noDataBlockSkipNum': 0,
                # 一个地形块被认为有效所需的最少点数。
                'minBlockPointNum': 5,
                # 机器人高度，用于裁剪/判断与机器人相关的点云，单位米。
                'vehicleHeight': 1.0,
                # 单个体素累计到该点数后才触发地形更新。
                'voxelPointUpdateThre': 50,
                # 单个体素距离上次更新时间超过该阈值后允许再次更新，单位秒。
                'voxelTimeUpdateThre': 1.0,
                # 接收点云相对机器人高度的下限，单位米。
                'minRelZ': -1.0,
                # 接收点云相对机器人高度的上限，单位米。
                'maxRelZ': 0.4,
                # 随水平距离放宽高度裁剪范围的比例。
                'disRatioZ': 0.2,
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
