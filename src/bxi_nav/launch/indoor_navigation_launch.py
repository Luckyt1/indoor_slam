import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_dir = get_package_share_directory("nav")
    point_lio_dir = get_package_share_directory("point_lio")
    relocalization_dir = get_package_share_directory("small_gicp_relocalization")

    use_point_lio = LaunchConfiguration("use_point_lio")
    use_relocalization = LaunchConfiguration("use_relocalization")
    use_nav2 = LaunchConfiguration("use_nav2")
    publish_base_link_tf = LaunchConfiguration("publish_base_link_tf")
    point_lio_cfg_dir = LaunchConfiguration("point_lio_cfg_dir")
    map_yaml_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    rviz = LaunchConfiguration("rviz")
    lio_frame = LaunchConfiguration("lio_frame")
    base_frame = LaunchConfiguration("base_frame")
    lio_to_base_x = LaunchConfiguration("lio_to_base_x")
    lio_to_base_y = LaunchConfiguration("lio_to_base_y")
    lio_to_base_z = LaunchConfiguration("lio_to_base_z")
    lio_to_base_roll = LaunchConfiguration("lio_to_base_roll")
    lio_to_base_pitch = LaunchConfiguration("lio_to_base_pitch")
    lio_to_base_yaw = LaunchConfiguration("lio_to_base_yaw")

    declare_use_point_lio = DeclareLaunchArgument(
        "use_point_lio",
        default_value="true",
        description="Start Point-LIO odometry.",
    )
    declare_use_relocalization = DeclareLaunchArgument(
        "use_relocalization",
        default_value="true",
        description="Start small_gicp_relocalization for map -> odom.",
    )
    declare_use_nav2 = DeclareLaunchArgument(
        "use_nav2",
        default_value="true",
        description="Start Nav2 navigation.",
    )
    declare_publish_base_link_tf = DeclareLaunchArgument(
        "publish_base_link_tf",
        default_value="true",
        description="Publish the static transform from Point-LIO body frame to base_link.",
    )
    declare_point_lio_cfg = DeclareLaunchArgument(
        "point_lio_cfg_dir",
        default_value=os.path.join(nav_dir, "config", "point_lio_mid360_nav.yaml"),
        description="Point-LIO config file.",
    )
    declare_map = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(nav_dir, "maps", "indoor_map_no_trail.yaml"),
        description="Full path to the 2D occupancy map yaml.",
    )
    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(nav_dir, "config", "nav2_params.yaml"),
        description="Full path to the Nav2 params yaml.",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use /clock when true.",
    )
    declare_autostart = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically activate Nav2 lifecycle nodes.",
    )
    declare_use_respawn = DeclareLaunchArgument(
        "use_respawn",
        default_value="false",
        description="Respawn Nav2 nodes if they crash.",
    )
    declare_log_level = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="ROS log level.",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Launch RViz with the Nav2 view.",
    )
    declare_lio_frame = DeclareLaunchArgument(
        "lio_frame",
        default_value="lio_base_link",
        description="Point-LIO body frame published under odom.",
    )
    declare_base_frame = DeclareLaunchArgument(
        "base_frame",
        default_value="base_link",
        description="Robot base frame used by Nav2.",
    )
    declare_lio_to_base_x = DeclareLaunchArgument(
        "lio_to_base_x",
        default_value="0.0",
        description="Static transform x from lio_frame to base_frame.",
    )
    declare_lio_to_base_y = DeclareLaunchArgument(
        "lio_to_base_y",
        default_value="0.0",
        description="Static transform y from lio_frame to base_frame.",
    )
    declare_lio_to_base_z = DeclareLaunchArgument(
        "lio_to_base_z",
        default_value="0.0",
        description="Static transform z from lio_frame to base_frame.",
    )
    declare_lio_to_base_roll = DeclareLaunchArgument(
        "lio_to_base_roll",
        default_value="0.0",
        description="Static transform roll from lio_frame to base_frame.",
    )
    declare_lio_to_base_pitch = DeclareLaunchArgument(
        "lio_to_base_pitch",
        default_value="0.0",
        description="Static transform pitch from lio_frame to base_frame.",
    )
    declare_lio_to_base_yaw = DeclareLaunchArgument(
        "lio_to_base_yaw",
        default_value="3.141592653589793",
        description="Static transform yaw from lio_frame to base_frame.",
    )

    point_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(point_lio_dir, "launch", "point_lio.launch.py")
        ),
        condition=IfCondition(use_point_lio),
        launch_arguments={
            "rviz": "false",
            "point_lio_cfg_dir": point_lio_cfg_dir,
        }.items(),
    )

    relocalization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                relocalization_dir,
                "launch",
                "small_gicp_relocalization_launch.py",
            )
        ),
        condition=IfCondition(use_relocalization),
    )

    base_link_tf = Node(
        condition=IfCondition(publish_base_link_tf),
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lio_to_base_link_tf",
        arguments=[
            "--x",
            lio_to_base_x,
            "--y",
            lio_to_base_y,
            "--z",
            lio_to_base_z,
            "--roll",
            lio_to_base_roll,
            "--pitch",
            lio_to_base_pitch,
            "--yaw",
            lio_to_base_yaw,
            "--frame-id",
            lio_frame,
            "--child-frame-id",
            base_frame,
        ],
        output="screen",
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_dir, "launch", "nav2_navigation_launch.py")),
        condition=IfCondition(use_nav2),
        launch_arguments={
            "map": map_yaml_file,
            "params_file": params_file,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "use_respawn": use_respawn,
            "log_level": log_level,
            "rviz": rviz,
        }.items(),
    )

    return LaunchDescription(
        [
            declare_use_point_lio,
            declare_use_relocalization,
            declare_use_nav2,
            declare_publish_base_link_tf,
            declare_point_lio_cfg,
            declare_map,
            declare_params_file,
            declare_use_sim_time,
            declare_autostart,
            declare_use_respawn,
            declare_log_level,
            declare_rviz,
            declare_lio_frame,
            declare_base_frame,
            declare_lio_to_base_x,
            declare_lio_to_base_y,
            declare_lio_to_base_z,
            declare_lio_to_base_roll,
            declare_lio_to_base_pitch,
            declare_lio_to_base_yaw,
            point_lio,
            relocalization,
            base_link_tf,
            nav2,
        ]
    )
