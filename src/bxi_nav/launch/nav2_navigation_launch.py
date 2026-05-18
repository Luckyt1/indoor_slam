import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_dir = get_package_share_directory("nav")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    namespace = LaunchConfiguration("namespace")
    map_yaml_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    use_rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace for Nav2 nodes.",
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
        description="Automatically configure and activate lifecycle nodes.",
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
        description="Launch RViz with the default Nav2 view.",
    )
    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(nav2_bringup_dir, "rviz", "nav2_default_view.rviz"),
        description="RViz config file.",
    )

    stdout_linebuf_envvar = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        namespace=namespace,
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            params_file,
            {"use_sim_time": use_sim_time},
            {"yaml_filename": map_yaml_file},
        ],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings,
    )

    lifecycle_manager_map = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map_server",
        namespace=namespace,
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": ["map_server"]},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "params_file": params_file,
            "use_composition": "False",
            "use_respawn": use_respawn,
            "log_level": log_level,
        }.items(),
    )

    rviz = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace=namespace,
        output="screen",
        arguments=["-d", rviz_config],
        remappings=remappings,
    )

    return LaunchDescription(
        [
            stdout_linebuf_envvar,
            declare_namespace,
            declare_map,
            declare_params_file,
            declare_use_sim_time,
            declare_autostart,
            declare_use_respawn,
            declare_log_level,
            declare_rviz,
            declare_rviz_config,
            map_server,
            lifecycle_manager_map,
            navigation,
            rviz,
        ]
    )
