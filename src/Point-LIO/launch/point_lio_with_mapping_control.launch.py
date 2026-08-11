import os

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static"), ("path", "/debug/path")]
    indoor_root = os.environ.get(
        "BXI_INDOOR_SLAM_ROOT", "/opt/bxi/bxi_rc_slam"
    )
    point_lio_source_dir = os.environ.get(
        "BXI_POINT_LIO_SOURCE_DIR",
        os.path.join(indoor_root, "src", "Point-LIO"),
    )

    namespace = LaunchConfiguration("namespace")
    use_rviz = LaunchConfiguration("rviz")
    point_lio_cfg_dir = LaunchConfiguration("point_lio_cfg_dir")
    mapping_control = LaunchConfiguration("mapping_control")
    mapping_cloud_topic = LaunchConfiguration("mapping_cloud_topic")
    mapping_odometry_topic = LaunchConfiguration("mapping_odometry_topic")
    mapping_robot_frame = LaunchConfiguration("mapping_robot_frame")
    mapping_scan = LaunchConfiguration("mapping_scan")
    mapping_scan_topic = LaunchConfiguration("mapping_scan_topic")
    mapping_scan_target_frame = LaunchConfiguration("mapping_scan_target_frame")
    mapping_output_dir = LaunchConfiguration("mapping_output_dir")
    mapping_resolution = LaunchConfiguration("mapping_resolution")
    mapping_size_x = LaunchConfiguration("mapping_size_x")
    mapping_size_y = LaunchConfiguration("mapping_size_y")
    mapping_origin_x = LaunchConfiguration("mapping_origin_x")
    mapping_origin_y = LaunchConfiguration("mapping_origin_y")
    mapping_dynamic_clear_min_observations = LaunchConfiguration(
        "mapping_dynamic_clear_min_observations"
    )
    mapping_grid_expansion_padding = LaunchConfiguration(
        "mapping_grid_expansion_padding"
    )
    mapping_max_grid_expansion_per_frame = LaunchConfiguration(
        "mapping_max_grid_expansion_per_frame"
    )
    mapping_max_live_grid_cells = LaunchConfiguration(
        "mapping_max_live_grid_cells"
    )
    mapping_status_period = LaunchConfiguration("mapping_status_period")
    mapping_map_period = LaunchConfiguration("mapping_map_period")
    pcd2pgm_executable = LaunchConfiguration("pcd2pgm_executable")
    pcd2pgm_config = LaunchConfiguration("pcd2pgm_config")
    mapping_pcd_input_path = LaunchConfiguration("mapping_pcd_input_path")
    map_store_root = LaunchConfiguration("map_store_root")
    pcd_merge_executable = LaunchConfiguration("pcd_merge_executable")

    point_lio_dir = get_package_share_directory("point_lio")

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for Point-LIO and mapping control nodes",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="True", description="Flag to launch RViz."
    )
    declare_point_lio_cfg_dir = DeclareLaunchArgument(
        "point_lio_cfg_dir",
        default_value=PathJoinSubstitution([point_lio_dir, "config", "mid360.yaml"]),
        description="Path to the Point-LIO config file",
    )
    declare_mapping_control = DeclareLaunchArgument(
        "mapping_control",
        default_value="True",
        description="Whether to start the mapping control node",
    )
    declare_mapping_cloud_topic = DeclareLaunchArgument(
        "mapping_cloud_topic",
        default_value="/cloud_registered",
        description="Point cloud topic accumulated by the mapping control node",
    )
    declare_mapping_odometry_topic = DeclareLaunchArgument(
        "mapping_odometry_topic",
        default_value="/aft_mapped_to_init",
        description="Point-LIO odometry used for the live App robot pose",
    )
    declare_mapping_robot_frame = DeclareLaunchArgument(
        "mapping_robot_frame",
        default_value="base_link",
        description="REP-103 robot frame shared by mapping and navigation",
    )
    declare_mapping_scan = DeclareLaunchArgument(
        "mapping_scan",
        default_value="True",
        description="Publish a lightweight 2D laser scan for the live App view",
    )
    declare_mapping_scan_topic = DeclareLaunchArgument(
        "mapping_scan_topic",
        default_value="/scan",
        description="LaserScan topic published during mapping",
    )
    declare_mapping_scan_target_frame = DeclareLaunchArgument(
        "mapping_scan_target_frame",
        default_value="base_link",
        description="Frame used by the mapping-time 2D laser scan",
    )
    declare_mapping_output_dir = DeclareLaunchArgument(
        "mapping_output_dir",
        default_value=os.path.join(indoor_root, "src", "bxi_nav", "maps"),
        description="Directory where saved .pgm and .yaml maps are written",
    )
    declare_mapping_resolution = DeclareLaunchArgument(
        "mapping_resolution",
        default_value="0.05",
        description="Saved occupancy map resolution in meters per cell",
    )
    declare_mapping_size_x = DeclareLaunchArgument(
        "mapping_size_x",
        default_value="60.0",
        description="Saved occupancy map width in meters",
    )
    declare_mapping_size_y = DeclareLaunchArgument(
        "mapping_size_y",
        default_value="60.0",
        description="Saved occupancy map height in meters",
    )
    declare_mapping_origin_x = DeclareLaunchArgument(
        "mapping_origin_x",
        default_value="-30.0",
        description="Saved occupancy map origin x in meters",
    )
    declare_mapping_origin_y = DeclareLaunchArgument(
        "mapping_origin_y",
        default_value="-30.0",
        description="Saved occupancy map origin y in meters",
    )
    declare_mapping_dynamic_clear_min_observations = DeclareLaunchArgument(
        "mapping_dynamic_clear_min_observations",
        default_value="10",
        description="Consecutive free cloud frames required to clear a stale obstacle",
    )
    declare_mapping_grid_expansion_padding = DeclareLaunchArgument(
        "mapping_grid_expansion_padding",
        default_value="5.0",
        description="Extra meters reserved whenever an extended map grows",
    )
    declare_mapping_max_grid_expansion_per_frame = DeclareLaunchArgument(
        "mapping_max_grid_expansion_per_frame",
        default_value="40.0",
        description="Maximum distance beyond the current grid used for one-frame growth",
    )
    declare_mapping_max_live_grid_cells = DeclareLaunchArgument(
        "mapping_max_live_grid_cells",
        default_value="4000000",
        description="Hard capacity limit for the live mapping grid",
    )
    declare_mapping_status_period = DeclareLaunchArgument(
        "mapping_status_period",
        default_value="1.0",
        description="Seconds between /mapping/status publications",
    )
    declare_mapping_map_period = DeclareLaunchArgument(
        "mapping_map_period",
        default_value="0.5",
        description="Seconds between live /map preview publications",
    )
    declare_pcd2pgm_executable = DeclareLaunchArgument(
        "pcd2pgm_executable",
        default_value=os.path.join(indoor_root, "pcd2pgm_headless"),
        description="Executable used to convert Point-LIO PCD into Nav2 map files",
    )
    declare_pcd2pgm_config = DeclareLaunchArgument(
        "pcd2pgm_config",
        default_value=os.path.join(indoor_root, "scans_nav2_map.cfg"),
        description="pcd2pgm_headless config file",
    )
    declare_mapping_pcd_input_path = DeclareLaunchArgument(
        "mapping_pcd_input_path",
        default_value=os.path.join(point_lio_source_dir, "PCD", "scans.pcd"),
        description="Point-LIO PCD file converted when /mapping/save is called",
    )
    declare_map_store_root = DeclareLaunchArgument(
        "map_store_root",
        default_value="/var/lib/bxi/maps",
        description="Robot-side versioned map bundle root",
    )
    declare_pcd_merge_executable = DeclareLaunchArgument(
        "pcd_merge_executable",
        default_value=os.path.join(
            get_package_prefix("point_lio"), "lib", "point_lio", "merge_pcd_maps"
        ),
        description="Executable that merges a parent PCD with an aligned increment",
    )

    start_point_lio_node = Node(
        package="point_lio",
        executable="pointlio_mapping",
        namespace=namespace,
        parameters=[point_lio_cfg_dir],
        remappings=remappings,
        output="screen",
    )

    # Point-LIO publishes odom -> body_raw for the upside-down LiDAR/IMU.
    # Publish the mounting correction with Point-LIO itself so mapping,
    # extension and navigation all consume one canonical base_link frame.
    body_raw_to_base_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="body_raw_to_base_link",
        output="screen",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "3.14159265",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            "body_raw",
            "--child-frame-id",
            "base_link",
        ],
    )

    start_mapping_control_node = Node(
        condition=IfCondition(mapping_control),
        package="point_lio",
        executable="mapping_control_node.py",
        namespace=namespace,
        name="mapping_control_node",
        output="screen",
        arguments=[
            "--cloud-topic",
            mapping_cloud_topic,
            "--odometry-topic",
            mapping_odometry_topic,
            "--robot-frame",
            mapping_robot_frame,
            "--output-dir",
            mapping_output_dir,
            "--resolution",
            mapping_resolution,
            "--size-x",
            mapping_size_x,
            "--size-y",
            mapping_size_y,
            "--origin-x",
            mapping_origin_x,
            "--origin-y",
            mapping_origin_y,
            "--dynamic-clear-min-observations",
            mapping_dynamic_clear_min_observations,
            "--grid-expansion-padding",
            mapping_grid_expansion_padding,
            "--max-grid-expansion-per-frame",
            mapping_max_grid_expansion_per_frame,
            "--max-live-grid-cells",
            mapping_max_live_grid_cells,
            "--status-period",
            mapping_status_period,
            "--map-period",
            mapping_map_period,
            "--pcd2pgm-executable",
            pcd2pgm_executable,
            "--pcd2pgm-config",
            pcd2pgm_config,
            "--pcd-input-path",
            mapping_pcd_input_path,
            "--map-store-root",
            map_store_root,
            "--pcd-merge-executable",
            pcd_merge_executable,
        ],
    )

    # The live occupancy grid is deliberately capped at 2 Hz because it is a
    # full 600x600 frame.  Fluid motion in the App comes from this much lighter
    # scan stream, so mapping must not depend on a stale navigation process to
    # provide /scan.
    start_mapping_scan_node = Node(
        condition=IfCondition(mapping_scan),
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="mapping_pointcloud_to_laserscan",
        output="screen",
        remappings=[
            ("cloud_in", mapping_cloud_topic),
            ("scan", mapping_scan_topic),
        ],
        parameters=[{
            "target_frame": mapping_scan_target_frame,
            "transform_tolerance": 0.05,
            "min_height": -1.0,
            "max_height": 0.40,
            "angle_min": -3.141592653589793,
            "angle_max": 3.141592653589793,
            "angle_increment": 0.008726646259971648,
            "scan_time": 0.1,
            "range_min": 0.2,
            "range_max": 10.0,
            "use_inf": True,
            "inf_epsilon": 1.0,
        }],
    )

    start_rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        namespace=namespace,
        name="rviz",
        remappings=remappings,
        arguments=[
            "-d",
            PathJoinSubstitution([point_lio_dir, "rviz_cfg", "loam_livox"]),
            ".rviz",
        ],
    )

    return LaunchDescription(
        [
            declare_namespace,
            declare_rviz,
            declare_point_lio_cfg_dir,
            declare_mapping_control,
            declare_mapping_cloud_topic,
            declare_mapping_odometry_topic,
            declare_mapping_robot_frame,
            declare_mapping_scan,
            declare_mapping_scan_topic,
            declare_mapping_scan_target_frame,
            declare_mapping_output_dir,
            declare_mapping_resolution,
            declare_mapping_size_x,
            declare_mapping_size_y,
            declare_mapping_origin_x,
            declare_mapping_origin_y,
            declare_mapping_dynamic_clear_min_observations,
            declare_mapping_grid_expansion_padding,
            declare_mapping_max_grid_expansion_per_frame,
            declare_mapping_max_live_grid_cells,
            declare_mapping_status_period,
            declare_mapping_map_period,
            declare_pcd2pgm_executable,
            declare_pcd2pgm_config,
            declare_mapping_pcd_input_path,
            declare_map_store_root,
            declare_pcd_merge_executable,
            start_point_lio_node,
            body_raw_to_base_link,
            start_mapping_scan_node,
            start_mapping_control_node,
            start_rviz_node,
        ]
    )
