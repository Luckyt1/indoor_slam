"""Integration contracts for the indoor_slam NavFn + PID planner port."""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml


NAV_PACKAGE = Path(__file__).resolve().parents[1]
REPOSITORY = NAV_PACKAGE.parents[1]
CONFIG_PATH = NAV_PACKAGE / "config" / "nav2_params.yaml"
LAUNCH_PATH = NAV_PACKAGE / "launch" / "indoor_navigation_launch.py"
CMAKE_PATH = NAV_PACKAGE / "CMakeLists.txt"
PACKAGE_XML_PATH = NAV_PACKAGE / "package.xml"
PID_PACKAGE = REPOSITORY / "src" / "pid_path_follower"
MAPPING_CONTROL = (
    REPOSITORY / "src" / "Point-LIO" / "scripts" / "mapping_control_node.py"
)
PROCESS_BACKEND = (
    REPOSITORY
    / "src"
    / "bxi_slam_manager"
    / "bxi_slam_manager"
    / "process_backend.py"
)
APP_GATEWAY = NAV_PACKAGE / "src" / "main.cpp"

CONFIG = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
CONTROLLER = CONFIG["controller_server"]["ros__parameters"]
PLANNER = CONFIG["planner_server"]["ros__parameters"]
VELOCITY_SMOOTHER = CONFIG["velocity_smoother"]["ros__parameters"]


def test_indoor_slam_planner_and_controller_are_selected() -> None:
    assert PLANNER["planner_plugins"] == ["GridBased"]
    assert PLANNER["GridBased"]["plugin"] == "nav2_navfn_planner/NavfnPlanner"
    assert PLANNER["GridBased"]["use_astar"] is True
    assert PLANNER["GridBased"]["allow_unknown"] is False

    assert CONTROLLER["controller_plugins"] == ["FollowPath"]
    follow_path = CONTROLLER["FollowPath"]
    assert follow_path["plugin"] == "pid_path_follower::PidPathFollower"
    assert follow_path["collision_enabled"] is True
    assert follow_path["collision_cloud_topic"] == "/terrain_map"
    assert follow_path["use_path_library"] is True


def test_port_uses_bxi_corrected_odometry_and_keeps_indoor_limits() -> None:
    assert CONTROLLER["odom_topic"] == "/nav/odom"
    assert CONFIG["bt_navigator"]["ros__parameters"]["odom_topic"] == "/nav/odom"
    assert VELOCITY_SMOOTHER["odom_topic"] == "/nav/odom"
    assert VELOCITY_SMOOTHER["max_velocity"] == pytest.approx([0.35, 0.0, 1.0])
    assert VELOCITY_SMOOTHER["min_velocity"] == pytest.approx([-0.10, 0.0, -1.0])


def test_launch_preserves_app_runtime_and_safety_boundaries() -> None:
    launch = LAUNCH_PATH.read_text(encoding="utf-8")
    assert "navigation_launch.py" in launch
    assert "nav2_params.yaml" in launch
    assert "executable='terrain_analysis'" in launch
    assert "'terrainMapTopic': '/terrain_map'" in launch
    assert "executable='nav_odom'" in launch
    assert "'output_topic': '/nav/odom'" in launch
    assert "name='app_nav_gateway'" in launch
    assert "executable='collision_monitor'" in launch
    assert "navigation_direct_cmd_vel_launch.py" not in launch

    backend = PROCESS_BACKEND.read_text(encoding="utf-8")
    assert '"nav", "indoor_navigation_launch.py"' in backend
    assert '"autostart:=true"' in backend
    assert '"rviz:=false"' in backend


def test_terrain_analysis_and_pid_plugin_are_built() -> None:
    cmake = CMAKE_PATH.read_text(encoding="utf-8")
    assert "add_executable(terrain_analysis src/terrain_analysis.cpp)" in cmake
    assert "ament_target_dependencies(terrain_analysis" in cmake
    assert "find_package(pcl_ros REQUIRED)" in cmake
    assert (NAV_PACKAGE / "src" / "terrain_analysis.cpp").is_file()
    assert not (PID_PACKAGE / "COLCON_IGNORE").exists()
    assert (PID_PACKAGE / "pid_path_follower_plugin.xml").is_file()

    package = ET.parse(PACKAGE_XML_PATH).getroot()
    runtime_dependencies = {node.text for node in package.findall("exec_depend")}
    assert {
        "nav2_navfn_planner",
        "nav2_smoother",
        "nav2_velocity_smoother",
        "nav2_waypoint_follower",
        "pid_path_follower",
        "nav2_collision_monitor",
    }.issubset(runtime_dependencies)
    assert {"nav2_smac_planner", "nav2_mppi_controller"}.isdisjoint(
        runtime_dependencies
    )


def test_app_navigation_contract_is_unchanged() -> None:
    gateway = APP_GATEWAY.read_text(encoding="utf-8")
    for endpoint in (
        '"/nav/init"',
        '"/nav/goto"',
        '"/nav/follow_waypoints"',
        '"/nav/pause"',
        '"/nav/pose"',
        '"/nav/status"',
    ):
        assert endpoint in gateway
    assert '"/navigate_to_pose"' in gateway
    assert '"/navigate_through_poses"' in gateway


def test_app_terrain_clear_also_clears_pid_collision_cloud() -> None:
    source = MAPPING_CONTROL.read_text(encoding="utf-8")
    assert 'Float32, "/map_clearing"' in source
    assert "self.terrain_clear_pub.publish(clear_msg)" in source

