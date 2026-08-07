from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
GICP_SOURCE = (
    ROOT
    / "small_gicp_relocalization"
    / "src"
    / "small_gicp_relocalization.cpp"
)
GICP_HEADER = (
    ROOT
    / "small_gicp_relocalization"
    / "include"
    / "small_gicp_relocalization"
    / "small_gicp_relocalization.hpp"
)
GICP_LAUNCH = (
    ROOT
    / "small_gicp_relocalization"
    / "launch"
    / "small_gicp_relocalization_launch.py"
)


def test_production_gicp_avoids_unconsumed_point_cloud_streams() -> None:
    source = GICP_SOURCE.read_text(encoding="utf-8")
    header = GICP_HEADER.read_text(encoding="utf-8")
    launch = GICP_LAUNCH.read_text(encoding="utf-8")

    assert 'declare_parameter("publish_debug_clouds", false)' in source
    assert '"publish_debug_clouds": False' in launch
    assert "if (publish_debug_clouds_)" in source
    assert "global_map_timer_" not in source
    assert "global_map_timer_" not in header


def test_gicp_input_drops_stale_backlog() -> None:
    source = GICP_SOURCE.read_text(encoding="utf-8")

    assert "rclcpp::SensorDataQoS().keep_last(2)" in source


def test_gicp_fails_fast_for_missing_or_unusable_prior_map() -> None:
    source = GICP_SOURCE.read_text(encoding="utf-8")

    assert 'throw std::runtime_error("prior PCD path is empty")' in source
    assert "could not read prior PCD file" in source
    assert "prior PCD contains no points" in source
    assert "prior PCD has too few usable points after downsampling" in source
    assert "timed out resolving prior-map frame transform" in source


def test_gicp_rejects_invalid_initial_pose_quaternion() -> None:
    source = GICP_SOURCE.read_text(encoding="utf-8")

    assert "Rejecting initial pose with non-finite values" in source
    assert "Rejecting initial pose with invalid quaternion" in source
    assert "map_to_robot_base_rotation.normalize()" in source


def test_gicp_initial_pose_runs_yaw_hypothesis_search() -> None:
    source = GICP_SOURCE.read_text(encoding="utf-8")
    launch = GICP_LAUNCH.read_text(encoding="utf-8")

    # /initialpose 多假设朝向搜索: 参数已声明、launch 已配置、回调已接线,
    # 且无缓存扫描/搜索失败时按给定位姿种子 (不得静默丢弃 initialpose)。
    assert 'declare_parameter("initial_pose_yaw_hypotheses", 8)' in source
    assert '"initial_pose_yaw_hypotheses": 8' in launch
    assert "searchYawHypotheses(map_to_odom, map_to_robot_base, odom_to_robot_base)" in source
    assert "return map_to_odom_guess;" in source
    assert "best_inlier_ratio < min_inlier_ratio_" in source


def test_gicp_guards_degenerate_corridor_geometry() -> None:
    source = GICP_SOURCE.read_text(encoding="utf-8")
    launch = GICP_LAUNCH.read_text(encoding="utf-8")

    # 退化方向检测: Hessian 平移块特征分析 + 弱方向平移抑制。
    assert 'declare_parameter("degeneracy_min_eigen_ratio", 0.05)' in source
    assert '"degeneracy_min_eigen_ratio": 0.05' in launch
    assert "result.H.block<2, 2>(3, 3)" in source
    assert "weak-axis translation suppressed" in source


def test_gicp_tf_covers_current_time_for_nav2_controller() -> None:
    source = GICP_SOURCE.read_text(encoding="utf-8")
    header = GICP_HEADER.read_text(encoding="utf-8")
    launch = GICP_LAUNCH.read_text(encoding="utf-8")

    assert 'declare_parameter("transform_publish_tolerance", 0.5)' in source
    assert '"transform_publish_tolerance": 0.5' in launch
    assert "transform_publish_tolerance_" in header
    assert "transform_stamped.header.stamp = this->now() +" in source
    assert "last_scan_time_" not in source[source.index("void SmallGicpRelocalizationNode::publishTransform()"):
                                           source.index("void SmallGicpRelocalizationNode::initialPoseCallback")]
