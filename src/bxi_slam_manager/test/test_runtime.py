from pathlib import Path

from bxi_slam_manager.runtime import (
    MapArtifacts,
    RuntimeMode,
    RuntimeRequest,
    RuntimeSupervisor,
)


class FakeProcesses:
    def __init__(self) -> None:
        self.events: list[tuple[str, object]] = []
        self.failures: dict[str, int] = {}
        self.stop_error: Exception | None = None
        self.poll_error: Exception | None = None
        self.driver_alive = True
        self.hot_switch_result = False

    def stop_pipeline(self, *, keep_driver: bool = False) -> None:
        # 事件里记录 keep_driver: True=模式切换的待机停组, False=全停。
        self.events.append(("stop", keep_driver))
        if not keep_driver:
            self.driver_alive = False
        if self.stop_error is not None:
            raise self.stop_error

    def stop_driver(self) -> None:
        self.events.append(("stop_driver", None))
        self.driver_alive = False

    def driver_running(self) -> bool:
        return self.driver_alive

    def try_switch_navigation_map(
        self, artifacts: MapArtifacts, *, reload_gicp: bool = True
    ) -> bool:
        self.events.append(("try_switch_navigation_map", artifacts, reload_gicp))
        return self.hot_switch_result

    def start_new_mapping(self) -> None:
        self.events.append(("start_new_mapping", None))

    def start_navigation(self, artifacts: MapArtifacts) -> None:
        self.events.append(("start_navigation", artifacts))

    def start_extend_mapping(self, artifacts: MapArtifacts) -> None:
        self.events.append(("start_extend_mapping", artifacts))

    def prepare_extend_mapping(self, artifacts: MapArtifacts) -> None:
        self.events.append(("prepare_extend_mapping", artifacts))

    def poll_failed_processes(self) -> dict[str, int]:
        if self.poll_error is not None:
            raise self.poll_error
        failures, self.failures = self.failures, {}
        return failures


class ManualClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now


def artifacts() -> MapArtifacts:
    return MapArtifacts(
        map_id="map-a",
        pcd_path=Path("/maps/map-a/map.pcd"),
        yaml_path=Path("/maps/map-a/map.yaml"),
    )


def test_boot_is_quiet_and_starts_no_algorithm_pipeline() -> None:
    processes = FakeProcesses()

    runtime = RuntimeSupervisor(processes)

    assert runtime.snapshot.current_mode is RuntimeMode.idle
    assert runtime.snapshot.desired_mode is RuntimeMode.idle
    assert processes.events == []


def test_mode_change_stops_previous_pipeline_before_starting_next() -> None:
    processes = FakeProcesses()
    runtime = RuntimeSupervisor(processes)

    result = runtime.apply(
        RuntimeRequest(
            request_id="req-1",
            mode=RuntimeMode.navigation,
            artifacts=artifacts(),
        )
    )

    assert result.accepted
    assert processes.events == [
        ("stop", True),
        ("start_navigation", artifacts()),
    ]
    assert runtime.snapshot.current_mode is RuntimeMode.localizing
    assert runtime.snapshot.active_map_id == "map-a"
    assert not runtime.snapshot.localized


def test_duplicate_request_id_is_idempotent() -> None:
    processes = FakeProcesses()
    runtime = RuntimeSupervisor(processes)
    request = RuntimeRequest(
        request_id="same-request",
        mode=RuntimeMode.new_mapping,
    )

    first = runtime.apply(request)
    second = runtime.apply(request)

    assert first.transition_id == second.transition_id
    assert processes.events == [("stop", True), ("start_new_mapping", None)]


def test_retried_same_mode_request_does_not_restart_pipeline() -> None:
    # 网关超时重试会带全新 request_id 重发同一目标模式 (典型: 扩图→新建图
    # 的慢转换期间 App 反复 POST mapping/start)。语义已满足的请求必须直接
    # 确认, 否则排队的重复请求会把刚起好的管线反复推倒。
    processes = FakeProcesses()
    runtime = RuntimeSupervisor(processes)

    first = runtime.apply(
        RuntimeRequest(request_id="retry-1", mode=RuntimeMode.new_mapping)
    )
    events_after_first = list(processes.events)
    second = runtime.apply(
        RuntimeRequest(request_id="retry-2", mode=RuntimeMode.new_mapping)
    )

    assert second.accepted
    assert second.transition_id == first.transition_id
    assert processes.events == events_after_first


def test_retried_extend_request_while_localizing_does_not_restart() -> None:
    # extend/navigation 的收敛中间态是 localizing; 同地图的重复请求同样短路。
    processes = FakeProcesses()
    runtime = RuntimeSupervisor(processes)

    first = runtime.apply(
        RuntimeRequest(
            request_id="extend-1",
            mode=RuntimeMode.extend_mapping,
            artifacts=artifacts(),
        )
    )
    events_after_first = list(processes.events)
    second = runtime.apply(
        RuntimeRequest(
            request_id="extend-2",
            mode=RuntimeMode.extend_mapping,
            artifacts=artifacts(),
        )
    )

    assert second.accepted
    assert second.transition_id == first.transition_id
    assert processes.events == events_after_first

    # 换一张地图不是重复请求, 必须真正走 stop+start。
    other = runtime.apply(
        RuntimeRequest(
            request_id="extend-other-map",
            mode=RuntimeMode.extend_mapping,
            artifacts=MapArtifacts(
                map_id="map-b",
                pcd_path=Path("/maps/map-b/map.pcd"),
                yaml_path=Path("/maps/map-b/map.yaml"),
            ),
        )
    )
    assert other.accepted
    assert other.transition_id != second.transition_id
    assert len(processes.events) > len(events_after_first)


def test_error_snapshot_is_never_short_circuited() -> None:
    # error 快照下重试同一模式必须真正重启管线恢复, 不能被幂等短路吞掉。
    processes = FakeProcesses()
    runtime = RuntimeSupervisor(processes)
    runtime.apply(
        RuntimeRequest(request_id="start", mode=RuntimeMode.new_mapping)
    )
    processes.failures = {"point_lio": 1}
    runtime.check_health()
    assert runtime.snapshot.current_mode is RuntimeMode.error

    events_before_retry = list(processes.events)
    retry = runtime.apply(
        RuntimeRequest(request_id="recover", mode=RuntimeMode.new_mapping)
    )

    assert retry.accepted
    assert len(processes.events) > len(events_before_retry)
    assert runtime.snapshot.current_mode is RuntimeMode.new_mapping


def test_navigation_requires_a_complete_3d_map_bundle() -> None:
    runtime = RuntimeSupervisor(FakeProcesses())

    result = runtime.apply(
        RuntimeRequest(request_id="bad-map", mode=RuntimeMode.navigation)
    )

    assert not result.accepted
    assert "artifacts" in result.message
    assert runtime.snapshot.current_mode is RuntimeMode.idle


def test_production_runtime_starts_lidar_lazily_with_a_health_grace() -> None:
    processes = FakeProcesses()
    clock = ManualClock()
    runtime = RuntimeSupervisor(
        processes,
        require_driver_health=True,
        driver_startup_timeout_s=5.0,
        clock=clock,
    )

    result = runtime.apply(
        RuntimeRequest(request_id="no-lidar", mode=RuntimeMode.new_mapping)
    )

    assert result.accepted
    processes.events.clear()
    clock.now = 4.9
    runtime.update_driver_health(False)
    assert processes.events == []

    clock.now = 5.0
    runtime.update_driver_health(False)
    assert processes.events == [("stop", False)]
    assert runtime.snapshot.current_mode is RuntimeMode.error
    assert "LiDAR" in runtime.snapshot.last_error


def test_lidar_dropout_stops_an_active_pipeline() -> None:
    processes = FakeProcesses()
    runtime = RuntimeSupervisor(processes, require_driver_health=True)
    runtime.apply(RuntimeRequest('map', RuntimeMode.new_mapping))
    runtime.update_driver_health(True)
    processes.events.clear()

    runtime.update_driver_health(False)

    assert processes.events == [('stop', False)]
    assert runtime.snapshot.current_mode is RuntimeMode.error
    assert 'LiDAR' in runtime.snapshot.last_error


def test_localization_health_controls_navigation_readiness() -> None:
    runtime = RuntimeSupervisor(FakeProcesses())
    runtime.apply(
        RuntimeRequest(
            request_id="nav",
            mode=RuntimeMode.navigation,
            artifacts=artifacts(),
        )
    )

    runtime.update_localization(
        localized=True,
        fitness_score=0.18,
        inlier_ratio=0.72,
    )

    assert runtime.snapshot.current_mode is RuntimeMode.navigation
    assert runtime.snapshot.localized
    assert runtime.snapshot.fitness_score == 0.18
    assert runtime.snapshot.inlier_ratio == 0.72


def test_localized_navigation_switches_to_extend_without_losing_gicp() -> None:
    processes = FakeProcesses()
    runtime = RuntimeSupervisor(processes)
    map_artifacts = artifacts()
    runtime.apply(RuntimeRequest('nav-first', RuntimeMode.navigation, map_artifacts))
    runtime.update_localization(
        localized=True, fitness_score=0.1, inlier_ratio=0.8
    )
    processes.events.clear()

    result = runtime.apply(
        RuntimeRequest('extend', RuntimeMode.extend_mapping, map_artifacts)
    )

    assert result.accepted
    assert processes.events == [('prepare_extend_mapping', map_artifacts)]
    assert runtime.snapshot.current_mode is RuntimeMode.extend_mapping
    assert runtime.snapshot.localized


def test_stale_localization_status_relocks_navigation() -> None:
    processes = FakeProcesses()
    clock = ManualClock()
    runtime = RuntimeSupervisor(
        processes,
        localization_timeout_s=3.0,
        clock=clock,
    )
    runtime.apply(RuntimeRequest('nav', RuntimeMode.navigation, artifacts()))
    runtime.update_localization(
        localized=True, fitness_score=0.1, inlier_ratio=0.8
    )
    processes.events.clear()

    clock.now = 3.1
    runtime.check_health()

    assert processes.events == [('stop', False)]
    assert runtime.snapshot.current_mode is RuntimeMode.error
    assert not runtime.snapshot.localized
    assert 'stale' in runtime.snapshot.last_error.lower()


def test_mapping_save_hold_pauses_stale_localization_fail_closed() -> None:
    processes = FakeProcesses()
    clock = ManualClock()
    runtime = RuntimeSupervisor(
        processes,
        localization_timeout_s=3.0,
        clock=clock,
    )
    runtime.apply(RuntimeRequest('nav', RuntimeMode.navigation, artifacts()))
    runtime.update_localization(
        localized=True, fitness_score=0.1, inlier_ratio=0.8
    )
    processes.events.clear()

    runtime.set_mapping_save_hold(True)
    clock.now = 30.0
    runtime.check_health()

    assert processes.events == []
    assert runtime.snapshot.current_mode is RuntimeMode.navigation
    assert runtime.snapshot.localized

    runtime.set_mapping_save_hold(False)
    clock.now = 32.9
    runtime.check_health()
    assert processes.events == []
    assert runtime.snapshot.current_mode is RuntimeMode.navigation

    clock.now = 33.1
    runtime.check_health()
    assert processes.events == [('stop', False)]
    assert runtime.snapshot.current_mode is RuntimeMode.error
    assert 'stale' in runtime.snapshot.last_error.lower()


def test_unexpected_child_exit_stops_remaining_pipeline() -> None:
    processes = FakeProcesses()
    runtime = RuntimeSupervisor(processes)
    runtime.apply(RuntimeRequest('nav', RuntimeMode.navigation, artifacts()))
    runtime.update_localization(
        localized=True, fitness_score=0.1, inlier_ratio=0.8
    )
    processes.events.clear()
    processes.failures = {'gicp': 2}

    runtime.check_health()

    assert processes.events == [('stop', False)]
    assert runtime.snapshot.current_mode is RuntimeMode.error
    assert not runtime.snapshot.localized
    assert 'gicp' in runtime.snapshot.last_error


def test_health_check_cleanup_failure_still_publishes_error_snapshot() -> None:
    processes = FakeProcesses()
    runtime = RuntimeSupervisor(processes)
    runtime.apply(RuntimeRequest('nav', RuntimeMode.navigation, artifacts()))
    processes.events.clear()
    processes.poll_error = RuntimeError("orphan process group")
    processes.stop_error = RuntimeError("cannot kill group")

    runtime.check_health()

    assert processes.events == [('stop', False)]
    assert runtime.snapshot.current_mode is RuntimeMode.error
    assert not runtime.snapshot.localized
    assert 'orphan process group' in runtime.snapshot.last_error
    assert 'cannot kill group' in runtime.snapshot.last_error


def test_transition_cleanup_failure_returns_rejected_result() -> None:
    processes = FakeProcesses()
    processes.stop_error = RuntimeError("cannot stop old pipeline")
    runtime = RuntimeSupervisor(processes)

    result = runtime.apply(RuntimeRequest('map', RuntimeMode.new_mapping))

    assert not result.accepted
    assert runtime.snapshot.current_mode is RuntimeMode.error
    assert 'cannot stop old pipeline' in result.message


def test_idle_ignores_queued_localization_messages() -> None:
    runtime = RuntimeSupervisor(FakeProcesses())

    runtime.update_localization(
        localized=True, fitness_score=0.1, inlier_ratio=0.8
    )

    assert runtime.snapshot.current_mode is RuntimeMode.idle
    assert not runtime.snapshot.localized


def test_error_ignores_queued_localization_and_lidar_messages() -> None:
    processes = FakeProcesses()
    runtime = RuntimeSupervisor(processes, require_driver_health=True)
    runtime.apply(RuntimeRequest('nav', RuntimeMode.navigation, artifacts()))
    runtime.update_driver_health(True)
    runtime.update_localization(
        localized=True, fitness_score=0.1, inlier_ratio=0.8
    )

    runtime.update_driver_health(False)
    assert runtime.snapshot.current_mode is RuntimeMode.error
    assert not runtime.snapshot.localized
    assert not runtime.snapshot.driver_healthy

    runtime.update_localization(
        localized=True, fitness_score=0.01, inlier_ratio=0.99
    )
    runtime.update_driver_health(True)

    assert runtime.snapshot.current_mode is RuntimeMode.error
    assert not runtime.snapshot.localized
    assert not runtime.snapshot.driver_healthy
    assert 'LiDAR' in runtime.snapshot.last_error


def test_idle_ignores_queued_lidar_health_messages() -> None:
    runtime = RuntimeSupervisor(FakeProcesses(), require_driver_health=True)

    runtime.update_driver_health(True)

    assert runtime.snapshot.current_mode is RuntimeMode.idle
    assert not runtime.snapshot.driver_healthy


def test_nonfinite_localization_metrics_are_not_exposed_to_json_clients() -> None:
    runtime = RuntimeSupervisor(FakeProcesses())
    runtime.apply(RuntimeRequest('nav', RuntimeMode.navigation, artifacts()))

    runtime.update_localization(
        localized=False, fitness_score=float('inf'), inlier_ratio=float('nan')
    )

    assert runtime.snapshot.fitness_score == 0.0
    assert runtime.snapshot.inlier_ratio == 0.0


def test_transient_lost_fix_keeps_navigation_pipeline_alive() -> None:
    processes = FakeProcesses()
    clock = ManualClock()
    runtime = RuntimeSupervisor(
        processes,
        localization_loss_grace_s=5.0,
        clock=clock,
    )
    runtime.apply(RuntimeRequest('nav', RuntimeMode.navigation, artifacts()))
    runtime.update_localization(
        localized=True, fitness_score=0.1, inlier_ratio=0.8
    )
    processes.events.clear()

    runtime.update_localization(
        localized=False, fitness_score=2.5, inlier_ratio=0.1
    )

    assert processes.events == []
    assert runtime.snapshot.current_mode is RuntimeMode.navigation
    assert not runtime.snapshot.localized
    assert 'waiting for recovery' in runtime.snapshot.last_error.lower()

    clock.now = 4.9
    runtime.update_localization(
        localized=True, fitness_score=0.2, inlier_ratio=0.75
    )
    runtime.check_health()

    assert processes.events == []
    assert runtime.snapshot.current_mode is RuntimeMode.navigation
    assert runtime.snapshot.localized
    assert runtime.snapshot.last_error == ''


def test_persistent_lost_fix_stops_navigation_after_grace_period() -> None:
    processes = FakeProcesses()
    clock = ManualClock()
    runtime = RuntimeSupervisor(
        processes,
        localization_loss_grace_s=5.0,
        clock=clock,
    )
    runtime.apply(RuntimeRequest('nav', RuntimeMode.navigation, artifacts()))
    runtime.update_localization(
        localized=True, fitness_score=0.1, inlier_ratio=0.8
    )
    processes.events.clear()
    runtime.update_localization(
        localized=False, fitness_score=2.5, inlier_ratio=0.1
    )

    clock.now = 5.1
    runtime.check_health()

    assert processes.events == [('stop', False)]
    assert runtime.snapshot.current_mode is RuntimeMode.error
    assert not runtime.snapshot.localized
    assert 'lost' in runtime.snapshot.last_error.lower()


def test_unlocalized_updates_are_expected_during_initial_alignment() -> None:
    processes = FakeProcesses()
    runtime = RuntimeSupervisor(processes)
    runtime.apply(RuntimeRequest('nav', RuntimeMode.navigation, artifacts()))
    processes.events.clear()

    runtime.update_localization(
        localized=False, fitness_score=2.5, inlier_ratio=0.1
    )

    assert processes.events == []
    assert runtime.snapshot.current_mode is RuntimeMode.localizing


def test_mode_switch_keeps_lidar_driver_in_standby() -> None:
    # Livox 驱动与模式无关: 活动模式之间切换只停上层组, 免掉雷达重连。
    processes = FakeProcesses()
    runtime = RuntimeSupervisor(processes)
    runtime.apply(RuntimeRequest("map", RuntimeMode.new_mapping))
    processes.events.clear()

    runtime.apply(RuntimeRequest("nav", RuntimeMode.navigation, artifacts()))

    assert processes.events == [
        ("stop", True),
        ("start_navigation", artifacts()),
    ]


def test_idle_standby_stops_driver_only_after_timeout() -> None:
    # idle = 待机: 驱动留在原地, 超时后才由 check_health 关停 (且只关一次)。
    processes = FakeProcesses()
    clock = ManualClock()
    runtime = RuntimeSupervisor(
        processes, driver_idle_timeout_s=300.0, clock=clock
    )
    runtime.apply(RuntimeRequest("map", RuntimeMode.new_mapping))
    runtime.apply(RuntimeRequest("park", RuntimeMode.idle))
    processes.events.clear()

    clock.now = 299.0
    runtime.check_health()
    assert processes.events == []

    clock.now = 300.1
    runtime.check_health()
    assert processes.events == [("stop_driver", None)]

    clock.now = 900.0
    runtime.check_health()
    assert processes.events == [("stop_driver", None)]


def test_new_request_within_idle_standby_window_cancels_driver_stop() -> None:
    processes = FakeProcesses()
    clock = ManualClock()
    runtime = RuntimeSupervisor(
        processes, driver_idle_timeout_s=300.0, clock=clock
    )
    runtime.apply(RuntimeRequest("map", RuntimeMode.new_mapping))
    runtime.apply(RuntimeRequest("park", RuntimeMode.idle))

    clock.now = 100.0
    runtime.apply(RuntimeRequest("map-2", RuntimeMode.new_mapping))
    processes.events.clear()

    clock.now = 900.0
    runtime.check_health()
    assert ("stop_driver", None) not in processes.events


def test_navigation_map_switch_hot_swaps_without_restart() -> None:
    # navigation→navigation 换图走热切换: 不 stop 不 start, 状态回 localizing
    # 等新地图上的重定位。
    processes = FakeProcesses()
    processes.hot_switch_result = True
    runtime = RuntimeSupervisor(processes)
    first = runtime.apply(
        RuntimeRequest("nav-a", RuntimeMode.navigation, artifacts())
    )
    runtime.update_localization(
        localized=True, fitness_score=0.1, inlier_ratio=0.8
    )
    processes.events.clear()
    map_b = MapArtifacts(
        map_id="map-b",
        pcd_path=Path("/maps/map-b/map.pcd"),
        yaml_path=Path("/maps/map-b/map.yaml"),
    )

    result = runtime.apply(RuntimeRequest("nav-b", RuntimeMode.navigation, map_b))

    assert result.accepted
    assert result.transition_id != first.transition_id
    assert processes.events == [("try_switch_navigation_map", map_b, True)]
    assert runtime.snapshot.current_mode is RuntimeMode.localizing
    assert runtime.snapshot.desired_mode is RuntimeMode.navigation
    assert runtime.snapshot.active_map_id == "map-b"
    assert not runtime.snapshot.localized


def test_navigation_map_switch_falls_back_to_restart_when_hot_swap_fails() -> None:
    processes = FakeProcesses()
    processes.hot_switch_result = False
    runtime = RuntimeSupervisor(processes)
    runtime.apply(RuntimeRequest("nav-a", RuntimeMode.navigation, artifacts()))
    processes.events.clear()
    map_b = MapArtifacts(
        map_id="map-b",
        pcd_path=Path("/maps/map-b/map.pcd"),
        yaml_path=Path("/maps/map-b/map.yaml"),
    )

    result = runtime.apply(RuntimeRequest("nav-b", RuntimeMode.navigation, map_b))

    assert result.accepted
    assert processes.events == [
        ("try_switch_navigation_map", map_b, True),
        ("stop", True),
        ("start_navigation", map_b),
    ]
    assert runtime.snapshot.active_map_id == "map-b"


def test_hot_swap_is_not_attempted_from_non_navigation_modes() -> None:
    # 建图→导航没有活着的 gicp/Nav2 可复用, 必须直接整管线重启。
    processes = FakeProcesses()
    processes.hot_switch_result = True
    runtime = RuntimeSupervisor(processes)
    runtime.apply(RuntimeRequest("map", RuntimeMode.new_mapping))
    processes.events.clear()

    runtime.apply(RuntimeRequest("nav", RuntimeMode.navigation, artifacts()))

    assert processes.events == [
        ("stop", True),
        ("start_navigation", artifacts()),
    ]


def test_shutdown_stops_standby_driver() -> None:
    # destroy_node 路径: idle 只待机, shutdown 必须全停防止孤儿 Livox。
    processes = FakeProcesses()
    runtime = RuntimeSupervisor(processes)
    runtime.apply(RuntimeRequest("map", RuntimeMode.new_mapping))
    runtime.apply(RuntimeRequest("park", RuntimeMode.idle))
    processes.events.clear()

    runtime.shutdown()

    assert processes.events == [("stop", False)]


def test_same_map_nav_grid_refresh_keeps_localization() -> None:
    # 网关重烙禁区/消除区后重发同图 (yaml 路径变了): 只热载 Nav2 静态图,
    # GICP 不动, 已定位状态与当前模式原样保留。
    processes = FakeProcesses()
    processes.hot_switch_result = True
    runtime = RuntimeSupervisor(processes)
    runtime.apply(RuntimeRequest("nav-a", RuntimeMode.navigation, artifacts()))
    runtime.update_localization(
        localized=True, fitness_score=0.1, inlier_ratio=0.8
    )
    processes.events.clear()
    refreshed = MapArtifacts(
        map_id="map-a",
        pcd_path=Path("/maps/map-a/map.pcd"),
        yaml_path=Path("/maps/map-a.nav/regions-abc123.yaml"),
    )

    result = runtime.apply(
        RuntimeRequest("regions-1", RuntimeMode.navigation, refreshed)
    )

    assert result.accepted
    assert processes.events == [
        ("try_switch_navigation_map", refreshed, False),
    ]
    assert runtime.snapshot.current_mode is RuntimeMode.navigation
    assert runtime.snapshot.localized
    assert runtime.snapshot.active_map_id == "map-a"

    # 相同工件再来一次 = 重复请求, 短路不动管线。
    events_after = list(processes.events)
    retry = runtime.apply(
        RuntimeRequest("regions-2", RuntimeMode.navigation, refreshed)
    )
    assert retry.accepted
    assert retry.message == "already active"
    assert processes.events == events_after
