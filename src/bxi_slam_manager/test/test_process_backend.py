from pathlib import Path

import signal
import subprocess
import threading

from bxi_slam_manager.process_backend import (
    MapArtifacts,
    ProcessRecord,
    ProcessRegistry,
    RosLaunchBackend,
)


class FakeChild:
    def __init__(self, pid: int, return_code=None) -> None:
        self.pid = pid
        self.return_code = return_code

    def poll(self):
        return self.return_code


def test_process_registry_persists_groups_for_crash_recovery(tmp_path: Path) -> None:
    registry = ProcessRegistry(tmp_path / "slam-processes.json")

    registry.replace({
        "point_lio": ProcessRecord(101, 1001),
        "gicp": ProcessRecord(102, 1002),
    })

    assert registry.load() == {
        "point_lio": ProcessRecord(101, 1001),
        "gicp": ProcessRecord(102, 1002),
    }


def test_process_registry_clear_removes_stale_state(tmp_path: Path) -> None:
    registry = ProcessRegistry(tmp_path / "slam-processes.json")
    registry.replace({"nav2": ProcessRecord(103, 1003)})

    registry.replace({})

    assert registry.load() == {}
    assert not registry.path.exists()


def test_backend_reports_unexpected_child_exit(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr(
        RosLaunchBackend,
        "_read_process_start_time",
        staticmethod(lambda pid: pid * 10),
    )
    backend = RosLaunchBackend(tmp_path / "slam-processes.json")
    stopped = []
    backend._stop_child = stopped.append
    backend._children = {
        "point_lio": FakeChild(101, None),
        "gicp": FakeChild(102, 2),
    }

    assert backend.poll_failed_processes() == {"gicp": 2}
    assert set(backend._children) == {"point_lio"}
    persisted = ProcessRegistry(tmp_path / "slam-processes.json").load()
    assert persisted["point_lio"].pid == 101
    assert [child.pid for child in stopped] == [102]


def test_stop_pipeline_keeps_cleaning_after_one_group_fails(tmp_path: Path) -> None:
    backend = RosLaunchBackend(tmp_path / "slam-processes.json")
    stopped = []

    def stop(child):
        stopped.append(child.pid)
        if child.pid == 102:
            raise PermissionError("cannot signal group")

    backend._stop_child = stop
    backend._children = {
        "livox": FakeChild(101, None),
        "point_lio": FakeChild(102, None),
    }

    try:
        backend.stop_pipeline()
    except RuntimeError as error:
        assert "cannot signal group" in str(error)
    else:
        raise AssertionError("stop_pipeline must report cleanup failures")

    # 并行停组后不再保证顺序, 只保证每个组都被尝试。
    assert set(stopped) == {101, 102}
    assert backend._children == {}
    assert ProcessRegistry(tmp_path / "slam-processes.json").load() == {}


def test_stop_pipeline_stops_groups_concurrently(tmp_path: Path) -> None:
    # 扩图/导航 runtime 的三组进程若顺序关停, 最坏耗时是各组超时之和, 会
    # 顶穿网关 30s 的 set_mode 超时。Barrier 要求两个 stop 同时在场 ——
    # 顺序实现会在此超时并抛 BrokenBarrierError。
    backend = RosLaunchBackend(tmp_path / "slam-processes.json")
    barrier = threading.Barrier(2, timeout=5)
    backend._stop_child = lambda child: barrier.wait()
    backend._children = {
        "livox": FakeChild(101, None),
        "point_lio": FakeChild(102, None),
    }

    backend.stop_pipeline()

    assert backend._children == {}


def test_backend_starts_livox_before_mapping_and_tracks_it(tmp_path: Path) -> None:
    backend = RosLaunchBackend(
        tmp_path / "slam-processes.json",
        livox_config_path=Path("/opt/bxi/livox.json"),
    )
    starts = []
    backend._start = lambda name, command: starts.append((name, command))

    backend.start_new_mapping()

    assert [name for name, _ in starts] == ["livox", "point_lio"]
    assert starts[0][1][-1] == "user_config_path:=/opt/bxi/livox.json"
    assert "mapping_scan:=False" not in starts[1][1]


def test_navigation_disables_mapping_scan_to_keep_single_scan_publisher(
    tmp_path: Path,
) -> None:
    backend = RosLaunchBackend(tmp_path / "slam-processes.json")
    starts = []
    backend._start = lambda name, command: starts.append((name, command))

    backend.start_navigation(
        MapArtifacts(
            map_id="map-1",
            yaml_path=Path("/maps/map.yaml"),
            pcd_path=Path("/maps/map.pcd"),
        )
    )

    point_lio_command = next(command for name, command in starts if name == "point_lio")
    nav2_command = next(command for name, command in starts if name == "nav2")
    assert "mapping_scan:=False" in point_lio_command
    assert not any(argument.startswith("params_file:=") for argument in nav2_command)
    assert [name for name, _ in starts] == ["livox", "point_lio", "gicp", "nav2"]


def test_prepare_extend_mapping_replaces_nav_scan_publisher(
    tmp_path: Path,
) -> None:
    backend = RosLaunchBackend(tmp_path / "slam-processes.json")
    stopped = []
    starts = []
    backend._stop_named = stopped.append
    backend._start = lambda name, command: starts.append((name, command))

    backend.prepare_extend_mapping(
        MapArtifacts(
            map_id="map-1",
            yaml_path=Path("/maps/map.yaml"),
            pcd_path=Path("/maps/map.pcd"),
        )
    )

    assert stopped == ["nav2"]
    assert [name for name, _ in starts] == ["mapping_scan"]
    scan_command = starts[0][1]
    assert "scan:=/scan" in scan_command
    params_index = scan_command.index("--params-file")
    assert scan_command[params_index + 1].endswith("scan_params.yaml")
    scan_params = Path(scan_command[params_index + 1]).read_text(encoding="utf-8")
    assert "range_min: 0.2" in scan_params
    assert "range_max: 12.0" in scan_params

def test_backend_cleans_registered_groups_on_supervisor_restart(
    tmp_path: Path, monkeypatch
) -> None:
    path = tmp_path / "slam-processes.json"
    ProcessRegistry(path).replace({
        "point_lio": ProcessRecord(501, 1501),
        "nav2": ProcessRecord(502, 1502),
    })
    terminated = []
    monkeypatch.setattr(
        RosLaunchBackend,
        "_registered_group_is_owned",
        classmethod(lambda cls, record: True),
    )
    monkeypatch.setattr(
        RosLaunchBackend,
        "_terminate_stale_group",
        staticmethod(terminated.append),
    )
    monkeypatch.setattr("bxi_slam_manager.process_backend.os.name", "posix")

    RosLaunchBackend(path)

    assert set(terminated) == {501, 502}
    assert not path.exists()


def test_backend_skips_recycled_registered_process_group(
    tmp_path: Path, monkeypatch
) -> None:
    path = tmp_path / "slam-processes.json"
    ProcessRegistry(path).replace({
        "point_lio": ProcessRecord(501, 1501),
    })
    terminated = []
    monkeypatch.setattr("bxi_slam_manager.process_backend.os.name", "posix")
    monkeypatch.setattr(
        RosLaunchBackend,
        "_registered_group_is_owned",
        classmethod(lambda cls, record: False),
    )
    monkeypatch.setattr(
        RosLaunchBackend,
        "_terminate_stale_group",
        staticmethod(terminated.append),
    )

    RosLaunchBackend(path)

    assert terminated == []
    assert not path.exists()


def test_stop_child_waits_for_entire_posix_group_after_leader_exits(
    monkeypatch,
) -> None:
    events = []

    class PosixProcessApi:
        name = "posix"

        @staticmethod
        def killpg(pid, sig):
            events.append(("signal", pid, sig))

    class RunningChild:
        pid = 601

        @staticmethod
        def poll():
            return None

        @staticmethod
        def wait(timeout):
            events.append(("wait_leader", timeout))
            return 0

    monkeypatch.setattr(
        "bxi_slam_manager.process_backend.os",
        PosixProcessApi,
    )
    monkeypatch.setattr(
        RosLaunchBackend,
        "_wait_for_group_exit",
        staticmethod(
            lambda pid, *, timeout_s: events.append(
                ("wait_group", pid, timeout_s)
            )
        ),
    )

    RosLaunchBackend._stop_child(RunningChild())

    assert events == [
        ("signal", 601, signal.SIGINT),
        ("wait_leader", 5),
        ("wait_group", 601, 2.0),
    ]


def test_stop_child_escalates_and_rechecks_posix_group(monkeypatch) -> None:
    events = []

    class PosixProcessApi:
        name = "posix"

        @staticmethod
        def killpg(pid, sig):
            events.append(("signal", pid, sig))

    class ExitedLeader:
        pid = 602

        @staticmethod
        def poll():
            return 1

    waits = iter([subprocess.TimeoutExpired("602", 2.0), None])
    monkeypatch.setattr("bxi_slam_manager.process_backend.os", PosixProcessApi)

    calls = []

    def wait_group(pid, *, timeout_s):
        calls.append((pid, timeout_s))
        outcome = next(waits)
        if outcome is not None:
            raise outcome

    monkeypatch.setattr(RosLaunchBackend, "_wait_for_group_exit", staticmethod(wait_group))

    RosLaunchBackend._stop_child(ExitedLeader())

    assert events == [
        ("signal", 602, signal.SIGINT),
        ("signal", 602, getattr(signal, "SIGKILL", 9)),
    ]
    assert calls == [(602, 2.0), (602, 2.0)]


def test_stop_pipeline_keeps_running_driver_in_standby(
    tmp_path: Path,
    monkeypatch,
) -> None:
    monkeypatch.setattr(
        RosLaunchBackend,
        "_read_process_start_time",
        staticmethod(lambda pid: pid * 10),
    )
    backend = RosLaunchBackend(tmp_path / "slam-processes.json")
    stopped = []
    backend._stop_child = lambda child: stopped.append(child.pid)
    livox = FakeChild(101, None)
    backend._children = {"livox": livox, "point_lio": FakeChild(102, None)}

    backend.stop_pipeline(keep_driver=True)

    assert stopped == [102]
    assert backend._children == {"livox": livox}
    persisted = ProcessRegistry(tmp_path / "slam-processes.json").load()
    assert set(persisted) == {"livox"}


def test_stop_pipeline_cleans_exited_driver_even_in_standby(
    tmp_path: Path,
) -> None:
    # 已退出的驱动组不保留: 仍要清整组防孤儿, 下次 start 重新拉起。
    backend = RosLaunchBackend(tmp_path / "slam-processes.json")
    stopped = []
    backend._stop_child = lambda child: stopped.append(child.pid)
    backend._children = {
        "livox": FakeChild(101, 3),
        "point_lio": FakeChild(102, None),
    }

    backend.stop_pipeline(keep_driver=True)

    assert set(stopped) == {101, 102}
    assert backend._children == {}


def test_start_reuses_standby_driver(tmp_path: Path) -> None:
    backend = RosLaunchBackend(tmp_path / "slam-processes.json")
    starts = []
    backend._start = lambda name, command: starts.append(name)
    backend._children = {"livox": FakeChild(101, None)}

    backend.start_new_mapping()

    assert starts == ["point_lio"]


def test_start_replaces_dead_standby_driver(tmp_path: Path) -> None:
    backend = RosLaunchBackend(tmp_path / "slam-processes.json")
    starts = []
    backend._start = lambda name, command: starts.append(name)
    backend._children = {"livox": FakeChild(101, 1)}

    backend.start_new_mapping()

    assert starts == ["livox", "point_lio"]


def _hot_swap_artifacts() -> MapArtifacts:
    return MapArtifacts(
        map_id="map-b",
        pcd_path=Path("/maps/map-b/map.pcd"),
        yaml_path=Path("/maps/map-b/map.yaml"),
    )


def test_navigation_map_hot_swap_requires_hooks(tmp_path: Path) -> None:
    backend = RosLaunchBackend(tmp_path / "slam-processes.json")
    backend._children = {
        name: FakeChild(pid, None)
        for pid, name in enumerate(["livox", "point_lio", "gicp", "nav2"], 101)
    }

    assert backend.try_switch_navigation_map(_hot_swap_artifacts()) is False


def test_navigation_map_hot_swap_requires_full_running_stack(
    tmp_path: Path,
) -> None:
    calls = []
    backend = RosLaunchBackend(
        tmp_path / "slam-processes.json",
        gicp_load_map=lambda path: calls.append(("gicp", path)),
        nav2_load_map=lambda path: calls.append(("nav2", path)),
    )
    backend._children = {
        "livox": FakeChild(101, None),
        "point_lio": FakeChild(102, None),
        "gicp": FakeChild(103, None),
    }

    assert backend.try_switch_navigation_map(_hot_swap_artifacts()) is False
    assert calls == []

    backend._children["nav2"] = FakeChild(104, None)
    artifacts = _hot_swap_artifacts()
    assert backend.try_switch_navigation_map(artifacts) is True
    assert calls == [("gicp", artifacts.pcd_path), ("nav2", artifacts.yaml_path)]

    # 同图纯导航栅格刷新: 跳过 gicp, 只热载 Nav2。
    calls.clear()
    assert (
        backend.try_switch_navigation_map(artifacts, reload_gicp=False) is True
    )
    assert calls == [("nav2", artifacts.yaml_path)]


def test_navigation_map_hot_swap_reports_hook_failure(tmp_path: Path) -> None:
    warnings = []

    def broken_gicp(_path):
        raise RuntimeError("load_map service down")

    backend = RosLaunchBackend(
        tmp_path / "slam-processes.json",
        gicp_load_map=broken_gicp,
        nav2_load_map=lambda path: None,
        log_warning=warnings.append,
    )
    backend._children = {
        name: FakeChild(pid, None)
        for pid, name in enumerate(["livox", "point_lio", "gicp", "nav2"], 101)
    }

    assert backend.try_switch_navigation_map(_hot_swap_artifacts()) is False
    assert "load_map service down" in warnings[0]
