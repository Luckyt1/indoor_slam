"""Child-process ownership for on-demand Point-LIO, GICP and Nav2 stacks."""

from __future__ import annotations

import json
import os
import signal
import subprocess
import threading
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from pathlib import Path

from .runtime import MapArtifacts


@dataclass(frozen=True)
class ProcessRecord:
    """Identity of a process-group leader persisted across supervisor restarts."""

    pid: int
    start_time_ticks: int


class ProcessRegistry:
    """Small atomic PID registry used to clean process groups after a crash."""

    def __init__(self, path: Path) -> None:
        self.path = path

    def load(self) -> dict[str, ProcessRecord]:
        try:
            payload = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return {}
        if not isinstance(payload, dict):
            return {}
        groups: dict[str, ProcessRecord] = {}
        for name, value in payload.items():
            if not isinstance(name, str):
                continue
            if not isinstance(value, dict):
                continue
            pid = value.get("pid")
            start_time = value.get("start_time_ticks")
            if not isinstance(pid, int) or isinstance(pid, bool) or pid <= 1:
                continue
            if (
                not isinstance(start_time, int)
                or isinstance(start_time, bool)
                or start_time < 0
            ):
                continue
            groups[name] = ProcessRecord(pid, start_time)
        return groups

    def replace(self, groups: dict[str, ProcessRecord]) -> None:
        if not groups:
            try:
                self.path.unlink()
            except FileNotFoundError:
                pass
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_name(f".{self.path.name}.{os.getpid()}.tmp")
        payload = {
            name: {
                "pid": record.pid,
                "start_time_ticks": record.start_time_ticks,
            }
            for name, record in groups.items()
        }
        temporary.write_text(
            json.dumps(payload, sort_keys=True), encoding="utf-8"
        )
        os.replace(temporary, self.path)


def _default_registry_path() -> Path:
    configured = os.environ.get("BXI_SLAM_PROCESS_REGISTRY")
    if configured:
        return Path(configured)
    if os.name == "posix" and hasattr(os, "geteuid") and os.geteuid() == 0:
        return Path("/run/bxi/slam-processes.json")
    runtime_dir = os.environ.get("XDG_RUNTIME_DIR")
    if runtime_dir:
        return Path(runtime_dir) / "bxi-slam-processes.json"
    uid = os.getuid() if hasattr(os, "getuid") else os.getpid()
    return Path("/tmp") / f"bxi-slam-processes-{uid}.json"


def _default_nav_share_path() -> Path:
    """Resolve the installed nav share, with a source-tree test fallback."""
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("nav"))
    except (ImportError, LookupError):
        source_nav = Path(__file__).resolve().parents[2] / "bxi_nav"
        if source_nav.is_dir():
            return source_nav
        raise RuntimeError("the ROS package 'nav' is not installed")


class RosLaunchBackend:
    """Starts ROS launches without a shell and stops them as process groups."""

    def __init__(
        self,
        registry_path: Path | None = None,
        *,
        livox_config_path: Path | None = None,
        gicp_load_map: Callable[[Path], None] | None = None,
        nav2_load_map: Callable[[Path], None] | None = None,
        log_warning: Callable[[str], None] | None = None,
        nav_share_path: Path | None = None,
    ) -> None:
        self._lock = threading.RLock()
        self._children: dict[str, subprocess.Popen[bytes]] = {}
        self._registry = ProcessRegistry(registry_path or _default_registry_path())
        self._livox_config_path = livox_config_path
        # navigation→navigation 换图的热切换钩子 (ROS 服务调用由 node 注入;
        # 缺省 None = 不支持热切换, 走整管线重启)。
        self._gicp_load_map = gicp_load_map
        self._nav2_load_map = nav2_load_map
        self._log_warning = log_warning or (lambda _message: None)
        # Resolve lazily: mapping-only modes do not require the nav package,
        # and process-registry recovery must still work before ROS is sourced.
        self._nav_share_path = nav_share_path
        self._cleanup_stale_groups()

    def stop_pipeline(self, *, keep_driver: bool = False) -> None:
        # keep_driver: 模式切换的"待机"路径 —— Livox 驱动与模式无关, 留着它
        # 可以免掉最慢的雷达重连, 也消除 App 侧"雷达反复重启"的观感。已退出
        # 的驱动组不保留 (仍需清整组防孤儿), 由下一次 start 重新拉起。
        with self._lock:
            children = dict(self._children)
            kept: dict[str, subprocess.Popen[bytes]] = {}
            if keep_driver:
                driver = children.get("livox")
                if driver is not None and driver.poll() is None:
                    kept["livox"] = children.pop("livox")
            self._children = kept
            children = list(children.values())
        # 并行停组: 扩图/导航 runtime 有 gicp + point_lio + livox 三组, 逐组
        # SIGINT→等待→SIGKILL 最坏是各组超时之和 (30s+), 会顶穿网关 30s 的
        # set_mode 服务超时并触发 App 重试风暴 (见 RuntimeSupervisor 的同模式
        # 短路注释)。同时发信号并行等待, 总耗时退化为最慢一组; 组间没有关停
        # 顺序依赖 —— 所有进程都在被终止, 短暂的话题断流无害。
        errors: list[Exception | None] = [None] * len(children)

        def stop_at(index: int, child: subprocess.Popen[bytes]) -> None:
            try:
                self._stop_child(child)
            except Exception as exc:  # keep cleaning the remaining groups
                errors[index] = exc

        threads = [
            threading.Thread(target=stop_at, args=(index, child), daemon=True)
            for index, child in enumerate(children)
        ]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        with self._lock:
            self._persist_children()
        failures = [error for error in errors if error is not None]
        if failures:
            raise RuntimeError(
                "failed to stop one or more SLAM process groups: "
                + "; ".join(str(error) for error in failures)
            ) from failures[0]

    def poll_failed_processes(self) -> dict[str, int]:
        failures: dict[str, int] = {}
        exited_children: list[subprocess.Popen[bytes]] = []
        with self._lock:
            for name, child in list(self._children.items()):
                return_code = child.poll()
                if return_code is not None:
                    failures[name] = int(return_code)
                    exited_children.append(child)
                    self._children.pop(name, None)
            if failures:
                self._persist_children()
        # ros2 launch 的父进程退出后，节点子进程仍可能留在原进程组。即使
        # leader 已退出也要清整组，不能仅从 registry 丢掉 PID。
        cleanup_errors: list[Exception] = []
        for child in exited_children:
            try:
                self._stop_child(child)
            except Exception as exc:
                cleanup_errors.append(exc)
        if cleanup_errors:
            details = ", ".join(
                f"{name} exited with code {code}"
                for name, code in sorted(failures.items())
            )
            raise RuntimeError(
                f"{details}; failed to clean exited process groups: "
                + "; ".join(str(error) for error in cleanup_errors)
            ) from cleanup_errors[0]
        return failures

    def stop_driver(self) -> None:
        """Stops a standby Livox driver (idle timeout / shutdown)."""
        self._stop_named("livox")

    def driver_running(self) -> bool:
        with self._lock:
            child = self._children.get("livox")
            return child is not None and child.poll() is None

    def try_switch_navigation_map(
        self, artifacts: MapArtifacts, *, reload_gicp: bool = True
    ) -> bool:
        """navigation→navigation 工件热切换: gicp/Nav2 原地换图, 不重启进程。

        reload_gicp=False 是"同一张 3D 地图, 只有导航栅格变了"(网关重烙了
        禁区/消除区) 的快路径 — 只让 Nav2 重载静态图, 定位完全不动。
        要求整套导航栈四组进程都在运行且钩子可用; 任一步失败返回 False,
        由 supervisor 回退到整管线重启 (语义与旧行为一致, 只是慢)。
        """
        if self._gicp_load_map is None or self._nav2_load_map is None:
            return False
        with self._lock:
            running = {
                name
                for name, child in self._children.items()
                if child.poll() is None
            }
        if not {"livox", "point_lio", "gicp", "nav2"} <= running:
            return False
        try:
            if reload_gicp:
                self._gicp_load_map(artifacts.pcd_path)
            self._nav2_load_map(artifacts.yaml_path)
        except Exception as exc:
            self._log_warning(
                f"navigation map hot-swap failed, falling back to restart: {exc}"
            )
            return False
        return True

    def start_new_mapping(self) -> None:
        self._start_livox_driver()
        self._start_point_lio()

    def start_navigation(self, artifacts: MapArtifacts) -> None:
        self._start_livox_driver()
        # indoor_navigation_launch.py 已有 Nav2 使用的唯一 /scan 发布器。
        # 这里关闭 Point-LIO launch 自带的建图预览 scan，避免同一话题双源
        # 交错、时间戳抖动和重复点云投影 CPU 开销。
        self._start_point_lio(mapping_scan=False)
        self._start_gicp(artifacts)
        self._start(
            "nav2",
            [
                "ros2", "launch", "nav", "indoor_navigation_launch.py",
                f"map:={artifacts.yaml_path}", "autostart:=true", "rviz:=false",
            ]
        )

    def start_extend_mapping(self, artifacts: MapArtifacts) -> None:
        self.start_new_mapping()
        self._start_gicp(artifacts)

    def prepare_extend_mapping(self, _artifacts: MapArtifacts) -> None:
        """Keep Point-LIO/GICP and remove Nav2's static /map publisher."""
        self._stop_named("nav2")
        # 快路径复用的是导航模式的 point_lio 组 (mapping_scan:=False, /scan
        # 由 nav2 组提供)。nav2 停掉后 /scan 就没有发布者了, App 的轻量激光
        # 叠加会消失 —— 补一个独立的 scan 转换节点组, 参数与建图 launch 的
        # mapping_pointcloud_to_laserscan 保持一致。
        self._start_mapping_scan()

    def _start_mapping_scan(self) -> None:
        nav_share_path = self._nav_share_path or _default_nav_share_path()
        self._start(
            "mapping_scan",
            [
                "ros2", "run", "pointcloud_to_laserscan",
                "pointcloud_to_laserscan_node",
                "--ros-args",
                "--params-file", str(nav_share_path / "config" / "scan_params.yaml"),
                "-r", "__node:=mapping_pointcloud_to_laserscan",
                "-r", "cloud_in:=/cloud_registered",
                "-r", "scan:=/scan",
                "-p", "transform_tolerance:=0.05",
            ],
        )

    def _start_point_lio(self, *, mapping_scan: bool = True) -> None:
        command = [
            "ros2", "launch", "point_lio",
            "point_lio_with_mapping_control.launch.py", "rviz:=False",
        ]
        if not mapping_scan:
            command.append("mapping_scan:=False")
        self._start(
            "point_lio",
            command,
        )

    def _start_livox_driver(self) -> None:
        # 待机复用: stop_pipeline(keep_driver=True) 留下的驱动直接继续用,
        # 重启它只会白付一次最慢的雷达重连。
        with self._lock:
            existing = self._children.get("livox")
            if existing is not None and existing.poll() is None:
                return
        command = [
            "ros2",
            "launch",
            "livox_ros_driver2",
            "msg_MID360s_launch.py",
        ]
        if self._livox_config_path is not None:
            command.append(
                f"user_config_path:={self._livox_config_path.as_posix()}"
            )
        self._start("livox", command)

    def _start_gicp(self, artifacts: MapArtifacts) -> None:
        self._start(
            "gicp",
            [
                "ros2", "launch", "small_gicp_relocalization",
                "small_gicp_relocalization_launch.py",
                f"prior_pcd_file:={artifacts.pcd_path}",
            ]
        )

    def _start(self, name: str, command: Sequence[str]) -> None:
        self._stop_named(name)
        child = subprocess.Popen(
            list(command),
            env={**os.environ},
            start_new_session=True,
        )
        with self._lock:
            self._children[name] = child
            self._persist_children()

    def _stop_named(self, name: str) -> None:
        with self._lock:
            child = self._children.pop(name, None)
        if child is not None:
            self._stop_child(child)
        with self._lock:
            self._persist_children()

    @staticmethod
    def _stop_child(child: subprocess.Popen[bytes]) -> None:
        parent_running = child.poll() is None
        if os.name == "posix":
            try:
                os.killpg(child.pid, signal.SIGINT)
            except ProcessLookupError:
                return
            try:
                if parent_running:
                    child.wait(timeout=5)
                # ros2 launch may reap its own leader before every node in the
                # process group has exited. Always verify the whole group,
                # otherwise those children become untracked after the registry
                # entry is removed and keep LiDAR/SLAM/Nav2 running in idle.
                RosLaunchBackend._wait_for_group_exit(child.pid, timeout_s=2.0)
                return
            except subprocess.TimeoutExpired:
                pass
            try:
                os.killpg(child.pid, getattr(signal, "SIGKILL", 9))
            except ProcessLookupError:
                return
            if parent_running:
                try:
                    child.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    # The process-group check below is authoritative and also
                    # covers launch children after their leader gets wedged.
                    pass
            RosLaunchBackend._wait_for_group_exit(child.pid, timeout_s=2.0)
            return

        if not parent_running:
            return
        child.terminate()
        try:
            child.wait(timeout=5)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait(timeout=2)

    @staticmethod
    def _wait_for_group_exit(process_group: int, *, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                os.killpg(process_group, 0)
            except ProcessLookupError:
                return
            time.sleep(0.05)
        raise subprocess.TimeoutExpired(str(process_group), timeout_s)

    def _persist_children(self) -> None:
        try:
            self._registry.replace(
                {
                    name: ProcessRecord(child.pid, start_time)
                    for name, child in self._children.items()
                    if (
                        start_time := self._read_process_start_time(child.pid)
                    ) is not None
                }
            )
        except OSError:
            # Registry recovery is defense-in-depth; never prevent an immediate
            # stop/start operation because the runtime directory is read-only.
            pass

    def _cleanup_stale_groups(self) -> None:
        stale = self._registry.load()
        try:
            self._registry.replace({})
        except OSError:
            pass
        if os.name != "posix":
            return
        for record in stale.values():
            if self._registered_group_is_owned(record):
                self._terminate_stale_group(record.pid)

    @staticmethod
    def _read_process_start_time(pid: int) -> int | None:
        """Read Linux `/proc/<pid>/stat` field 22 without misparsing comm."""
        if os.name != "posix":
            return None
        try:
            stat = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
            after_comm = stat[stat.rfind(")") + 2 :].split()
            return int(after_comm[19])
        except (OSError, ValueError, IndexError):
            return None

    @classmethod
    def _registered_group_is_owned(cls, record: ProcessRecord) -> bool:
        """Refuse to signal a recycled PID from a stale registry file."""
        try:
            if os.getpgid(record.pid) != record.pid:
                return False
        except (ProcessLookupError, PermissionError, OSError):
            return False

        return cls._read_process_start_time(record.pid) == record.start_time_ticks

    @staticmethod
    def _terminate_stale_group(process_group: int) -> None:
        try:
            os.killpg(process_group, signal.SIGINT)
        except ProcessLookupError:
            return
        except PermissionError:
            return
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            try:
                os.killpg(process_group, 0)
            except ProcessLookupError:
                return
            except PermissionError:
                return
            time.sleep(0.05)
        try:
            os.killpg(process_group, signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass
