"""Pure, testable state machine for the SLAM process supervisor."""

from __future__ import annotations

import math
import threading
import time
import uuid
from dataclasses import dataclass, replace
from enum import Enum
from pathlib import Path
from typing import Callable, Protocol


class RuntimeMode(str, Enum):
    """Externally visible and transitional runtime modes."""

    idle = "idle"
    new_mapping = "new_mapping"
    localizing = "localizing"
    navigation = "navigation"
    extend_mapping = "extend_mapping"
    error = "error"


@dataclass(frozen=True)
class MapArtifacts:
    """The immutable 3D and 2D inputs required to activate a map."""

    map_id: str
    pcd_path: Path
    yaml_path: Path

    def validate(self) -> None:
        if not self.map_id.strip():
            raise ValueError("map artifacts require a map_id")
        if self.pcd_path.suffix.lower() != ".pcd":
            raise ValueError("map artifacts require a PCD file")
        if self.yaml_path.suffix.lower() not in {".yaml", ".yml"}:
            raise ValueError("map artifacts require a Nav2 YAML file")


@dataclass(frozen=True)
class RuntimeRequest:
    request_id: str
    mode: RuntimeMode
    artifacts: MapArtifacts | None = None


@dataclass(frozen=True)
class RuntimeResult:
    accepted: bool
    transition_id: str
    message: str


@dataclass(frozen=True)
class RuntimeSnapshot:
    current_mode: RuntimeMode = RuntimeMode.idle
    desired_mode: RuntimeMode = RuntimeMode.idle
    transition_id: str = ""
    active_map_id: str = ""
    localized: bool = False
    fitness_score: float = 0.0
    inlier_ratio: float = 0.0
    driver_healthy: bool = False
    last_error: str = ""


class ProcessBackend(Protocol):
    """Owns every on-demand SLAM/Nav2 child process."""

    def stop_pipeline(self, *, keep_driver: bool = False) -> None: ...

    def stop_driver(self) -> None: ...

    def driver_running(self) -> bool: ...

    def start_new_mapping(self) -> None: ...

    def start_navigation(self, artifacts: MapArtifacts) -> None: ...

    def start_extend_mapping(self, artifacts: MapArtifacts) -> None: ...

    def prepare_extend_mapping(self, artifacts: MapArtifacts) -> None: ...

    def try_switch_navigation_map(
        self, artifacts: MapArtifacts, *, reload_gicp: bool = True
    ) -> bool: ...

    def poll_failed_processes(self) -> dict[str, int]: ...


class RuntimeSupervisor:
    """Serializes mode transitions and makes repeated requests idempotent."""

    def __init__(
        self,
        processes: ProcessBackend,
        *,
        require_driver_health: bool = False,
        driver_startup_timeout_s: float = 5.0,
        localization_timeout_s: float = 5.0,
        localization_loss_grace_s: float = 5.0,
        driver_idle_timeout_s: float = 300.0,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._processes = processes
        self._require_driver_health = require_driver_health
        self._driver_startup_timeout_s = max(0.5, driver_startup_timeout_s)
        self._localization_timeout_s = max(0.5, localization_timeout_s)
        self._localization_loss_grace_s = max(0.5, localization_loss_grace_s)
        # idle "待机"窗口: 转 idle 时 Livox 驱动先留着 (再进任何模式免雷达
        # 重连), 超过该时长没有新请求才真正关驱动省电。
        self._driver_idle_timeout_s = max(5.0, driver_idle_timeout_s)
        self._clock = clock
        self._driver_idle_since: float | None = None
        # 当前管线实际加载的地图工件。幂等短路与热切换都按**完整工件**比较
        # 而非 map_id: 网关把禁区/消除区烙成内容寻址的派生 YAML, 区域一变
        # 路径就变 — 同图重发即可识别为"导航栅格更新"并只热载 Nav2。
        self._active_artifacts: MapArtifacts | None = None
        self._lock = threading.RLock()
        self._snapshot = RuntimeSnapshot()
        self._results: dict[str, RuntimeResult] = {}
        self._last_localization_update_at: float | None = None
        self._localization_lost_at: float | None = None
        self._driver_started_at: float | None = None
        self._driver_was_healthy = False
        self._mapping_save_hold = False

    @property
    def snapshot(self) -> RuntimeSnapshot:
        with self._lock:
            return self._snapshot

    def apply(self, request: RuntimeRequest) -> RuntimeResult:
        with self._lock:
            cached = self._results.get(request.request_id)
            if cached is not None:
                return cached
            validation_error = self._validate_request(request)
            if validation_error:
                result = RuntimeResult(False, "", validation_error)
                self._results[request.request_id] = result
                return result

            if self._already_satisfies(request):
                # 网关/App 在慢转换 (如扩图→新建图要先停 3 组进程) 超时后会带
                # 全新 request_id 重发同一目标模式; 这些重复请求会在单线程
                # executor 里排队。若每个都走完整 stop+start, 刚起好的管线会被
                # 反复推倒 (App 侧表现为雷达反复重启、WS 心跳被启动风暴饿死、
                # 各种超时)。语义已满足的请求直接确认, 不动管线。
                result = RuntimeResult(
                    True, self._snapshot.transition_id, "already active"
                )
                self._results[request.request_id] = result
                return result

            transition_id = uuid.uuid4().hex
            try:
                self._transition(request, transition_id)
            except Exception as exc:
                try:
                    self._processes.stop_pipeline()
                except Exception as cleanup_exc:
                    message = f"{exc}; pipeline cleanup failed: {cleanup_exc}"
                else:
                    message = str(exc)
                self._driver_started_at = None
                self._driver_was_healthy = False
                self._last_localization_update_at = None
                self._localization_lost_at = None
                self._mapping_save_hold = False
                self._active_artifacts = None
                self._snapshot = RuntimeSnapshot(
                    current_mode=RuntimeMode.error,
                    desired_mode=request.mode,
                    transition_id=transition_id,
                    last_error=message,
                )
                result = RuntimeResult(False, transition_id, message)
            else:
                # 真正的管线切换后, 建图进程组已被替换; 遗留的保存保持
                # 状态不能带进新模式, 否则定位失联判定被永久冻结。
                self._mapping_save_hold = False
                result = RuntimeResult(True, transition_id, "accepted")
            self._results[request.request_id] = result
            # 幂等缓存只需覆盖网关的短期重试窗口；request_id 带 uuid 后缀，
            # 不设上限会随运行时间无界增长。dict 保序，弹出最老的即可。
            while len(self._results) > 256:
                self._results.pop(next(iter(self._results)))
            return result

    def update_localization(
        self,
        *,
        localized: bool,
        fitness_score: float,
        inlier_ratio: float,
    ) -> None:
        with self._lock:
            # GICP publishes asynchronously. A final queued status can arrive
            # after idle/error has already stopped the pipeline; accepting it
            # would make clients see localized=true for a runtime with no
            # localization process behind it.
            if self._snapshot.current_mode not in {
                RuntimeMode.localizing,
                RuntimeMode.navigation,
                RuntimeMode.extend_mapping,
            } or self._snapshot.desired_mode not in {
                RuntimeMode.navigation,
                RuntimeMode.extend_mapping,
            }:
                return
            self._last_localization_update_at = self._clock()
            fitness_score = (
                float(fitness_score) if math.isfinite(fitness_score) else 0.0
            )
            inlier_ratio = (
                float(inlier_ratio) if math.isfinite(inlier_ratio) else 0.0
            )
            if localized:
                self._localization_lost_at = None
            elif (
                self._snapshot.current_mode in {
                    RuntimeMode.navigation,
                    RuntimeMode.extend_mapping,
                }
                and self._localization_lost_at is None
            ):
                self._localization_lost_at = self._clock()
            current = self._snapshot.current_mode
            if localized and current is RuntimeMode.localizing:
                current = self._snapshot.desired_mode
            self._snapshot = replace(
                self._snapshot,
                current_mode=current,
                localized=localized,
                fitness_score=fitness_score,
                inlier_ratio=inlier_ratio,
                last_error=(
                    ""
                    if localized
                    else (
                        "3D localization fix is unstable; waiting for recovery"
                        if self._localization_lost_at is not None
                        else self._snapshot.last_error
                    )
                ),
            )

    def check_health(self) -> None:
        """Relock stale localization and fail closed on child-process exit."""
        with self._lock:
            try:
                failures = self._processes.poll_failed_processes()
            except Exception as exc:
                self._fail_closed(f"SLAM process health check failed: {exc}")
                return
            if failures and self._snapshot.current_mode not in {
                RuntimeMode.idle,
                RuntimeMode.error,
            }:
                details = ", ".join(
                    f"{name} exited with code {code}"
                    for name, code in sorted(failures.items())
                )
                self._fail_closed(details)
                return

            if (
                self._driver_idle_since is not None
                and self._clock() - self._driver_idle_since
                > self._driver_idle_timeout_s
            ):
                # idle 待机窗口用完, 关掉最后剩下的 Livox 驱动省电。
                try:
                    self._processes.stop_driver()
                except Exception:
                    # 关不掉就退避一个完整窗口再试, 避免每秒重试刷错误。
                    self._driver_idle_since = self._clock()
                else:
                    self._driver_idle_since = None

            if self._mapping_save_hold:
                # Point-LIO 地图保存会把 500Hz 主循环阻塞数秒到数十秒,
                # 期间 odom/TF 停发, GICP 无输入是预期现象而非定位失联。
                # 保存期间冻结定位失联判定, 否则 extend_mapping 保存必然
                # 被 fail-closed 中途击杀 (进程崩溃检查仍照常执行)。
                return

            lost_at = self._localization_lost_at
            if (
                lost_at is not None
                and self._clock() - lost_at > self._localization_loss_grace_s
            ):
                self._fail_closed("3D localization fix was lost")
                return

            last_update = self._last_localization_update_at
            if (
                self._snapshot.localized
                and last_update is not None
                and self._clock() - last_update > self._localization_timeout_s
            ):
                self._fail_closed("3D localization status is stale")

    def set_mapping_save_hold(self, saving: bool) -> None:
        """Freeze localization-loss checks while the mapper is saving."""
        with self._lock:
            if saving == self._mapping_save_hold:
                return
            self._mapping_save_hold = saving
            if not saving:
                # 保存结束后从头计时: hold 期间积累的陈旧时间戳不能在下一个
                # tick 立即触发 fail-closed, 给 GICP 一个完整恢复窗口。
                now = self._clock()
                if self._localization_lost_at is not None:
                    self._localization_lost_at = now
                if self._last_localization_update_at is not None:
                    self._last_localization_update_at = now

    def update_driver_health(self, healthy: bool) -> None:
        with self._lock:
            active = self._snapshot.current_mode not in {
                RuntimeMode.idle,
                RuntimeMode.error,
            }
            # Livox health frames can also be queued while its launch group is
            # shutting down. Never resurrect driver health for an idle/error
            # snapshot after ownership of the driver has been released.
            if not active:
                if self._snapshot.driver_healthy:
                    self._snapshot = replace(
                        self._snapshot, driver_healthy=False
                    )
                return
            if healthy:
                self._driver_was_healthy = True
                self._snapshot = replace(self._snapshot, driver_healthy=True)
                return

            within_startup_grace = (
                active
                and not self._driver_was_healthy
                and self._driver_started_at is not None
                and self._clock() - self._driver_started_at
                < self._driver_startup_timeout_s
            )
            if self._require_driver_health and active and not within_startup_grace:
                self._fail_closed("LiDAR data stream was lost")
            else:
                self._snapshot = replace(self._snapshot, driver_healthy=False)

    def shutdown(self) -> None:
        """Full stop including the standby driver.

        进程退出路径必须真正关驱动 —— destroy_node 只 apply(idle) 的话, 待机
        的 Livox 会变成无人监管的孤儿组 (直到下一次 manager 启动清 registry)。
        """
        with self._lock:
            self._driver_idle_since = None
            try:
                self._processes.stop_pipeline()
            except Exception:
                pass

    def _fail_closed(self, message: str) -> None:
        """Stop every owned process while preserving a stable error snapshot."""
        self._driver_idle_since = None
        self._active_artifacts = None
        self._mapping_save_hold = False
        try:
            self._processes.stop_pipeline()
        except Exception as cleanup_exc:
            message = f"{message}; pipeline cleanup failed: {cleanup_exc}"
        self._snapshot = replace(
            self._snapshot,
            current_mode=RuntimeMode.error,
            driver_healthy=False,
            localized=False,
            last_error=message,
        )
        self._driver_started_at = None
        self._driver_was_healthy = False
        self._last_localization_update_at = None
        self._localization_lost_at = None

    def _already_satisfies(self, request: RuntimeRequest) -> bool:
        """请求的目标模式与当前快照语义一致 (同模式幂等短路)。

        error 快照永不短路 —— 用户重试同一模式必须真正重启管线恢复。
        """
        snapshot = self._snapshot
        if snapshot.desired_mode is not request.mode:
            return False
        if snapshot.current_mode is RuntimeMode.error:
            return False
        if request.mode in {RuntimeMode.navigation, RuntimeMode.extend_mapping}:
            # 按完整工件比较: 网关的区域派生 YAML 变了 = 不是重复请求,
            # 需要走 (热) 切换把新导航栅格载给 Nav2。
            if (
                request.artifacts is None
                or self._active_artifacts != request.artifacts
            ):
                return False
            # localizing 是这两个模式的合法收敛中间态。
            return snapshot.current_mode in {
                request.mode,
                RuntimeMode.localizing,
            }
        return snapshot.current_mode is request.mode

    def _validate_request(self, request: RuntimeRequest) -> str:
        if not request.request_id.strip():
            return "request_id is required"
        needs_map = request.mode in {
            RuntimeMode.navigation,
            RuntimeMode.extend_mapping,
        }
        if needs_map and request.artifacts is None:
            return "map artifacts are required for this mode"
        if request.artifacts is not None:
            try:
                request.artifacts.validate()
            except ValueError as exc:
                return str(exc)
        if request.mode is RuntimeMode.localizing or request.mode is RuntimeMode.error:
            return f"{request.mode.value} is an internal mode"
        return ""

    def _transition(self, request: RuntimeRequest, transition_id: str) -> None:
        self._driver_idle_since = None
        can_reuse_localization = (
            request.mode is RuntimeMode.extend_mapping
            and request.artifacts is not None
            and self._snapshot.current_mode is RuntimeMode.navigation
            and self._snapshot.localized
            and self._snapshot.active_map_id == request.artifacts.map_id
        )
        if can_reuse_localization:
            self._processes.prepare_extend_mapping(request.artifacts)
            self._active_artifacts = request.artifacts
            self._snapshot = replace(
                self._snapshot,
                current_mode=RuntimeMode.extend_mapping,
                desired_mode=RuntimeMode.extend_mapping,
                transition_id=transition_id,
                last_error="",
            )
            return

        # navigation→navigation 工件变更: 整套导航栈还活着, 优先热切换。
        # 换 3D 地图 (pcd/map_id 变) → gicp+Nav2 都换图, 回到 localizing 等
        # 重定位; 只有导航栅格变 (同图, 网关重烙了禁区/消除区) → 只热载
        # Nav2 静态图, GICP 与已定位状态原样保留。失败回退整管线重启。
        previous_artifacts = self._active_artifacts
        wants_hot_map_switch = (
            request.mode is RuntimeMode.navigation
            and request.artifacts is not None
            and self._snapshot.desired_mode is RuntimeMode.navigation
            and self._snapshot.current_mode
            in {RuntimeMode.navigation, RuntimeMode.localizing}
            and previous_artifacts is not None
            and previous_artifacts != request.artifacts
        )
        if wants_hot_map_switch:
            reload_gicp = (
                previous_artifacts.map_id != request.artifacts.map_id
                or previous_artifacts.pcd_path != request.artifacts.pcd_path
            )
            if self._processes.try_switch_navigation_map(
                request.artifacts, reload_gicp=reload_gicp
            ):
                self._active_artifacts = request.artifacts
                if reload_gicp:
                    self._last_localization_update_at = None
                    self._localization_lost_at = None
                    self._snapshot = replace(
                        self._snapshot,
                        current_mode=RuntimeMode.localizing,
                        transition_id=transition_id,
                        active_map_id=request.artifacts.map_id,
                        localized=False,
                        fitness_score=0.0,
                        inlier_ratio=0.0,
                        last_error="",
                    )
                else:
                    # 区域刷新: 定位没动过, 快照除 transition_id 外原样保留。
                    self._snapshot = replace(
                        self._snapshot,
                        transition_id=transition_id,
                        last_error="",
                    )
                return

        # 模式切换保留 Livox 驱动 (待机): 它与模式无关, 重启只会白付一次
        # 雷达重连。真正的全停只发生在 fail-closed / shutdown。
        self._processes.stop_pipeline(keep_driver=True)
        self._active_artifacts = None
        self._last_localization_update_at = None
        self._localization_lost_at = None
        self._driver_started_at = None
        self._driver_was_healthy = False
        active_map_id = ""
        current_mode = request.mode
        if request.mode is RuntimeMode.new_mapping:
            self._processes.start_new_mapping()
        elif request.mode is RuntimeMode.navigation:
            assert request.artifacts is not None
            self._processes.start_navigation(request.artifacts)
            self._active_artifacts = request.artifacts
            active_map_id = request.artifacts.map_id
            current_mode = RuntimeMode.localizing
        elif request.mode is RuntimeMode.extend_mapping:
            assert request.artifacts is not None
            self._processes.start_extend_mapping(request.artifacts)
            self._active_artifacts = request.artifacts
            active_map_id = request.artifacts.map_id
            current_mode = RuntimeMode.localizing

        if request.mode is not RuntimeMode.idle:
            # The backend starts the Livox driver together with the requested
            # pipeline. Health is allowed a short grace period before the
            # supervisor fails closed, so idle boot does not need a hot driver.
            self._driver_started_at = self._clock()
        elif self._processes.driver_running():
            # idle 待机: 驱动留在原地, 计时到 driver_idle_timeout_s 后由
            # check_health 关停。窗口内的任何新模式请求都免雷达重连。
            self._driver_idle_since = self._clock()

        self._snapshot = RuntimeSnapshot(
            current_mode=current_mode,
            desired_mode=request.mode,
            transition_id=transition_id,
            active_map_id=active_map_id,
            driver_healthy=False,
        )
