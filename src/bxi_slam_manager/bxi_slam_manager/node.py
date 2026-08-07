"""ROS 2 adapter for the pure runtime supervisor."""

from __future__ import annotations

import threading
import time
from pathlib import Path

import rclpy
from bxi_nav_interfaces.msg import MappingStatus, RelocalizationStatus, RuntimeStatus
from bxi_nav_interfaces.srv import SetRuntimeMode
from nav2_msgs.srv import ClearEntireCostmap, LoadMap
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Empty

from .process_backend import RosLaunchBackend
from .runtime import MapArtifacts, RuntimeMode, RuntimeRequest, RuntimeSupervisor


_MODE_BY_VALUE = {
    SetRuntimeMode.Request.MODE_IDLE: RuntimeMode.idle,
    SetRuntimeMode.Request.MODE_NEW_MAPPING: RuntimeMode.new_mapping,
    SetRuntimeMode.Request.MODE_NAVIGATION: RuntimeMode.navigation,
    SetRuntimeMode.Request.MODE_EXTEND_MAPPING: RuntimeMode.extend_mapping,
}

_LIDAR_HEALTH_TIMEOUT_S = 2.5
_LIDAR_SUBSCRIPTION_REFRESH_S = 5.0
_GICP_LOAD_MAP_SERVICE = "/relocalization/load_map"
_NAV2_LOAD_MAP_SERVICE = "/map_server/load_map"
_NAV2_CLEAR_COSTMAP_SERVICE = "/global_costmap/clear_entirely_global_costmap"
# 建图保存冻结定位失联判定的兜底上限: mapping_control 若在 saving 态被杀,
# 不会再发 "saved/error" 状态帧, 超过该时长后强制解除冻结恢复安全判定。
_MAPPING_SAVE_HOLD_MAX_S = 120.0


class SlamManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("bxi_slam_manager")
        configured_livox_path = str(
            self.declare_parameter("livox_config_path", "").value
        ).strip()
        driver_idle_timeout_s = float(
            self.declare_parameter("driver_idle_timeout_s", 300.0).value
        )
        # Livox MID360 冷启动/网络重连常超过 5s; 宽限过紧会让上电后第一次
        # set_mode 偶发直接 fail-closed 进 error, App 被迫重试。
        driver_startup_timeout_s = float(
            self.declare_parameter("driver_startup_timeout_s", 15.0).value
        )

        # 出站服务调用 (gicp/Nav2 热换图) 不能在本节点的单线程 executor 里
        # 自旋等待自己 —— 独立 client 节点 + 专用 spin 线程, set_mode 回调里
        # 只阻塞等 future 完成事件 (与网关 map_ros_hub 同款模式)。
        # The launch file remaps the primary node via ``__node``.  rclpy applies
        # global remaps to every Node created in this process unless explicitly
        # disabled, which used to rename this helper to ``bxi_slam_manager`` as
        # well and caused duplicate rosout publisher registration.  The helper
        # has no parameters/remaps of its own, so keep its stable private name.
        self._client_node = Node(
            "bxi_slam_manager_clients",
            use_global_arguments=False,
        )
        self._client_executor = SingleThreadedExecutor()
        self._client_executor.add_node(self._client_node)
        self._client_thread = threading.Thread(
            target=self._client_executor.spin,
            name="slam-manager-clients",
            daemon=True,
        )
        self._client_thread.start()

        self._runtime = RuntimeSupervisor(
            RosLaunchBackend(
                livox_config_path=(
                    Path(configured_livox_path) if configured_livox_path else None
                ),
                gicp_load_map=self._gicp_load_map,
                nav2_load_map=self._nav2_load_map,
                log_warning=lambda message: self.get_logger().warning(message),
            ),
            require_driver_health=True,
            driver_startup_timeout_s=driver_startup_timeout_s,
            localization_timeout_s=5.0,
            localization_loss_grace_s=5.0,
            driver_idle_timeout_s=driver_idle_timeout_s,
        )
        self._last_lidar_at = 0.0
        self._driver_expected = False
        self._last_lidar_sub_refresh_at = time.monotonic()
        self._last_driver_healthy: bool | None = None
        self._localization_epoch_ns = 0
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._status_pub = self.create_publisher(
            RuntimeStatus, "/slam/runtime/status", status_qos
        )
        self._set_mode_srv = self.create_service(
            SetRuntimeMode, "/slam/runtime/set_mode", self._on_set_mode
        )
        self._relocalization_sub = self.create_subscription(
            RelocalizationStatus,
            "/nav/relocalization_status",
            self._on_relocalization_status,
            status_qos,
        )
        self._mapping_save_hold_since: float | None = None
        self._mapping_status_sub = self.create_subscription(
            MappingStatus,
            "/mapping/status",
            self._on_mapping_status,
            status_qos,
        )
        self._lidar_sub = self._create_lidar_subscription()
        self._health_timer = self.create_timer(1.0, self._health_tick)
        self._publish_status()

    def _call_service(self, srv_type, name: str, request, timeout_s: float):
        """Blocking service call routed through the dedicated client executor."""
        client = self._client_node.create_client(srv_type, name)
        try:
            if not client.wait_for_service(timeout_sec=1.0):
                raise RuntimeError(f"service {name} is unavailable")
            future = client.call_async(request)
            done = threading.Event()
            future.add_done_callback(lambda _future: done.set())
            if not done.wait(timeout_s):
                future.cancel()
                raise TimeoutError(
                    f"service {name} timed out after {timeout_s:.0f}s"
                )
            if future.exception() is not None:
                raise RuntimeError(f"service {name} failed: {future.exception()}")
            return future.result()
        finally:
            self._client_node.destroy_client(client)

    def _gicp_load_map(self, pcd_path: Path) -> None:
        # 热换图预算必须留在网关 30s set_mode 包络内: gicp 侧同步做
        # PCD 读入+降采样+建树+协方差, 与一次冷启动同量级 (数秒)。
        request = LoadMap.Request()
        request.map_url = str(pcd_path)
        response = self._call_service(
            LoadMap, _GICP_LOAD_MAP_SERVICE, request, timeout_s=20.0
        )
        if response.result != LoadMap.Response.RESULT_SUCCESS:
            raise RuntimeError(
                f"GICP prior-map hot-swap rejected (result={response.result})"
            )

    def _nav2_load_map(self, yaml_path: Path) -> None:
        request = LoadMap.Request()
        request.map_url = str(yaml_path)
        response = self._call_service(
            LoadMap, _NAV2_LOAD_MAP_SERVICE, request, timeout_s=8.0
        )
        if response.result != LoadMap.Response.RESULT_SUCCESS:
            raise RuntimeError(
                f"Nav2 map_server load_map rejected (result={response.result})"
            )
        # 旧图的静态障碍还留在全局代价地图里, 清一次让新静态层立即生效。
        # 失败不致命 (代价地图随传感器数据自愈), 记警告即可。
        try:
            self._call_service(
                ClearEntireCostmap,
                _NAV2_CLEAR_COSTMAP_SERVICE,
                ClearEntireCostmap.Request(),
                timeout_s=3.0,
            )
        except Exception as exc:
            self.get_logger().warning(
                f"global costmap clear after map swap failed: {exc}"
            )

    def _create_lidar_subscription(self):
        lidar_qos = QoSProfile(depth=1)
        lidar_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        lidar_qos.durability = DurabilityPolicy.VOLATILE
        return self.create_subscription(
            Empty,
            "/livox/health",
            self._on_lidar,
            lidar_qos,
        )

    def _on_set_mode(self, request, response):
        mode = _MODE_BY_VALUE.get(request.mode)
        if mode is None:
            response.accepted = False
            response.message = "unsupported runtime mode"
            return response
        artifacts = None
        if request.map_id or request.pcd_path or request.yaml_path:
            artifacts = MapArtifacts(
                map_id=request.map_id,
                pcd_path=Path(request.pcd_path),
                yaml_path=Path(request.yaml_path),
            )
        previous_mode = self._runtime.snapshot.current_mode
        result = self._runtime.apply(
            RuntimeRequest(
                request_id=request.request_id,
                mode=mode,
                artifacts=artifacts,
            )
        )
        if result.accepted:
            if mode is RuntimeMode.idle:
                self._driver_expected = False
                self._last_lidar_at = 0.0
            else:
                if previous_mode in {
                    RuntimeMode.idle,
                    RuntimeMode.error,
                }:
                    self._last_lidar_at = 0.0
                self._driver_expected = True
        if result.accepted and mode in {
            RuntimeMode.navigation,
            RuntimeMode.extend_mapping,
        }:
            self._localization_epoch_ns = self.get_clock().now().nanoseconds
        response.accepted = result.accepted
        response.transition_id = result.transition_id
        response.message = result.message
        self._publish_status()
        return response

    def _on_relocalization_status(self, msg: RelocalizationStatus) -> None:
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(
            msg.header.stamp.nanosec
        )
        if stamp_ns <= self._localization_epoch_ns:
            return
        self._runtime.update_localization(
            localized=bool(msg.localized),
            fitness_score=float(msg.fitness_score),
            inlier_ratio=float(msg.inlier_ratio),
        )
        self._publish_status()

    def _on_mapping_status(self, msg: MappingStatus) -> None:
        # Point-LIO 保存会阻塞 odom/TF 数秒到数十秒; saving 期间冻结定位
        # 失联 fail-closed, 避免 extend_mapping 保存被中途击杀 (B-H2)。
        saving = msg.state == "saving"
        if saving and self._mapping_save_hold_since is None:
            self._mapping_save_hold_since = time.monotonic()
        elif not saving:
            self._mapping_save_hold_since = None
        self._runtime.set_mapping_save_hold(saving)

    def _on_lidar(self, _msg: Empty) -> None:
        if not self._driver_expected:
            return
        if self._last_lidar_at == 0.0:
            self.get_logger().info("First Livox LiDAR health frame received")
        self._last_lidar_at = time.monotonic()

    def _health_tick(self) -> None:
        now = time.monotonic()
        # fail-closed 进 error 不经过 set_mode, _driver_expected 不会被复位;
        # 若不同步, 订阅刷新逻辑会在驱动已被停掉后每 5 秒销毁重建订阅刷
        # warning。idle/error 下驱动本就不被期待。
        snapshot_mode = self._runtime.snapshot.current_mode
        if (
            self._driver_expected
            and snapshot_mode in {RuntimeMode.idle, RuntimeMode.error}
        ):
            self._driver_expected = False
            self._last_lidar_at = 0.0
        if (
            self._mapping_save_hold_since is not None
            and now - self._mapping_save_hold_since > _MAPPING_SAVE_HOLD_MAX_S
        ):
            self.get_logger().warning(
                "mapping save hold exceeded its budget; re-enabling "
                "localization safety checks"
            )
            self._mapping_save_hold_since = None
            self._runtime.set_mapping_save_hold(False)
        healthy = (
            self._driver_expected
            and self._last_lidar_at > 0.0
            and now - self._last_lidar_at < _LIDAR_HEALTH_TIMEOUT_S
        )
        if healthy != self._last_driver_healthy:
            self.get_logger().info(f"Livox LiDAR health changed: {healthy}")
            self._last_driver_healthy = healthy
        self._refresh_lidar_subscription_if_needed(now, healthy)
        self._runtime.update_driver_health(healthy)
        self._runtime.check_health()
        self._publish_status()

    def _refresh_lidar_subscription_if_needed(self, now: float, healthy: bool) -> None:
        # In idle mode the Livox driver is intentionally absent. Recreating a
        # lightweight health subscription every five seconds cannot recover
        # data that is not expected and only creates log/graph churn.
        if not self._driver_expected or healthy:
            return
        if now - self._last_lidar_sub_refresh_at < _LIDAR_SUBSCRIPTION_REFRESH_S:
            return
        self._last_lidar_sub_refresh_at = now
        self.get_logger().warning(
            "Refreshing Livox LiDAR subscription after data timeout"
        )
        try:
            self.destroy_subscription(self._lidar_sub)
        except Exception as exc:  # pragma: no cover - defensive ROS cleanup
            self.get_logger().warning(
                f"Failed to destroy stale Livox LiDAR subscription: {exc}"
            )
        self._lidar_sub = self._create_lidar_subscription()

    def _publish_status(self) -> None:
        snapshot = self._runtime.snapshot
        msg = RuntimeStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.current_mode = snapshot.current_mode.value
        msg.desired_mode = snapshot.desired_mode.value
        msg.transition_id = snapshot.transition_id
        msg.active_map_id = snapshot.active_map_id
        msg.localized = snapshot.localized
        msg.fitness_score = snapshot.fitness_score
        msg.inlier_ratio = snapshot.inlier_ratio
        msg.driver_healthy = snapshot.driver_healthy
        msg.last_error = snapshot.last_error
        self._status_pub.publish(msg)

    def destroy_node(self) -> bool:
        self._runtime.apply(
            RuntimeRequest(
                request_id=f"shutdown-{time.monotonic_ns()}",
                mode=RuntimeMode.idle,
            )
        )
        # idle 只把驱动转入待机; 进程退出必须真正全停, 否则待机的 Livox 会
        # 变成孤儿组。
        self._runtime.shutdown()
        self._client_executor.shutdown()
        self._client_node.destroy_node()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = SlamManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
