#!/usr/bin/env python3
"""面向 App 对接的 ROS 2 建图控制节点。

该节点订阅 Point-LIO 发布的配准点云，并增量生成 Nav2 风格的 2D 栅格地图。
App 通过 ROS 2 action/service/topic 控制地图累积流程：开始、暂停/继续、取消、保存、查询状态。
"""

from __future__ import annotations

import argparse
import array
import json
import math
import os
import re
import shutil
import subprocess
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

import numpy as np


# 地图名会直接用于生成文件名，只允许安全字符，避免路径穿越和奇怪文件名。
MAP_NAME_RE = re.compile(r"^[A-Za-z0-9_-]{1,64}$")


def indoor_slam_root() -> Path:
    return Path(
        os.environ.get("BXI_INDOOR_SLAM_ROOT", "/opt/bxi/bxi_rc_slam")
    ).expanduser()


def indoor_slam_path(*parts: str) -> Path:
    return indoor_slam_root().joinpath(*parts)


def sanitize_map_name(map_name: str) -> str:
    # 清理并校验地图名，确保后续可以安全拼成 .pgm/.yaml 文件路径。
    cleaned = map_name.strip()
    if not MAP_NAME_RE.fullmatch(cleaned):
        raise ValueError("map_name must be 1-64 chars: A-Z, a-z, 0-9, _ or -")
    return cleaned


def read_binary_pcd(path: Path) -> tuple[bytes, np.ndarray, np.ndarray]:
    """Read binary PCD rows while preserving every non-XYZ field byte."""
    content = path.read_bytes()
    match = re.search(br"(?m)^DATA[ \t]+binary\r?\n", content)
    if match is None:
        raise ValueError("only binary PCD files are supported")
    header = content[:match.end()]
    values: dict[str, list[str]] = {}
    for raw_line in header.decode("ascii").splitlines():
        parts = raw_line.split()
        if parts and not parts[0].startswith("#"):
            values[parts[0].upper()] = parts[1:]

    fields = values.get("FIELDS", [])
    sizes = [int(value) for value in values.get("SIZE", [])]
    types = values.get("TYPE", [])
    counts = [int(value) for value in values.get("COUNT", ["1"] * len(fields))]
    if not fields or not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise ValueError("PCD field metadata is invalid")
    point_count = int(values.get("POINTS", values.get("WIDTH", ["0"]))[0])
    point_step = sum(size * count for size, count in zip(sizes, counts))
    payload = content[match.end():]
    if point_count < 0 or point_step <= 0 or len(payload) != point_count * point_step:
        raise ValueError("PCD binary payload size does not match its header")

    rows = np.frombuffer(payload, dtype=np.uint8).reshape(point_count, point_step).copy()
    xyz_columns = []
    offset = 0
    for axis in ("x", "y", "z"):
        try:
            index = fields.index(axis)
        except ValueError as exc:
            raise ValueError(f"PCD is missing {axis} field") from exc
        offset = sum(sizes[i] * counts[i] for i in range(index))
        if sizes[index] != 4 or types[index].upper() != "F" or counts[index] != 1:
            raise ValueError(f"PCD {axis} field must be one float32")
        xyz_columns.append(np.ndarray(
            shape=(point_count,), dtype="<f4", buffer=payload,
            offset=offset, strides=(point_step,),
        ).copy())
    return header, rows, np.column_stack(xyz_columns)


def write_binary_pcd(path: Path, header: bytes, rows: np.ndarray) -> None:
    """Write filtered binary PCD rows and repair WIDTH/POINTS metadata."""
    rows = np.asarray(rows, dtype=np.uint8)
    if rows.ndim != 2:
        raise ValueError("PCD rows must be a two-dimensional uint8 array")
    count = str(rows.shape[0]).encode("ascii")
    header = re.sub(br"(?m)^WIDTH\s+\d+\r?$", b"WIDTH " + count, header)
    header = re.sub(br"(?m)^POINTS\s+\d+\r?$", b"POINTS " + count, header)
    path.write_bytes(header + rows.tobytes())


def transient_ghost_drop_mask(
    xyz: np.ndarray,
    *,
    free_since_occ: np.ndarray,
    occ_counts: np.ndarray,
    resolution: float,
    origin_x: float,
    origin_y: float,
    z_keep_below: float,
    min_free_frames: int,
) -> np.ndarray:
    """Mark historical obstacle points in cells now confirmed free."""
    drop = np.zeros(len(xyz), dtype=bool)
    if xyz.size == 0:
        return drop
    ix = np.floor((xyz[:, 0] - origin_x) / resolution).astype(np.int64)
    iy = np.floor((xyz[:, 1] - origin_y) / resolution).astype(np.int64)
    inside = (
        (ix >= 0) & (ix < free_since_occ.shape[1])
        & (iy >= 0) & (iy < free_since_occ.shape[0])
    )
    positions = np.flatnonzero(inside)
    if positions.size:
        cells_free = (
            free_since_occ[iy[positions], ix[positions]] >= max(min_free_frames, 1)
        ) & (occ_counts[iy[positions], ix[positions]] == 0)
        drop[positions] = (xyz[positions, 2] > z_keep_below) & cells_free
    return drop


def apply_rigid_transform(
    xyz: np.ndarray,
    *,
    translation: tuple[float, float, float],
    quaternion: tuple[float, float, float, float],
) -> tuple[np.ndarray, np.ndarray]:
    """Apply a map<-cloud rigid transform to an Nx3 point array."""
    qx, qy, qz, qw = quaternion
    norm = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError("transform quaternion is invalid")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    rotation = np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),
             2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz),
             2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw),
             1 - 2 * (qx * qx + qy * qy)],
        ],
        dtype=np.float64,
    )
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = np.asarray(translation, dtype=np.float64)
    transformed = np.asarray(xyz, dtype=np.float64) @ rotation.T + matrix[:3, 3]
    return transformed, matrix


def quaternion_to_yaw(
    quaternion: tuple[float, float, float, float],
) -> float:
    """Return planar yaw from a ROS quaternion."""
    qx, qy, qz, qw = quaternion
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("pose quaternion is invalid")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def unique_grid_cells(ix: np.ndarray, iy: np.ndarray, grid_width: int) -> np.ndarray:
    """Return unique flattened grid indices without sorting dense point sets.

    LiDAR frames usually occupy a compact local bounding box. Remapping that box
    lets ``bincount`` deduplicate in O(n) while avoiding an allocation sized to
    the whole (potentially multi-million-cell) map. Sparse/wide inputs retain
    ``np.unique`` as the bounded-memory fallback.
    """
    if ix.size == 0:
        return np.empty(0, dtype=np.int64)
    if ix.size == 1:
        return (iy.astype(np.int64, copy=False) * grid_width
                + ix.astype(np.int64, copy=False))

    min_x = int(np.min(ix))
    max_x = int(np.max(ix))
    min_y = int(np.min(iy))
    max_y = int(np.max(iy))
    local_width = max_x - min_x + 1
    local_height = max_y - min_y + 1
    local_cells = local_width * local_height

    # Dense local boxes benefit from bincount. Cap temporary memory to roughly
    # 8 MiB (one million int64 counters); sparse scans stay on np.unique.
    if local_cells <= max(ix.size * 12, 4096) and local_cells <= 1_000_000:
        local_indices = ((iy - min_y) * local_width + (ix - min_x)).astype(
            np.int64, copy=False
        )
        present = np.flatnonzero(np.bincount(local_indices, minlength=local_cells))
        local_y, local_x = np.divmod(present, local_width)
        return (local_y + min_y) * grid_width + (local_x + min_x)

    return np.unique(iy * grid_width + ix)


def best_effort_ros_cleanup(executor: Any, node: Any, ros: Any) -> None:
    """Tear down ROS objects without allowing a repeated SIGINT to escape."""
    try:
        executor.shutdown()
    except (Exception, KeyboardInterrupt):
        pass
    try:
        node.destroy_node()
    except (Exception, KeyboardInterrupt):
        pass
    try:
        if ros.ok():
            ros.shutdown()
    except (Exception, KeyboardInterrupt):
        pass


def _binary_dilate(mask: np.ndarray) -> np.ndarray:
    expanded = mask.copy()
    expanded[1:, :] |= mask[:-1, :]
    expanded[:-1, :] |= mask[1:, :]
    expanded[:, 1:] |= mask[:, :-1]
    expanded[:, :-1] |= mask[:, 1:]
    expanded[1:, 1:] |= mask[:-1, :-1]
    expanded[1:, :-1] |= mask[:-1, 1:]
    expanded[:-1, 1:] |= mask[1:, :-1]
    expanded[:-1, :-1] |= mask[1:, 1:]
    return expanded


def _binary_erode(mask: np.ndarray) -> np.ndarray:
    eroded = mask.copy()
    eroded[1:, :] &= mask[:-1, :]
    eroded[:-1, :] &= mask[1:, :]
    eroded[:, 1:] &= mask[:, :-1]
    eroded[:, :-1] &= mask[:, 1:]
    eroded[1:, 1:] &= mask[:-1, :-1]
    eroded[1:, :-1] &= mask[:-1, 1:]
    eroded[:-1, 1:] &= mask[1:, :-1]
    eroded[:-1, :-1] &= mask[1:, 1:]
    return eroded


def clean_occupied_mask(
    occupied: np.ndarray,
    *,
    closing_iterations: int,
    min_neighbors: int,
) -> np.ndarray:
    """Close one-cell wall seams and remove unsupported occupied speckles."""
    cleaned = np.asarray(occupied, dtype=bool).copy()

    def remove_speckles(mask: np.ndarray) -> np.ndarray:
        if min_neighbors <= 0 or not np.any(mask):
            return mask
        neighbors = np.zeros(mask.shape, dtype=np.uint8)
        neighbors[1:, :] += mask[:-1, :]
        neighbors[:-1, :] += mask[1:, :]
        neighbors[:, 1:] += mask[:, :-1]
        neighbors[:, :-1] += mask[:, 1:]
        neighbors[1:, 1:] += mask[:-1, :-1]
        neighbors[1:, :-1] += mask[:-1, 1:]
        neighbors[:-1, 1:] += mask[1:, :-1]
        neighbors[:-1, :-1] += mask[1:, 1:]
        return mask & (neighbors >= min_neighbors)

    cleaned = remove_speckles(cleaned)
    for _ in range(max(0, closing_iterations)):
        cleaned = _binary_erode(_binary_dilate(cleaned))
    return remove_speckles(cleaned)


@dataclass
class MappingConfig:
    """建图输出和点云投影参数。"""

    # 保存 .pgm 和 .yaml 的目录。
    output_dir: Path = field(
        default_factory=lambda: indoor_slam_path("src", "bxi_nav", "maps")
    )
    # 栅格分辨率，单位 m/cell。
    resolution: float = 0.05
    # 输出地图的物理宽度和高度，单位 m。
    size_x: float = 60.0
    size_y: float = 60.0
    # 输出地图左下角在 Point-LIO 世界坐标中的位置。
    origin_x: float = -30.0
    origin_y: float = -30.0
    # 落在该高度范围内的点会被当作障碍物证据。
    occupied_z_min: float = -0.8
    occupied_z_max: float = 0.3
    # 落在该高度范围内的点会被当作可通行/空闲证据。
    free_z_min: float = -1.30
    free_z_max: float = -0.35
    # 静态图保留原始障碍边界；机器人足迹和安全距离由 Nav2 统一处理。
    occupied_dilation: int = 0
    occupied_min_observations: int = 3
    free_min_observations: int = 2
    # 当前 free 证据连续达到该帧数后，清除同一世界栅格里的历史障碍。
    dynamic_clear_min_observations: int = 10
    # 可选低延迟诊断预览阈值；App /map 和最终文件统一使用上面的严格阈值。
    preview_occupied_min_observations: int = 1
    preview_free_min_observations: int = 1
    occupied_closing_iterations: int = 1
    occupied_despeckle_neighbors: int = 1
    # 扩图时一次多预留一些边界，避免机器人每越过一格就复制整张栅格。
    grid_expansion_padding: float = 5.0
    # 单帧只允许使用距当前边界不超过该距离的点触发扩图。Livox 的有效近场
    # 已远小于该值，因此这主要用于拦截坐标跳变和极远离群点。
    max_grid_expansion_per_frame: float = 40.0
    # 两个 uint32 置信度数组在 400 万格时约占 32 MiB；同时远低于 App/RC
    # 的 16 MiB 单帧栅格上限，避免异常点把机器人内存一次性吃满。
    max_live_grid_cells: int = 4_000_000
    # 使用外部 PCD 转图程序生成重定位点云；二维地图由冻结的实时栅格写出。
    pcd2pgm_executable: Path = field(
        default_factory=lambda: indoor_slam_path("pcd2pgm_headless")
    )
    pcd2pgm_config: Path = field(
        default_factory=lambda: indoor_slam_path("scans_nav2_map.cfg")
    )
    pcd_input_path: Path = field(
        default_factory=lambda: indoor_slam_path("src", "Point-LIO", "PCD", "scans.pcd")
    )
    map_store_root: Path = Path("/var/lib/bxi/maps")
    pcd_merge_executable: Path = field(
        default_factory=lambda: indoor_slam_path("merge_pcd_maps")
    )
    # Must stay below the RC gateway's 120 s ROS service timeout. This is one
    # total budget shared by Point-LIO flush, optional parent merge and
    # pcd2pgm, rather than a fresh timeout for every subprocess.
    save_timeout_sec: float = 105.0


class MappingSession:
    """单次 App 建图任务的线程安全内存状态。"""

    def __init__(
        self,
        config: MappingConfig,
        command_runner: Callable[[list[str], Path], None] | None = None,
    ) -> None:
        self.config = config
        self.command_runner = command_runner or self._run_command
        if (
            not np.isfinite(config.resolution) or config.resolution <= 0
            or not np.isfinite(config.size_x) or config.size_x <= 0
            or not np.isfinite(config.size_y) or config.size_y <= 0
            or not np.isfinite(config.origin_x)
            or not np.isfinite(config.origin_y)
            or config.dynamic_clear_min_observations <= 0
            or config.max_live_grid_cells <= 0
            or not np.isfinite(config.save_timeout_sec)
            or config.save_timeout_sec <= 0
        ):
            raise ValueError("mapping grid geometry is invalid")
        self._default_geometry = (
            float(config.resolution),
            float(config.size_x),
            float(config.size_y),
            float(config.origin_x),
            float(config.origin_y),
        )
        # /mapping/save runs in a ReentrantCallbackGroup. A gateway timeout can
        # therefore leave the first conversion running while a retry enters on
        # another executor thread. Serialize the whole "flush Point-LIO PCD +
        # convert artifacts" operation so two callbacks never delete/write the
        # same staging directory concurrently.
        self._save_operation_lock = threading.RLock()
        self._save_deadline: float | None = None
        # HTTP 请求线程和 ROS 点云回调线程会共享同一个 session，因此这里必须加锁。
        self.lock = threading.RLock()
        # 状态取值：idle/mapping/paused/saving/saved/cancelled。
        self.state = "idle"
        self.current_map_name = ""
        self.session_id = ""
        self.base_map_id = ""
        self.last_error = ""
        self.last_map_pgm_path = ""
        self.last_map_yaml_path = ""
        self.last_relocalization_pcd_path = ""
        self.map_from_cloud = np.eye(4, dtype=np.float64)
        self.base_cells: np.ndarray | None = None
        self.width = 0
        self.height = 0
        self.occ_counts = np.empty((0, 0), dtype=np.uint32)
        self.free_counts = np.empty((0, 0), dtype=np.uint32)
        self._restore_default_geometry()

    def start(
        self,
        map_name: str,
        session_id: str = "",
        base_map_id: str = "",
    ) -> dict[str, Any]:
        clean_map_name = sanitize_map_name(map_name)
        clean_session_id = session_id.strip()
        clean_base_map_id = base_map_id.strip()
        if clean_base_map_id:
            sanitize_map_name(clean_base_map_id)
        with self.lock:
            if self.state == "saving":
                raise RuntimeError("mapping save is in progress")
            # 每个任务都从默认几何和空 artifact 状态开始。扩图会改变分辨率、
            # 原点和尺寸，若这里只 fill(0)，下一次新建图就会继承上一张地图。
            self._restore_default_geometry()
            self._clear_session_metadata()
            self.current_map_name = clean_map_name
            self.session_id = clean_session_id
            self.base_map_id = clean_base_map_id
            try:
                if self.base_map_id:
                    self._seed_from_parent_grid()
            except Exception as exc:
                # 父地图损坏或缺失时不能留下半初始化 session，避免 App 随后
                # 读取到父地图的部分元数据或上一任务的 PCD/transform。
                self._restore_default_geometry()
                self._clear_session_metadata()
                self.state = "error"
                self.last_error = str(exc)
                raise
            self.state = "mapping"
            self.last_error = ""
            return self.status()

    def pause(self, paused: bool) -> dict[str, Any]:
        with self.lock:
            if not self.current_map_name or self.state not in {"mapping", "paused"}:
                raise RuntimeError("no active mapping task")
            # 暂停只影响地图累积；Point-LIO 仍继续发布里程计、点云和 TF。
            self.state = "paused" if paused else "mapping"
            return self.status()

    def cancel(self) -> dict[str, Any]:
        with self.lock:
            if self.state == "saving":
                raise RuntimeError("mapping save is in progress")
            # 取消表示放弃当前地图，因此清空所有未保存证据。
            self._restore_default_geometry()
            self._clear_session_metadata()
            self.state = "cancelled"
            return self.status()

    def active_cloud_base_map_id(self) -> str | None:
        """Return mapping context, or None when cloud decoding can be skipped."""
        with self.lock:
            return self.base_map_id if self.state == "mapping" else None

    def active_pose_base_map_id(self) -> str | None:
        """Return the active map frame context used by the live App pose."""
        with self.lock:
            if self.state not in {"mapping", "paused", "saving"}:
                return None
            return self.base_map_id

    def ingest_xyz(self, xyz: np.ndarray) -> None:
        with self.lock:
            # 在 idle/paused/saved 等状态下点云仍可能到达；只有 mapping 状态才累积。
            if self.state != "mapping" or xyz.size == 0:
                return

            # 丢弃 NaN/Inf 点，避免索引计算报错或污染地图。
            xyz = xyz[np.isfinite(xyz).all(axis=1)]
            if xyz.size == 0:
                return

            cfg = self.config
            if self.base_map_id:
                self._expand_to_include(xyz)
            # 将米制 x/y 坐标转换为固定输出栅格里的列/行索引。
            ix = np.floor((xyz[:, 0] - cfg.origin_x) / cfg.resolution).astype(np.int64)
            iy = np.floor((xyz[:, 1] - cfg.origin_y) / cfg.resolution).astype(np.int64)
            # 只保留落在输出地图范围内的点。
            keep = (ix >= 0) & (ix < self.width) & (iy >= 0) & (iy < self.height)
            ix, iy, z = ix[keep], iy[keep], xyz[keep, 2]

            # 通过高度带判断某个栅格收到的是障碍物证据还是空闲证据。
            occ = (z >= cfg.occupied_z_min) & (z <= cfg.occupied_z_max)
            free = (z >= cfg.free_z_min) & (z <= cfg.free_z_max)
            # 同一帧里落入同一栅格的数百个点只算一次时间证据。这样既避免
            # np.add.at 在重复索引上的高开销，也不会让近处高密度点云在一帧内
            # 直接越过多帧置信度阈值。
            occupied_cells = np.empty(0, dtype=np.int64)
            if np.any(occ):
                occupied_cells = unique_grid_cells(ix[occ], iy[occ], self.width)
            if np.any(free):
                free_cells = unique_grid_cells(ix[free], iy[free], self.width)
                if occupied_cells.size:
                    free_cells = np.setdiff1d(
                        free_cells, occupied_cells, assume_unique=True
                    )
                free_flat = self.free_counts.ravel()
                clear_min = max(cfg.dynamic_clear_min_observations, 1)
                free_flat[free_cells] = np.minimum(
                    free_flat[free_cells], clear_min - 1
                ) + 1
                cleared_cells = free_cells[free_flat[free_cells] >= clear_min]
                self.occ_counts.ravel()[cleared_cells] = 0
                if self.base_cells is not None:
                    self.base_cells.ravel()[cleared_cells] = 0
            if occupied_cells.size:
                occ_flat = self.occ_counts.ravel()
                occupied_min = max(cfg.occupied_min_observations, 1)
                occ_flat[occupied_cells] = np.minimum(
                    occ_flat[occupied_cells], occupied_min - 1
                ) + 1
                self.free_counts.ravel()[occupied_cells] = 0

    def clear_radius(self, center_x: float, center_y: float, radius: float) -> int:
        """Clear current evidence; later scans may occupy the same cells again."""
        if not all(np.isfinite(value) for value in (center_x, center_y, radius)):
            raise ValueError("clear center and radius must be finite")
        if radius <= 0:
            raise ValueError("clear radius must be positive")

        with self.lock:
            if not self.current_map_name or self.state not in {"mapping", "paused"}:
                raise RuntimeError("no active mapping task")

            cfg = self.config
            min_ix = max(0, int(np.floor((center_x - radius - cfg.origin_x) / cfg.resolution)))
            max_ix = min(self.width - 1, int(np.floor((center_x + radius - cfg.origin_x) / cfg.resolution)))
            min_iy = max(0, int(np.floor((center_y - radius - cfg.origin_y) / cfg.resolution)))
            max_iy = min(self.height - 1, int(np.floor((center_y + radius - cfg.origin_y) / cfg.resolution)))
            if min_ix > max_ix or min_iy > max_iy:
                return 0

            xs = cfg.origin_x + (np.arange(min_ix, max_ix + 1) + 0.5) * cfg.resolution
            ys = cfg.origin_y + (np.arange(min_iy, max_iy + 1) + 0.5) * cfg.resolution
            mask = ((xs[np.newaxis, :] - center_x) ** 2
                    + (ys[:, np.newaxis] - center_y) ** 2) <= radius ** 2
            target = np.s_[min_iy:max_iy + 1, min_ix:max_ix + 1]
            cleared = int(np.count_nonzero(mask))
            self.occ_counts[target][mask] = 0
            self.free_counts[target][mask] = max(
                self.config.free_min_observations,
                self.config.dynamic_clear_min_observations,
                1,
            )
            if self.base_cells is not None:
                self.base_cells[target][mask] = 0
            return cleared

    def _expand_to_include(self, xyz: np.ndarray) -> None:
        cfg = self.config
        resolution = cfg.resolution
        current_max_x = cfg.origin_x + self.width * resolution
        current_max_y = cfg.origin_y + self.height * resolution
        outside = (
            (xyz[:, 0] < cfg.origin_x) | (xyz[:, 0] >= current_max_x)
            | (xyz[:, 1] < cfg.origin_y) | (xyz[:, 1] >= current_max_y)
        )
        if not np.any(outside):
            return

        max_step = max(float(cfg.max_grid_expansion_per_frame), resolution)
        candidates = xyz[
            outside
            & (xyz[:, 0] >= cfg.origin_x - max_step)
            & (xyz[:, 0] <= current_max_x + max_step)
            & (xyz[:, 1] >= cfg.origin_y - max_step)
            & (xyz[:, 1] <= current_max_y + max_step)
        ]
        if candidates.size == 0:
            return

        padding = max(float(cfg.grid_expansion_padding), 0.0)
        candidate_min_x = float(np.min(candidates[:, 0]))
        candidate_min_y = float(np.min(candidates[:, 1]))
        candidate_max_x = float(np.max(candidates[:, 0]))
        candidate_max_y = float(np.max(candidates[:, 1]))
        min_x = (
            min(cfg.origin_x, candidate_min_x - padding)
            if candidate_min_x < cfg.origin_x else cfg.origin_x
        )
        min_y = (
            min(cfg.origin_y, candidate_min_y - padding)
            if candidate_min_y < cfg.origin_y else cfg.origin_y
        )
        max_x = (
            max(current_max_x, candidate_max_x + padding + resolution)
            if candidate_max_x >= current_max_x else current_max_x
        )
        max_y = (
            max(current_max_y, candidate_max_y + padding + resolution)
            if candidate_max_y >= current_max_y else current_max_y
        )
        new_origin_x = np.floor(min_x / resolution) * resolution
        new_origin_y = np.floor(min_y / resolution) * resolution
        new_width = int(np.ceil((max_x - new_origin_x) / resolution))
        new_height = int(np.ceil((max_y - new_origin_y) / resolution))
        if (
            new_width == self.width
            and new_height == self.height
            and np.isclose(new_origin_x, cfg.origin_x)
            and np.isclose(new_origin_y, cfg.origin_y)
        ):
            return
        if new_width <= 0 or new_height <= 0:
            return
        if new_width * new_height > cfg.max_live_grid_cells:
            # 容量上限只阻止扩容；当前栅格仍继续工作，落在现有范围内的点照常累积。
            return
        x_offset = int(round((cfg.origin_x - new_origin_x) / resolution))
        y_offset = int(round((cfg.origin_y - new_origin_y) / resolution))
        old_slice = (
            slice(y_offset, y_offset + self.height),
            slice(x_offset, x_offset + self.width),
        )
        new_occ = np.zeros((new_height, new_width), dtype=np.uint32)
        new_free = np.zeros((new_height, new_width), dtype=np.uint32)
        new_occ[old_slice] = self.occ_counts
        new_free[old_slice] = self.free_counts
        self.occ_counts = new_occ
        self.free_counts = new_free
        if self.base_cells is not None:
            new_base = np.full((new_height, new_width), -1, dtype=np.int8)
            new_base[old_slice] = self.base_cells
            self.base_cells = new_base
        self.width = new_width
        self.height = new_height
        cfg.origin_x = float(new_origin_x)
        cfg.origin_y = float(new_origin_y)
        cfg.size_x = new_width * resolution
        cfg.size_y = new_height * resolution

    def set_map_from_cloud_transform(self, matrix: np.ndarray) -> None:
        with self.lock:
            if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
                raise ValueError("map-from-cloud transform must be a finite 4x4 matrix")
            if self.state not in {"mapping", "paused"}:
                return
            self.map_from_cloud = matrix.copy()

    def _seed_from_parent_grid(self) -> None:
        record_path = self.config.map_store_root / f"{self.base_map_id}.json"
        try:
            record = json.loads(record_path.read_text(encoding="utf-8"))
            grid = record["grid"]
            width = int(grid["width"])
            height = int(grid["height"])
            resolution = float(grid["resolution"])
            origin_x = float(grid.get("origin_x", 0.0))
            origin_y = float(grid.get("origin_y", 0.0))
            raw_data = grid["data"]
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            raise RuntimeError(
                f"failed to load parent 2D grid {self.base_map_id}: {exc}"
            ) from exc
        if (
            width <= 0 or height <= 0 or not np.isfinite(resolution) or resolution <= 0
            or not np.isfinite(origin_x) or not np.isfinite(origin_y)
        ):
            raise RuntimeError("parent 2D grid dimensions are invalid")
        cell_count = width * height
        if cell_count > self.config.max_live_grid_cells:
            raise RuntimeError(
                f"parent 2D grid is too large: {cell_count} cells"
            )
        if not isinstance(raw_data, list) or len(raw_data) != cell_count:
            raise RuntimeError("parent 2D grid data length does not match dimensions")
        try:
            data = np.asarray(raw_data, dtype=np.int16).reshape(height, width)
        except (TypeError, ValueError, OverflowError) as exc:
            raise RuntimeError(f"parent 2D grid data is invalid: {exc}") from exc
        if np.any(data < -1) or np.any(data > 100):
            raise RuntimeError("parent 2D grid contains invalid occupancy values")
        self.width = width
        self.height = height
        self.config.resolution = resolution
        self.config.origin_x = origin_x
        self.config.origin_y = origin_y
        self.config.size_x = width * resolution
        self.config.size_y = height * resolution
        self.occ_counts = np.zeros((height, width), dtype=np.uint32)
        self.free_counts = np.zeros((height, width), dtype=np.uint32)
        self.base_cells = data.astype(np.int8, copy=True)

    def save_with_cloud_snapshot(
        self,
        flush_cloud: Callable[[], None],
        map_name: str | None = None,
    ) -> dict[str, Any]:
        """Serialize a complete save request and reuse a completed result.

        The RC gateway may legitimately retry after its ROS timeout while the
        first callback is still finishing. The retry waits for that callback;
        if the three artifacts are already complete it returns them directly
        instead of triggering another Point-LIO save and pcd2pgm conversion.
        Failed conversions remain retryable.
        """
        with self._save_operation_lock:
            cached = self._completed_save_result()
            if cached is not None:
                return cached
            self._begin_save(map_name)
            try:
                flush_cloud()
                return self._finish_save()
            except Exception as exc:
                self._mark_save_failed(exc)
                raise

    def _completed_save_result(self) -> dict[str, Any] | None:
        with self.lock:
            if self.state != "saved":
                return None
            artifact_paths = (
                self.last_map_pgm_path,
                self.last_map_yaml_path,
                self.last_relocalization_pcd_path,
            )
            if any(not value or not Path(value).is_file() for value in artifact_paths):
                return None
            result = self.status()
            result["success"] = True
            return result

    def save(self, map_name: str | None = None) -> dict[str, Any]:
        # 锁只护住状态翻转；pcd2pgm / PCD 合并子进程可达分钟级，若持锁执行，
        # /mapping/status 定时器、live map 发布和 pause/cancel 服务会整段阻塞
        # （多线程 executor 的工作线程会被状态查询逐个耗尽）。进入 saving 后
        # ingest_xyz 会跳过累积、map_from_cloud 不再更新，转换期间无并发写。
        with self._save_operation_lock:
            cached = self._completed_save_result()
            if cached is not None:
                return cached
            self._begin_save(map_name)
            try:
                return self._finish_save()
            except Exception as exc:
                self._mark_save_failed(exc)
                raise

    def _begin_save(self, map_name: str | None) -> None:
        """Freeze the active session before Point-LIO flush/conversion starts."""
        with self.lock:
            # 保存时允许重新指定地图名，但仍必须满足安全文件名规则。
            if map_name is not None:
                self.current_map_name = sanitize_map_name(map_name)
            if not self.current_map_name:
                raise RuntimeError("map_name is required before saving")
            if self.state == "saving":
                raise RuntimeError("mapping save is already in progress")
            if self.state not in {"mapping", "paused", "error"}:
                raise RuntimeError("no active mapping task to save")
            # 必须在请求 Point-LIO 落盘前进入 saving。否则 flush 等待期间另一条
            # start/cancel/pause 请求会替换数组、几何或 session_id，最终把一张
            # 地图的 PCD 与另一张地图的二维栅格组合到同一个 bundle。
            self.state = "saving"
            self.last_error = ""
            self._save_deadline = time.monotonic() + self.config.save_timeout_sec

    def _finish_save(self) -> dict[str, Any]:
        self.config.output_dir.mkdir(parents=True, exist_ok=True)
        paths = self._run_pcd2pgm_conversion()

        with self.lock:
            self.last_map_pgm_path = str(paths["map_pgm_path"])
            self.last_map_yaml_path = str(paths["map_yaml_path"])
            self.last_relocalization_pcd_path = str(paths["relocalization_pcd_path"])
            self.state = "saved"
            self.last_error = ""
            self._save_deadline = None
            result = self.status()
            result["success"] = True
            return result

    def _mark_save_failed(self, exc: Exception) -> None:
        with self.lock:
            if self.state == "saving":
                self.state = "error"
                self.last_error = str(exc)
            self._save_deadline = None

    def status(self) -> dict[str, Any]:
        # 返回给 App 的状态字段保持简单稳定，便于前端直接展示。
        with self.lock:
            cells_known = self._known_cell_count()
            return {
                "state": self.state,
                "current_map_name": self.current_map_name,
                "map_pgm_path": self.last_map_pgm_path or (
                    str(self._pgm_path()) if self.current_map_name else ""
                ),
                "map_yaml_path": self.last_map_yaml_path or (
                    str(self._yaml_path()) if self.current_map_name else ""
                ),
                "relocalization_pcd_path": self.last_relocalization_pcd_path,
                "cells_known": cells_known,
                "resolution": self.config.resolution,
                "width": self.width,
                "height": self.height,
                "last_error": self.last_error,
                "session_id": self.session_id,
                "base_map_id": self.base_map_id,
            }

    def _restore_default_geometry(self) -> None:
        resolution, size_x, size_y, origin_x, origin_y = self._default_geometry
        width = int(round(size_x / resolution))
        height = int(round(size_y / resolution))
        if width <= 0 or height <= 0 or width * height > self.config.max_live_grid_cells:
            raise ValueError("default mapping grid exceeds configured capacity")
        self.config.resolution = resolution
        self.config.size_x = size_x
        self.config.size_y = size_y
        self.config.origin_x = origin_x
        self.config.origin_y = origin_y
        self.width = width
        self.height = height
        self.occ_counts = np.zeros((height, width), dtype=np.uint32)
        self.free_counts = np.zeros((height, width), dtype=np.uint32)
        self.base_cells = None

    def _clear_session_metadata(self) -> None:
        self.current_map_name = ""
        self.session_id = ""
        self.base_map_id = ""
        self.last_error = ""
        self.last_map_pgm_path = ""
        self.last_map_yaml_path = ""
        self.last_relocalization_pcd_path = ""
        self.map_from_cloud = np.eye(4, dtype=np.float64)

    def _pgm_path(self) -> Path:
        return self.config.output_dir / f"{self.current_map_name}.pgm"

    def _yaml_path(self) -> Path:
        return self.config.output_dir / f"{self.current_map_name}.yaml"

    def _write_frozen_grid_map(self, pgm_path: Path, yaml_path: Path) -> None:
        """Write the same strict occupancy grid published to the App."""
        with self.lock:
            grid = self._build_grid(preview=False)
            width = self.width
            height = self.height
            resolution = self.config.resolution
            origin_x = self.config.origin_x
            origin_y = self.config.origin_y

        pgm_partial = pgm_path.with_suffix(pgm_path.suffix + ".partial")
        yaml_partial = yaml_path.with_suffix(yaml_path.suffix + ".partial")
        pgm_partial.write_bytes(
            f"P5\n{width} {height}\n255\n".encode("ascii")
            + np.flipud(grid).tobytes()
        )
        yaml_partial.write_text(
            f'image: {json.dumps(pgm_path.name)}\n'
            "mode: trinary\n"
            f"resolution: {resolution:.12g}\n"
            f"origin: [{origin_x:.12g}, {origin_y:.12g}, 0.0]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.196\n",
            encoding="utf-8",
        )
        os.replace(pgm_partial, pgm_path)
        os.replace(yaml_partial, yaml_path)

    def _filter_ghost_points(self, pcd_path: Path, output_dir: Path) -> Path:
        """Remove stale raised points only where current observations prove free."""
        try:
            header, rows, xyz = read_binary_pcd(pcd_path)
            with self.lock:
                drop = transient_ghost_drop_mask(
                    xyz,
                    free_since_occ=self.free_counts,
                    occ_counts=self.occ_counts,
                    resolution=self.config.resolution,
                    origin_x=self.config.origin_x,
                    origin_y=self.config.origin_y,
                    z_keep_below=self.config.free_z_max,
                    min_free_frames=self.config.dynamic_clear_min_observations,
                )
            if not np.any(drop):
                return pcd_path
            filtered_path = output_dir / "dynamic_filtered.pcd"
            write_binary_pcd(filtered_path, header, rows[~drop])
            return filtered_path
        except (OSError, UnicodeDecodeError, ValueError):
            # Unsupported/corrupt PCD must still reach the existing converter,
            # which owns the authoritative validation and error message.
            return pcd_path

    def _run_pcd2pgm_conversion(self) -> dict[str, Path]:
        cfg = self.config
        executable = self._resolve_path(cfg.pcd2pgm_executable)
        config_path = self._resolve_path(cfg.pcd2pgm_config)
        pcd_input = self._resolve_path(cfg.pcd_input_path)
        output_root = self._resolve_path(cfg.output_dir)
        session_name = self.session_id or self.current_map_name
        session_name = sanitize_map_name(session_name)
        output_dir = output_root / ".staging" / session_name
        relocalization_pcd = output_dir / "map.pcd"
        # 同一个 session/name 可能在异常退出后留下完整的旧三件套。转换前必须
        # 清空暂存目录，否则本次转换器未产出文件时会把旧文件误报为保存成功。
        if output_dir.exists():
            shutil.rmtree(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)

        if self.base_map_id:
            base_pcd = cfg.map_store_root / self.base_map_id / "map.pcd"
            if not base_pcd.is_file():
                raise FileNotFoundError(f"parent PCD not found: {base_pcd}")
            merge_executable = self._resolve_path(cfg.pcd_merge_executable)
            if not merge_executable.is_file():
                raise FileNotFoundError(
                    f"PCD merge executable not found: {merge_executable}")
            merged_pcd = output_dir / "merged.pcd"
            transform_csv = ",".join(
                f"{value:.12g}" for value in self.map_from_cloud.reshape(-1)
            )
            self.command_runner(
                [
                    str(merge_executable),
                    "--base", str(base_pcd),
                    "--increment", str(pcd_input),
                    "--output", str(merged_pcd),
                    "--transform", transform_csv,
                    "--leaf", "0.10",
                ],
                output_dir,
            )
            pcd_input = merged_pcd

        if not pcd_input.is_file():
            raise FileNotFoundError(f"Point-LIO PCD not found: {pcd_input}")
        if pcd_input.stat().st_size <= 0:
            raise RuntimeError(f"Point-LIO PCD is empty: {pcd_input}")
        pcd_input = self._filter_ghost_points(pcd_input, output_dir)
        if not config_path.is_file():
            raise FileNotFoundError(f"pcd2pgm config not found: {config_path}")
        if config_path.stat().st_size <= 0:
            raise RuntimeError(f"pcd2pgm config is empty: {config_path}")
        if not executable.is_file():
            raise FileNotFoundError(f"pcd2pgm executable not found: {executable}")

        cmd = [
            str(executable),
            "--headless",
            str(pcd_input),
            "--config",
            str(config_path),
            "--save-point-cloud",
            str(relocalization_pcd),
        ]
        self.command_runner(cmd, output_dir)

        output_stem = self._pcd2pgm_output_stem(config_path)
        pgm_path = output_dir / f"{output_stem}.pgm"
        yaml_path = output_dir / f"{output_stem}.yaml"
        self._write_frozen_grid_map(pgm_path, yaml_path)
        missing = [
            str(path)
            for path in (pgm_path, yaml_path, relocalization_pcd)
            if not path.is_file()
        ]
        if missing:
            raise RuntimeError("pcd2pgm did not create expected output: " + ", ".join(missing))
        empty = [
            str(path)
            for path in (pgm_path, yaml_path, relocalization_pcd)
            if path.stat().st_size <= 0
        ]
        if empty:
            raise RuntimeError(
                "pcd2pgm created empty output: " + ", ".join(empty)
            )
        return {
            "map_pgm_path": pgm_path,
            "map_yaml_path": yaml_path,
            "relocalization_pcd_path": relocalization_pcd,
        }

    def _pcd2pgm_output_stem(self, config_path: Path) -> str:
        for raw_line in config_path.read_text(encoding="utf-8").splitlines():
            line = raw_line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            if key.strip() == "output_stem":
                stem = value.strip()
                if stem:
                    return sanitize_map_name(stem)
        return self.current_map_name

    def _resolve_path(self, path: Path) -> Path:
        return path if path.is_absolute() else (Path.cwd() / path)

    def _run_command(self, cmd: list[str], cwd: Path) -> None:
        deadline = self._save_deadline
        timeout = self.config.save_timeout_sec
        if deadline is not None:
            timeout = deadline - time.monotonic()
        if timeout <= 0:
            raise RuntimeError("mapping save exceeded its external command budget")
        try:
            subprocess.run(
                cmd,
                cwd=cwd,
                check=True,
                text=True,
                capture_output=True,
                timeout=timeout,
            )
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError(
                f"mapping save command timed out after {timeout:.1f}s: {cmd[0]}"
            ) from exc
        except subprocess.CalledProcessError as exc:
            details = (exc.stderr or exc.stdout or str(exc)).strip()
            raise RuntimeError(f"pcd2pgm conversion failed: {details}") from exc

    def _build_grid(self, *, preview: bool = False) -> np.ndarray:
        # Nav2/ROS PGM 约定：0 表示占用，254 表示空闲，205 通常作为未知灰度。
        occupied_min = (
            self.config.preview_occupied_min_observations
            if preview else self.config.occupied_min_observations
        )
        free_min = (
            self.config.preview_free_min_observations
            if preview else self.config.free_min_observations
        )
        occ = self.occ_counts >= max(occupied_min, 1)
        free = self.free_counts >= max(free_min, 1)
        occ = clean_occupied_mask(
            occ,
            closing_iterations=self.config.occupied_closing_iterations,
            min_neighbors=self.config.occupied_despeckle_neighbors,
        )
        if self.base_cells is None:
            grid = np.full((self.height, self.width), 205, dtype=np.uint8)
        else:
            grid = np.full(self.base_cells.shape, 205, dtype=np.uint8)
            grid[(self.base_cells >= 0) & (self.base_cells < 65)] = 254
            grid[self.base_cells >= 65] = 0
        grid[free & (grid != 0)] = 254
        # 对障碍物做曼哈顿邻域膨胀，给导航避障留下安全边界。
        for _ in range(max(self.config.occupied_dilation, 0)):
            occ = _binary_dilate(occ)
        grid[occ] = 0
        return grid

    def occupancy_grid_snapshot(
        self,
        *,
        preview: bool = False,
        active_only: bool = False,
    ) -> dict[str, Any]:
        """Atomically snapshot metadata and cells for one OccupancyGrid frame."""
        with self.lock:
            if active_only and self.state not in {"mapping", "paused", "saving"}:
                # live-map timer 在 idle 时仍会周期触发；不要为一个不会发布的帧
                # 执行整图形态学清理和数组复制。
                return {"state": self.state}
            grid = self._build_grid(preview=preview)
            cells = np.full(grid.shape, -1, dtype=np.int8)
            cells[grid == 254] = 0
            cells[grid == 0] = 100
            return {
                "state": self.state,
                "resolution": float(self.config.resolution),
                "width": self.width,
                "height": self.height,
                "origin_x": float(self.config.origin_x),
                "origin_y": float(self.config.origin_y),
                "cells": cells,
            }

    def _known_cell_count(self) -> int:
        # 状态/Action feedback 只保留诊断用的已知栅格数，不把固定画布占用率
        # 误报为建图完成度。这里不重复执行耗时的形态学清理。
        # 最终栅格仍在 occupancy_grid_snapshot 中按严格规则构建。
        known = (
            self.occ_counts >= max(self.config.occupied_min_observations, 1)
        ) | (
            self.free_counts >= max(self.config.free_min_observations, 1)
        )
        if self.base_cells is not None:
            known |= self.base_cells >= 0
        return int(np.count_nonzero(known))

def run_ros(args: argparse.Namespace, session: MappingSession) -> None:
    # ROS 相关依赖只在实际启动 ROS 模式时导入，便于单元测试直接加载纯逻辑。
    import rclpy
    from bxi_nav_interfaces.action import BuildMap
    from bxi_nav_interfaces.msg import MappingStatus, NavPose
    from bxi_nav_interfaces.srv import ClearTerrain, SaveMap
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Float32
    from std_srvs.srv import SetBool, Trigger
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException, TransformListener

    class MappingControlNode(Node):
        """订阅 Point-LIO 点云，并提供 App 建图 action/service/topic。"""

        def __init__(self) -> None:
            super().__init__("mapping_control_node")
            self.callback_group = ReentrantCallbackGroup()
            self.goal_active = False
            self.active_goal_handle = None
            self.completed_build_result: dict[str, Any] | None = None
            self.build_done = threading.Event()
            self.build_lock = threading.RLock()
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.last_tf_warning_at = 0.0
            self.last_pose_tf_warning_at = 0.0
            cloud_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=5,
                durability=QoSDurabilityPolicy.VOLATILE,
            )
            status_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.status_pub = self.create_publisher(MappingStatus, "/mapping/status", status_qos)
            # 建图模式不启动整套 bxi_nav/Nav2，但 App 的轻量 /scan 仍需要机器人
            # 位姿才能投影到地图坐标。直接把 Point-LIO 里程计转换成与导航模式
            # 相同的 /nav/pose 契约，避免为了一个位姿额外拉起整套导航管线。
            self.pose_pub = self.create_publisher(NavPose, "/nav/pose", status_qos)
            # terrain_analysis maintains a separate rolling collision cloud.
            # Keep App terrain clears aligned with the PID planner's cloud.
            self.terrain_clear_pub = self.create_publisher(
                Float32, "/map_clearing", 10
            )
            # 实时建图栅格 → /map（App 建图画面靠它，接口契约 §6.1/§8 要求
            # reliable + transient_local，volatile 会被网关的订阅静默不兼容）。
            # 仅在建图会话活跃时发布；状态和地图使用独立定时器，避免 Action
            # feedback 重复构图，也让实时预览能稳定达到 2Hz。会话结束即销毁
            # 发布器——把 latched 帧从 DDS 撤掉，避免与 nav2 map_server 的静态
            # 先验图在 /map 上双 latch 打架。
            self.map_qos = status_qos
            self.map_pub = None
            self.create_subscription(PointCloud2, args.cloud_topic, self.on_cloud, cloud_qos)
            self.create_subscription(Odometry, args.odometry_topic, self.on_odometry, cloud_qos)
            self.build_server = ActionServer(self, BuildMap, "/mapping/build",
                execute_callback=self.execute_build,
                goal_callback=self.on_build_goal,
                cancel_callback=self.on_build_cancel,
                callback_group=self.callback_group,
            )
            self.create_service(SetBool, "/mapping/pause", self.on_pause, callback_group=self.callback_group)
            self.create_service(SaveMap, "/mapping/save", self.on_save, callback_group=self.callback_group)
            self.create_service(ClearTerrain, "/mapping/clear_terrain", self.on_clear_terrain,
                callback_group=self.callback_group)
            self.point_lio_start_client = self.create_client(Trigger, "/point_lio/mapping/start",
                callback_group=self.callback_group)
            self.point_lio_pause_client = self.create_client(SetBool, "/point_lio/mapping/pause",
                callback_group=self.callback_group)
            self.point_lio_save_client = self.create_client(Trigger, "/point_lio/mapping/save",
                callback_group=self.callback_group)
            self.point_lio_cancel_client = self.create_client(Trigger, "/point_lio/mapping/cancel",
                callback_group=self.callback_group)
            self.create_timer(args.status_period, self.publish_status, callback_group=self.callback_group)
            self.create_timer(args.map_period, self.publish_live_map, callback_group=self.callback_group)
            self.get_logger().info(
                "mapping control ready: "
                f"build_action=/mapping/build, pause_service=/mapping/pause, "
                f"save_service=/mapping/save, clear_service=/mapping/clear_terrain, "
                f"status_topic=/mapping/status, "
                f"cloud_topic={args.cloud_topic}"
            )
            self.publish_status()
            self.publish_live_map()

        def on_odometry(self, msg: Odometry) -> None:
            base_map_id = session.active_pose_base_map_id()
            if base_map_id is None:
                return

            # Point-LIO odometry is odom -> body_raw.  body_raw follows the
            # upside-down LiDAR/IMU mounting and is not the same physical pose
            # as the REP-103 base_link used by Nav2 and by the App marker.
            # Always resolve the configured robot frame through TF so new-map
            # and extend-map sessions publish the same robot definition.
            target_frame = (
                "map" if base_map_id else (msg.header.frame_id or "odom")
            )
            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame, args.robot_frame, Time())
                position = transform.transform.translation
                orientation = transform.transform.rotation
            except TransformException as exc:
                now = time.monotonic()
                if now - self.last_pose_tf_warning_at > 5.0:
                    self.get_logger().warning(
                        f"waiting for {target_frame}<-{args.robot_frame} "
                        f"pose: {exc}"
                    )
                    self.last_pose_tf_warning_at = now
                return

            try:
                yaw = quaternion_to_yaw((
                    orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w,
                ))
            except ValueError as exc:
                self.get_logger().warning(f"skip invalid mapping pose: {exc}")
                return

            pose = NavPose()
            pose.header.stamp = msg.header.stamp
            pose.header.frame_id = "map"
            pose.x = float(position.x)
            pose.y = float(position.y)
            pose.yaw = yaw
            pose.twist = msg.twist.twist
            self.pose_pub.publish(pose)

        def on_cloud(self, msg: PointCloud2) -> None:
            base_map_id = session.active_cloud_base_map_id()
            if base_map_id is None:
                return
            # 将 ROS PointCloud2 转成 Nx3 numpy 数组，再交给纯 Python 状态逻辑处理。
            pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"))
            if base_map_id:
                source_frame = msg.header.frame_id or "odom"
                try:
                    transform = self.tf_buffer.lookup_transform(
                        "map", source_frame, Time())
                    t = transform.transform.translation
                    q = transform.transform.rotation
                    pts, matrix = apply_rigid_transform(
                        pts,
                        translation=(t.x, t.y, t.z),
                        quaternion=(q.x, q.y, q.z, q.w),
                    )
                    session.set_map_from_cloud_transform(matrix)
                except (TransformException, ValueError) as exc:
                    now = time.monotonic()
                    if now - self.last_tf_warning_at > 5.0:
                        self.get_logger().warning(
                            f"waiting for map<-{source_frame} transform: {exc}")
                        self.last_tf_warning_at = now
                    return
            session.ingest_xyz(pts)

        def on_build_goal(self, goal_request: BuildMap.Goal) -> GoalResponse:
            # 同一时间只允许一个建图会话，避免多个 App 客户端互相覆盖地图名和累计数据。
            try:
                sanitize_map_name(goal_request.map_name)
            except ValueError as exc:
                self.get_logger().error(f"reject mapping goal: {exc}")
                return GoalResponse.REJECT
            with self.build_lock:
                if self.goal_active:
                    self.get_logger().warn("reject mapping goal: another build goal is active")
                    return GoalResponse.REJECT
                self.goal_active = True
                return GoalResponse.ACCEPT

        def on_build_cancel(self, goal_handle) -> CancelResponse:
            # 取消 action 表示丢弃未保存数据；execute_build 会完成真正的清理和返回。
            return CancelResponse.ACCEPT

        def execute_build(self, goal_handle) -> BuildMap.Result:
            # build action 是一次建图会话：start 后持续 feedback，直到 save 成功或 action cancel。
            started_at = time.monotonic()
            with self.build_lock:
                self.active_goal_handle = goal_handle
                self.completed_build_result = None
                self.build_done.clear()

            try:
                self._call_point_lio_trigger(self.point_lio_start_client,
                    "start Point-LIO mapping accumulation")
                session.start(
                    goal_handle.request.map_name,
                    goal_handle.request.session_id,
                    goal_handle.request.base_map_id,
                )
            except Exception as exc:
                session.last_error = str(exc)
                self.get_logger().error(f"mapping build failed to start: {exc}")
                goal_handle.abort()
                self.publish_status()
                self._clear_active_goal()
                return self._build_action_result(False, str(exc), "", "", "")

            self.publish_status()
            while rclpy.ok():
                # 保存结果优先于取消：保存已经落盘的数据不应被随后到达的
                # 取消请求丢弃，App 会拿到保存结果而不是 "cancelled"。
                with self.build_lock:
                    completed = self.completed_build_result
                if completed is not None:
                    self.publish_status()
                    success = bool(completed.get("success", False))
                    message = str(completed.get("message", "")).strip()
                    if not message:
                        message = "map saved" if success else "mapping save failed"
                    (goal_handle.succeed if success else goal_handle.abort)()
                    result = self._build_action_result(
                        success,
                        message,
                        str(completed.get("map_pgm_path", "")),
                        str(completed.get("map_yaml_path", "")),
                        str(completed.get("relocalization_pcd_path", "")),
                    )
                    self._clear_active_goal()
                    return result

                if goal_handle.is_cancel_requested:
                    if session.status()["state"] == "saving":
                        # 保存进行中不能丢弃数据；等保存结束后由 completed
                        # 分支返回。取消异常绝不能逃出 execute 回调，否则
                        # goal_active 永久为 True，后续建图 goal 全部被拒。
                        self.get_logger().warn(
                            "cancel requested while save is in progress; deferring")
                        self.build_done.wait(args.status_period)
                        continue
                    try:
                        session.cancel()
                    except RuntimeError as exc:
                        # 状态检查后刚好进入 saving 的窄竞态：同样推迟。
                        self.get_logger().warn(f"mapping cancel deferred: {exc}")
                        self.build_done.wait(args.status_period)
                        continue
                    try:
                        self._call_point_lio_trigger(self.point_lio_cancel_client,
                            "cancel Point-LIO mapping accumulation")
                    except Exception as exc:
                        session.last_error = str(exc)
                        self.get_logger().error(f"Point-LIO mapping cancel failed: {exc}")
                    goal_handle.canceled()
                    self.publish_status()
                    result = self._build_action_result(
                        False, "mapping cancelled", "", "", "")
                    self._clear_active_goal()
                    return result

                goal_handle.publish_feedback(self._build_feedback(started_at))
                self.build_done.wait(args.status_period)

            self._clear_active_goal()
            return self._build_action_result(False, "rclpy shutdown", "", "", "")

        def on_pause(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
            # true 暂停累计，false 继续累计；Point-LIO 本体仍照常运行。
            try:
                current = session.status()
                if not current["current_map_name"] or current["state"] in {"idle", "cancelled"}:
                    raise RuntimeError("no active mapping task")
                self._call_point_lio_set_bool(self.point_lio_pause_client, request.data,
                    "pause Point-LIO mapping accumulation" if request.data
                    else "resume Point-LIO mapping accumulation",
                )
                session.pause(request.data)
                response.success = True
                response.message = "paused" if request.data else "mapping"
            except Exception as exc:
                session.last_error = str(exc)
                response.success = False
                response.message = str(exc)
                self.get_logger().error(f"mapping pause failed: {exc}")
            self.publish_status()
            self.publish_live_map()
            return response

        def on_save(self, request: SaveMap.Request, response: SaveMap.Response) -> SaveMap.Response:
            # 保存成功后唤醒 build action，让 action result 返回最终文件路径。
            try:
                map_name = request.map_name.strip() or None
                if map_name is not None:
                    sanitize_map_name(map_name)
                result = session.save_with_cloud_snapshot(
                    lambda: self._call_point_lio_trigger(
                        self.point_lio_save_client,
                        "save Point-LIO PCD map",
                        timeout_sec=45.0,
                    ),
                    map_name,
                )
                response.success = True
                response.message = "map saved"
                response.map_pgm_path = str(result["map_pgm_path"])
                response.map_yaml_path = str(result["map_yaml_path"])
                response.map_pcd_path = str(result["relocalization_pcd_path"])
                with self.build_lock:
                    self.completed_build_result = result
                    self.build_done.set()
            except Exception as exc:
                session.last_error = str(exc)
                response.success = False
                response.message = str(exc)
                response.map_pgm_path = ""
                response.map_yaml_path = ""
                response.map_pcd_path = ""
                self.get_logger().error(f"mapping save failed: {exc}")
                # A failed save must also finish the long-running BuildMap
                # action. Otherwise goal_active remains true forever and the
                # next mapping request is rejected as "build goal failed".
                with self.build_lock:
                    self.completed_build_result = {
                        "success": False,
                        "message": str(exc),
                        "map_pgm_path": "",
                        "map_yaml_path": "",
                        "relocalization_pcd_path": "",
                    }
                    self.build_done.set()
            self.publish_status()
            self.publish_live_map()
            return response

        def on_clear_terrain(
            self,
            request: ClearTerrain.Request,
            response: ClearTerrain.Response,
        ) -> ClearTerrain.Response:
            try:
                target_frame = "map" if session.base_map_id else "odom"
                transform = self.tf_buffer.lookup_transform(
                    target_frame, args.robot_frame, Time())
                position = transform.transform.translation
                cleared = session.clear_radius(position.x, position.y, float(request.radius))
                clear_msg = Float32()
                clear_msg.data = float(request.radius)
                self.terrain_clear_pub.publish(clear_msg)
                response.success = True
                response.message = (
                    f"cleared {cleared} cells within {float(request.radius):.2f} m"
                )
            except (TransformException, RuntimeError, ValueError) as exc:
                response.success = False
                response.message = str(exc)
                session.last_error = str(exc)
                self.get_logger().error(f"mapping terrain clear failed: {exc}")
            self.publish_status()
            return response

        def publish_status(self) -> None:
            data = session.status()
            status_msg = MappingStatus()
            status_msg.header.stamp = self.get_clock().now().to_msg()
            status_msg.header.frame_id = "map"
            status_msg.state = str(data["state"])
            status_msg.current_map_name = str(data["current_map_name"])
            status_msg.resolution = float(data["resolution"])
            status_msg.width = int(data["width"])
            status_msg.height = int(data["height"])
            status_msg.last_error = str(data["last_error"])
            status_msg.session_id = str(data["session_id"])
            status_msg.base_map_id = str(data["base_map_id"])
            self.status_pub.publish(status_msg)

        def publish_live_map(self) -> None:
            # 建图/暂停/保存中把累积栅格发出去；其余状态销毁发布器撤掉 latch。
            snapshot = session.occupancy_grid_snapshot(
                preview=False,
                active_only=True,
            )
            state = snapshot["state"]
            if state in ("mapping", "paused", "saving"):
                if self.map_pub is None:
                    self.map_pub = self.create_publisher(
                        OccupancyGrid, args.map_topic, self.map_qos)
                msg = OccupancyGrid()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "map"
                msg.info.resolution = snapshot["resolution"]
                msg.info.width = snapshot["width"]
                msg.info.height = snapshot["height"]
                msg.info.origin.position.x = snapshot["origin_x"]
                msg.info.origin.position.y = snapshot["origin_y"]
                msg.info.origin.orientation.w = 1.0
                # array('b') 直接吃 numpy int8 字节，绕过 36 万格的 Python 列表转换。
                msg.data = array.array(
                    "b", snapshot["cells"].tobytes()
                )
                self.map_pub.publish(msg)
            elif self.map_pub is not None:
                self.destroy_publisher(self.map_pub)
                self.map_pub = None

        def _call_point_lio_trigger(
            self,
            client,
            action_name: str,
            timeout_sec: float = 10.0,
        ) -> str:
            return self._call_point_lio_service(
                client,
                Trigger.Request(),
                action_name,
                timeout_sec=timeout_sec,
            )

        def _call_point_lio_set_bool(self, client, value: bool, action_name: str) -> str:
            request = SetBool.Request()
            request.data = value
            return self._call_point_lio_service(client, request, action_name)

        def _call_point_lio_service(
            self,
            client,
            request,
            action_name: str,
            timeout_sec: float = 10.0,
        ) -> str:
            deadline = time.monotonic() + timeout_sec
            while rclpy.ok() and not client.wait_for_service(timeout_sec=0.1):
                if time.monotonic() >= deadline:
                    raise RuntimeError(f"timeout waiting to {action_name}")

            future = client.call_async(request)
            while rclpy.ok() and not future.done():
                if time.monotonic() >= deadline:
                    raise RuntimeError(f"timeout trying to {action_name}")
                time.sleep(0.02)

            if not future.done():
                raise RuntimeError(f"ROS shutdown while trying to {action_name}")
            result = future.result()
            if result is None:
                raise RuntimeError(f"failed to {action_name}: empty response")
            if not result.success:
                message = result.message or "service returned success=false"
                raise RuntimeError(f"failed to {action_name}: {message}")
            return result.message

        def _build_feedback(self, started_at: float) -> BuildMap.Feedback:
            data = session.status()
            feedback = BuildMap.Feedback()
            feedback.state = str(data["state"])
            feedback.cells_known = int(data["cells_known"])
            feedback.elapsed_sec = float(time.monotonic() - started_at)
            feedback.resolution = float(data["resolution"])
            feedback.width = int(data["width"])
            feedback.height = int(data["height"])
            return feedback

        def _build_action_result(
            self,
            success: bool,
            message: str,
            pgm_path: str,
            yaml_path: str,
            pcd_path: str,
        ) -> BuildMap.Result:
            result = BuildMap.Result()
            result.success = success
            result.message = message
            result.map_pgm_path = pgm_path
            result.map_yaml_path = yaml_path
            result.map_pcd_path = pcd_path
            return result

        def _clear_active_goal(self) -> None:
            with self.build_lock:
                self.goal_active = False
                self.active_goal_handle = None
                self.completed_build_result = None
                self.build_done.clear()

    rclpy.init()
    node = MappingControlNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        best_effort_ros_cleanup(executor, node, rclpy)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    # 这些参数也会由 launch 文件透传，便于现场按地图范围和高度带调参。
    root = indoor_slam_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cloud-topic", default="/cloud_registered")
    parser.add_argument("--odometry-topic", default="/aft_mapped_to_init")
    parser.add_argument(
        "--robot-frame",
        default="base_link",
        help="REP-103 robot frame used for App pose and terrain clear center",
    )
    parser.add_argument("--map-topic", default="/map",
                        help="建图期间实时占用栅格的发布话题 (latched, 1Hz)")
    parser.add_argument("--output-dir", default=root / "src" / "bxi_nav" / "maps", type=Path)
    parser.add_argument("--resolution", default=0.05, type=float)
    parser.add_argument("--size-x", default=60.0, type=float)
    parser.add_argument("--size-y", default=60.0, type=float)
    parser.add_argument("--origin-x", default=-30.0, type=float)
    parser.add_argument("--origin-y", default=-30.0, type=float)
    parser.add_argument("--occupied-z-min", default=-0.8, type=float)
    parser.add_argument("--occupied-z-max", default=0.3, type=float)
    parser.add_argument("--free-z-min", default=-1.30, type=float)
    parser.add_argument("--free-z-max", default=-0.35, type=float)
    parser.add_argument("--occupied-dilation", default=0, type=int)
    parser.add_argument("--occupied-min-observations", default=3, type=int)
    parser.add_argument("--free-min-observations", default=2, type=int)
    parser.add_argument("--dynamic-clear-min-observations", default=10, type=int)
    parser.add_argument("--preview-occupied-min-observations", default=1, type=int)
    parser.add_argument("--preview-free-min-observations", default=1, type=int)
    parser.add_argument("--occupied-closing-iterations", default=1, type=int)
    parser.add_argument("--occupied-despeckle-neighbors", default=1, type=int)
    parser.add_argument("--grid-expansion-padding", default=5.0, type=float)
    parser.add_argument("--max-grid-expansion-per-frame", default=40.0, type=float)
    parser.add_argument("--max-live-grid-cells", default=4_000_000, type=int)
    parser.add_argument("--status-period", default=1.0, type=float)
    parser.add_argument("--map-period", default=0.5, type=float)
    parser.add_argument("--pcd2pgm-executable", default=root / "pcd2pgm_headless", type=Path)
    parser.add_argument("--pcd2pgm-config", default=root / "scans_nav2_map.cfg", type=Path)
    parser.add_argument(
        "--pcd-input-path",
        default=root / "src" / "Point-LIO" / "PCD" / "scans.pcd",
        type=Path,
    )
    parser.add_argument("--map-store-root", default="/var/lib/bxi/maps", type=Path)
    parser.add_argument("--pcd-merge-executable", default=root / "merge_pcd_maps", type=Path)
    parser.add_argument("--save-timeout-sec", type=float, default=105.0)
    # ROS2 launch 会自动追加 --ros-args/-r 等参数；这里忽略未知参数，留给 rclpy 处理。
    args, _ = parser.parse_known_args(argv)
    return args


def main() -> None:
    args = parse_args()
    # 同一个 MappingSession 同时被 ROS 点云回调线程和 HTTP 请求线程共享。
    session = MappingSession(
        MappingConfig(
            output_dir=args.output_dir,
            resolution=args.resolution,
            size_x=args.size_x,
            size_y=args.size_y,
            origin_x=args.origin_x,
            origin_y=args.origin_y,
            occupied_z_min=args.occupied_z_min,
            occupied_z_max=args.occupied_z_max,
            free_z_min=args.free_z_min,
            free_z_max=args.free_z_max,
            occupied_dilation=args.occupied_dilation,
            occupied_min_observations=args.occupied_min_observations,
            free_min_observations=args.free_min_observations,
            dynamic_clear_min_observations=args.dynamic_clear_min_observations,
            preview_occupied_min_observations=args.preview_occupied_min_observations,
            preview_free_min_observations=args.preview_free_min_observations,
            occupied_closing_iterations=args.occupied_closing_iterations,
            occupied_despeckle_neighbors=args.occupied_despeckle_neighbors,
            grid_expansion_padding=args.grid_expansion_padding,
            max_grid_expansion_per_frame=args.max_grid_expansion_per_frame,
            max_live_grid_cells=args.max_live_grid_cells,
            pcd2pgm_executable=args.pcd2pgm_executable,
            pcd2pgm_config=args.pcd2pgm_config,
            pcd_input_path=args.pcd_input_path,
            map_store_root=args.map_store_root,
            pcd_merge_executable=args.pcd_merge_executable,
            save_timeout_sec=args.save_timeout_sec,
        )
    )
    run_ros(args, session)


if __name__ == "__main__":
    main()
