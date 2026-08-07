"""残影过滤的行为测试: 人走后的悬空点被删, 墙/地面/周期性稀疏命中物保留。

运行: PYTHONPATH=src/Point-LIO/scripts python -m pytest src/Point-LIO/test -q
"""

import struct
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from mapping_control_node import (  # noqa: E402
    MappingConfig,
    MappingSession,
    read_binary_pcd,
    transient_ghost_drop_mask,
    write_binary_pcd,
)


def write_test_pcd(path: Path, xyz: np.ndarray) -> None:
    """按 PCL PointXYZINormal 的二进制布局写一个最小 PCD。"""
    n = len(xyz)
    fields = "x y z normal_x normal_y normal_z intensity curvature"
    header = "\n".join([
        "# .PCD v0.7 - Point Cloud Data file format",
        "VERSION 0.7",
        f"FIELDS {fields}",
        "SIZE 4 4 4 4 4 4 4 4",
        "TYPE F F F F F F F F",
        "COUNT 1 1 1 1 1 1 1 1",
        f"WIDTH {n}",
        "HEIGHT 1",
        "VIEWPOINT 0 0 0 1 0 0 0",
        f"POINTS {n}",
        "DATA binary",
    ]) + "\n"
    with open(path, "wb") as handle:
        handle.write(header.encode("ascii"))
        for row in xyz:
            handle.write(struct.pack(
                "<8f", row[0], row[1], row[2], 0.0, 0.0, 1.0, 42.0, 0.0))


def test_pcd_roundtrip_preserves_rows(tmp_path):
    xyz = np.array([[1.0, 2.0, 3.0], [-4.5, 0.25, -1.0], [0.0, 0.0, 0.0]])
    source = tmp_path / "in.pcd"
    write_test_pcd(source, xyz)

    header, raw_rows, parsed = read_binary_pcd(source)
    assert parsed == pytest.approx(xyz)
    assert raw_rows.shape == (3, 32)

    out = tmp_path / "out.pcd"
    write_binary_pcd(out, header, raw_rows[[0, 2]])
    _, raw_again, xyz_again = read_binary_pcd(out)
    assert xyz_again == pytest.approx(xyz[[0, 2]])
    # 非 xyz 字段字节原样保留 (intensity=42 位于偏移 24)。
    assert struct.unpack("<f", raw_again[0, 24:28].tobytes())[0] == 42.0


def make_session() -> MappingSession:
    session = MappingSession(MappingConfig(
        output_dir=Path("unused"),
        occupied_closing_iterations=0,
        occupied_despeckle_neighbors=0,
        dynamic_clear_min_observations=10,
    ))
    session.start("test")
    return session


WALL = np.array([[2.0, 0.0, 0.0]])          # 占用带, 每帧可见
GHOST = np.array([[0.0, 2.0, 0.0]])         # 占用带, 前 10 帧后消失
GROUND_AT_GHOST = np.array([[0.0, 2.0, -1.0]])  # 人走后该格露出的地面
FLOOR = np.array([[0.0, 0.0, -1.0]])        # 空闲带, 每帧可见


def test_person_ghost_removed_wall_and_ground_kept(tmp_path):
    session = make_session()
    for _ in range(10):  # 有人站着
        session.ingest_xyz(np.vstack([WALL, GHOST, FLOOR]))
    for _ in range(40):  # 人离开, 激光打到那格地面
        session.ingest_xyz(np.vstack([WALL, GROUND_AT_GHOST, FLOOR]))

    ghost_ix = int((GHOST[0, 0] - session.config.origin_x) / session.config.resolution)
    ghost_iy = int((GHOST[0, 1] - session.config.origin_y) / session.config.resolution)
    assert session.occ_counts[ghost_iy, ghost_ix] == 0
    assert session.occupancy_grid_snapshot()["cells"][ghost_iy, ghost_ix] == 0

    source = tmp_path / "scans.pcd"
    write_test_pcd(source, np.vstack([WALL, GHOST, GROUND_AT_GHOST, FLOOR]))
    result = session._filter_ghost_points(source, tmp_path)

    assert result != source  # 确实产出了过滤文件
    _, _, xyz = read_binary_pcd(result)
    assert xyz == pytest.approx(np.vstack([WALL, GROUND_AT_GHOST, FLOOR]))


def test_sparse_periodic_hits_survive(tmp_path):
    # 桌腿: 稀疏点云每 5 帧才命中一次, 其间该格常有地面证据 —— 占用观测
    # 周期性清零计时器, 不允许当残影删掉。
    session = make_session()
    leg = np.array([[1.0, 1.0, 0.0]])
    ground_at_leg = np.array([[1.0, 1.0, -1.0]])
    for frame in range(60):
        cloud = [ground_at_leg]
        if frame % 5 == 0:
            cloud.append(leg)
        session.ingest_xyz(np.vstack(cloud))

    source = tmp_path / "scans.pcd"
    write_test_pcd(source, leg)
    result = session._filter_ghost_points(source, tmp_path)
    _, _, xyz = read_binary_pcd(result)
    assert len(xyz) == 1  # 桌腿保留


def test_mask_ignores_out_of_grid_and_ground_points():
    since = np.full((4, 4), 10, dtype=np.uint32)
    occ = np.zeros((4, 4), dtype=np.uint32)
    xyz = np.array([
        [0.05, 0.05, 0.0],    # 格内, 悬空 -> 删
        [0.05, 0.05, -1.0],   # 格内, 地面带以下 -> 留
        [99.0, 99.0, 0.0],    # 格外 -> 留
    ])
    drop = transient_ghost_drop_mask(
        xyz, free_since_occ=since, occ_counts=occ, resolution=0.1,
        origin_x=0.0, origin_y=0.0, z_keep_below=-0.35, min_free_frames=10)
    assert drop.tolist() == [True, False, False]


def test_corrupt_pcd_passes_through(tmp_path):
    session = make_session()
    for _ in range(10):
        session.ingest_xyz(np.vstack([GHOST]))
    for _ in range(40):
        session.ingest_xyz(np.vstack([GROUND_AT_GHOST]))
    bad = tmp_path / "bad.pcd"
    bad.write_bytes(b"not a pcd at all")
    assert session._filter_ghost_points(bad, tmp_path) == bad
