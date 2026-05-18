#!/usr/bin/env python3
"""Convert a binary PCD map into a ROS/Nav2 occupancy grid map."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np


def read_binary_pcd_xyz(path: Path) -> tuple[np.ndarray, list[str]]:
    header: list[str] = []
    with path.open("rb") as f:
        while True:
            line = f.readline()
            if not line:
                raise ValueError("PCD header ended before DATA line")
            decoded = line.decode("ascii", errors="replace").strip()
            header.append(decoded)
            if decoded.startswith("DATA"):
                if decoded != "DATA binary":
                    raise ValueError(f"Only binary PCD is supported, got: {decoded}")
                data_offset = f.tell()
                break

    fields: list[str] = []
    sizes: list[int] = []
    types: list[str] = []
    counts: list[int] = []
    points = None
    for line in header:
        parts = line.split()
        if not parts:
            continue
        key = parts[0]
        if key == "FIELDS":
            fields = parts[1:]
        elif key == "SIZE":
            sizes = [int(v) for v in parts[1:]]
        elif key == "TYPE":
            types = parts[1:]
        elif key == "COUNT":
            counts = [int(v) for v in parts[1:]]
        elif key == "POINTS":
            points = int(parts[1])

    if points is None:
        raise ValueError("PCD header has no POINTS field")
    if not {"x", "y", "z"}.issubset(fields):
        raise ValueError(f"PCD must contain x/y/z fields, got: {fields}")

    formats = []
    for size, typ, count in zip(sizes, types, counts):
        if typ == "F" and size == 4:
            fmt = "<f4"
        elif typ == "F" and size == 8:
            fmt = "<f8"
        elif typ == "I" and size == 4:
            fmt = "<i4"
        elif typ == "U" and size == 4:
            fmt = "<u4"
        else:
            raise ValueError(f"Unsupported PCD field type: size={size}, type={typ}, count={count}")
        formats.append(fmt if count == 1 else (fmt, count))

    dtype = np.dtype({"names": fields, "formats": formats})
    points_array = np.fromfile(path, dtype=dtype, count=points, offset=data_offset)
    xyz = np.column_stack((points_array["x"], points_array["y"], points_array["z"])).astype(np.float32)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    return xyz, header


def points_to_cells(
    points: np.ndarray,
    origin_x: float,
    origin_y: float,
    resolution: float,
    width: int,
    height: int,
) -> tuple[np.ndarray, np.ndarray]:
    ix = np.floor((points[:, 0] - origin_x) / resolution).astype(np.int64)
    iy = np.floor((points[:, 1] - origin_y) / resolution).astype(np.int64)
    keep = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
    return ix[keep], iy[keep]


def write_pgm(path: Path, image: np.ndarray) -> None:
    with path.open("wb") as f:
        f.write(f"P5\n{image.shape[1]} {image.shape[0]}\n255\n".encode("ascii"))
        f.write(image.tobytes())


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pcd", required=True, type=Path, help="Input binary PCD file")
    parser.add_argument("--output-dir", default=Path("maps"), type=Path, help="Output directory")
    parser.add_argument("--name", default="indoor_map", help="Output map basename")
    parser.add_argument("--resolution", default=0.10, type=float, help="Map resolution in meters/cell")
    parser.add_argument("--padding", default=1.0, type=float, help="Padding around PCD bounds in meters")
    parser.add_argument("--min-x", default=None, type=float, help="Manual minimum x bound")
    parser.add_argument("--max-x", default=None, type=float, help="Manual maximum x bound")
    parser.add_argument("--min-y", default=None, type=float, help="Manual minimum y bound")
    parser.add_argument("--max-y", default=None, type=float, help="Manual maximum y bound")
    parser.add_argument("--occupied-z-min", default=-0.20, type=float)
    parser.add_argument("--occupied-z-max", default=1.80, type=float)
    parser.add_argument("--free-z-min", default=-1.30, type=float)
    parser.add_argument("--free-z-max", default=-0.35, type=float)
    parser.add_argument(
        "--min-occupied-points",
        default=1,
        type=int,
        help="Minimum obstacle points in a cell to mark it occupied",
    )
    parser.add_argument(
        "--min-free-points",
        default=1,
        type=int,
        help="Minimum floor points in a cell to mark it free",
    )
    parser.add_argument(
        "--occupied-dilation",
        default=1,
        type=int,
        help="Dilate occupied cells by this many grid cells for visibility/safety",
    )
    args = parser.parse_args()

    points, _ = read_binary_pcd_xyz(args.pcd)
    min_x = math.floor(args.min_x if args.min_x is not None else float(points[:, 0].min() - args.padding))
    min_y = math.floor(args.min_y if args.min_y is not None else float(points[:, 1].min() - args.padding))
    max_x = math.ceil(args.max_x if args.max_x is not None else float(points[:, 0].max() + args.padding))
    max_y = math.ceil(args.max_y if args.max_y is not None else float(points[:, 1].max() + args.padding))
    width = int(math.ceil((max_x - min_x) / args.resolution))
    height = int(math.ceil((max_y - min_y) / args.resolution))

    grid = np.full((height, width), 205, dtype=np.uint8)  # unknown

    free = points[(points[:, 2] >= args.free_z_min) & (points[:, 2] <= args.free_z_max)]
    if free.size:
        fx, fy = points_to_cells(free, min_x, min_y, args.resolution, width, height)
        free_counts = np.zeros((height, width), dtype=np.uint16)
        np.add.at(free_counts, (fy, fx), 1)
        grid[free_counts >= args.min_free_points] = 254

    occupied = points[
        (points[:, 2] >= args.occupied_z_min) & (points[:, 2] <= args.occupied_z_max)
    ]
    ox, oy = points_to_cells(occupied, min_x, min_y, args.resolution, width, height)
    occupied_counts = np.zeros((height, width), dtype=np.uint16)
    np.add.at(occupied_counts, (oy, ox), 1)
    occupied_mask = occupied_counts >= args.min_occupied_points

    for _ in range(max(args.occupied_dilation, 0)):
        expanded = occupied_mask.copy()
        expanded[1:, :] |= occupied_mask[:-1, :]
        expanded[:-1, :] |= occupied_mask[1:, :]
        expanded[:, 1:] |= occupied_mask[:, :-1]
        expanded[:, :-1] |= occupied_mask[:, 1:]
        occupied_mask = expanded
    grid[occupied_mask] = 0

    # PGM origin is top-left, ROS map origin is bottom-left.
    pgm_image = np.flipud(grid)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    pgm_path = args.output_dir / f"{args.name}.pgm"
    yaml_path = args.output_dir / f"{args.name}.yaml"
    write_pgm(pgm_path, pgm_image)
    yaml_path.write_text(
        "\n".join(
            [
                f"image: {pgm_path.name}",
                "mode: trinary",
                f"resolution: {args.resolution:.6f}",
                f"origin: [{min_x:.6f}, {min_y:.6f}, 0.000000]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.25",
                "",
            ]
        ),
        encoding="ascii",
    )

    occupied_cells = int(np.count_nonzero(grid == 0))
    free_cells = int(np.count_nonzero(grid == 254))
    unknown_cells = int(np.count_nonzero(grid == 205))
    print(f"wrote {pgm_path}")
    print(f"wrote {yaml_path}")
    print(f"size: {width} x {height}, resolution: {args.resolution} m/cell")
    print(f"origin: [{min_x}, {min_y}, 0]")
    print(f"cells: occupied={occupied_cells}, free={free_cells}, unknown={unknown_cells}")


if __name__ == "__main__":
    main()
