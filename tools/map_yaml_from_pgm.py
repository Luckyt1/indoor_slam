#!/usr/bin/env python3
"""Generate a ROS/Nav2 map YAML from a PGM image.

PGM files store image dimensions and pixels only. They do not store map
resolution or the world-frame origin, so the caller must provide an alignment
rule.
"""

from __future__ import annotations

import argparse
from pathlib import Path


WHITESPACE = b" \t\r\n"


def next_token(data: bytes, pos: int) -> tuple[bytes, int]:
    n = len(data)
    while pos < n:
        while pos < n and data[pos] in WHITESPACE:
            pos += 1
        if pos < n and data[pos] == ord("#"):
            while pos < n and data[pos] not in b"\r\n":
                pos += 1
            continue
        break

    start = pos
    while pos < n and data[pos] not in WHITESPACE and data[pos] != ord("#"):
        pos += 1
    if start == pos:
        raise ValueError("Invalid PGM header: missing token")
    return data[start:pos], pos


def read_pgm_size(path: Path) -> tuple[int, int]:
    data = path.read_bytes()
    pos = 0
    magic, pos = next_token(data, pos)
    if magic not in (b"P2", b"P5"):
        raise ValueError(f"{path}: expected P2 or P5 PGM, got {magic!r}")

    width_token, pos = next_token(data, pos)
    height_token, pos = next_token(data, pos)
    max_token, _ = next_token(data, pos)
    width = int(width_token)
    height = int(height_token)
    max_value = int(max_token)
    if width <= 0 or height <= 0:
        raise ValueError(f"{path}: invalid image size {width}x{height}")
    if max_value <= 0:
        raise ValueError(f"{path}: invalid max value {max_value}")
    return width, height


def parse_xy(values: list[float], name: str) -> tuple[float, float]:
    if len(values) != 2:
        raise ValueError(f"{name} needs exactly two values")
    return values[0], values[1]


def compute_origin(args: argparse.Namespace, width: int, height: int) -> tuple[float, float]:
    resolution = args.resolution
    if args.origin is not None:
        return parse_xy(args.origin, "--origin")

    if args.center_at is not None:
        center_x, center_y = parse_xy(args.center_at, "--center-at")
        return center_x - width * resolution * 0.5, center_y - height * resolution * 0.5

    if args.anchor_pixel is not None and args.anchor_world is not None:
        pixel_x, pixel_y = parse_xy(args.anchor_pixel, "--anchor-pixel")
        world_x, world_y = parse_xy(args.anchor_world, "--anchor-world")
        origin_x = world_x - (pixel_x + 0.5) * resolution
        origin_y = world_y - (height - pixel_y - 0.5) * resolution
        return origin_x, origin_y

    raise ValueError("Choose one alignment mode: --origin, --center-at, or --anchor-pixel with --anchor-world")


def write_yaml(args: argparse.Namespace, width: int, height: int, origin_x: float, origin_y: float) -> None:
    image_name = args.image if args.image is not None else args.pgm.name
    output = args.output if args.output is not None else args.pgm.with_suffix(".yaml")
    output.parent.mkdir(parents=True, exist_ok=True)

    lines = [
        f'image: "{image_name}"',
        f"mode: {args.mode}",
        f"resolution: {args.resolution:.10g}",
        f"origin: [{origin_x:.8f}, {origin_y:.8f}, {args.yaw:.8f}]",
        f"negate: {args.negate}",
        f"occupied_thresh: {args.occupied_thresh:.10g}",
        f"free_thresh: {args.free_thresh:.10g}",
        "",
    ]
    output.write_text("\n".join(lines), encoding="ascii")

    print(f"wrote {output}")
    print(f"image: {image_name}")
    print(f"size: {width} x {height}")
    print(f"resolution: {args.resolution}")
    print(f"origin: [{origin_x:.8f}, {origin_y:.8f}, {args.yaw:.8f}]")
    print(
        "bounds: "
        f"x=[{origin_x:.8f}, {origin_x + width * args.resolution:.8f}], "
        f"y=[{origin_y:.8f}, {origin_y + height * args.resolution:.8f}]"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pgm", type=Path, help="Input PGM map image")
    parser.add_argument("--output", type=Path, help="Output YAML path; defaults to <pgm>.yaml")
    parser.add_argument("--image", help="Image path to write inside YAML; defaults to the PGM basename")
    parser.add_argument("--resolution", required=True, type=float, help="Map resolution in meters per pixel")
    parser.add_argument("--yaw", default=0.0, type=float, help="Map yaw in radians")

    alignment = parser.add_argument_group("alignment")
    alignment.add_argument("--origin", nargs=2, type=float, metavar=("X", "Y"), help="Set origin directly")
    alignment.add_argument(
        "--center-at",
        nargs=2,
        type=float,
        metavar=("X", "Y"),
        help="Set origin so the map center is at this world coordinate",
    )
    alignment.add_argument(
        "--anchor-pixel",
        nargs=2,
        type=float,
        metavar=("COL", "ROW"),
        help="Image pixel coordinate to align, measured from top-left",
    )
    alignment.add_argument(
        "--anchor-world",
        nargs=2,
        type=float,
        metavar=("X", "Y"),
        help="World coordinate for --anchor-pixel",
    )

    parser.add_argument("--mode", default="trinary")
    parser.add_argument("--negate", default=0, type=int)
    parser.add_argument("--occupied-thresh", default=0.65, type=float)
    parser.add_argument("--free-thresh", default=0.25, type=float)
    args = parser.parse_args()

    if args.resolution <= 0.0:
        parser.error("--resolution must be positive")

    direct_modes = int(args.origin is not None) + int(args.center_at is not None)
    anchor_mode = int(args.anchor_pixel is not None or args.anchor_world is not None)
    if direct_modes + anchor_mode != 1:
        parser.error("choose exactly one alignment mode: --origin, --center-at, or --anchor-pixel with --anchor-world")
    if anchor_mode and (args.anchor_pixel is None or args.anchor_world is None):
        parser.error("--anchor-pixel and --anchor-world must be used together")

    width, height = read_pgm_size(args.pgm)
    origin_x, origin_y = compute_origin(args, width, height)
    write_yaml(args, width, height, origin_x, origin_y)


if __name__ == "__main__":
    main()
