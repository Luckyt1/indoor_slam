#!/usr/bin/env python3
"""Clean or extract walls from ROS-style PGM occupancy maps.

The script treats dark pixels as occupied cells, keeps likely structural
edges/walls, and removes small isolated obstacle components. In wall-only mode
it also converts every non-wall pixel to white so the result contains only
black walls and white free space.
"""

from __future__ import annotations

import argparse
from collections import Counter, deque
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


WHITESPACE = b" \t\r\n"


@dataclass(frozen=True)
class PgmImage:
    width: int
    height: int
    max_value: int
    pixels: bytearray


@dataclass
class Component:
    pixels: list[int]
    min_x: int
    min_y: int
    max_x: int
    max_y: int

    @property
    def area(self) -> int:
        return len(self.pixels)

    @property
    def width(self) -> int:
        return self.max_x - self.min_x + 1

    @property
    def height(self) -> int:
        return self.max_y - self.min_y + 1

    @property
    def span(self) -> int:
        return max(self.width, self.height)


def _next_token(data: bytes, pos: int) -> tuple[bytes, int]:
    n = len(data)
    while pos < n:
        if data[pos] in WHITESPACE:
            pos += 1
            continue
        if data[pos] == ord("#"):
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


def read_pgm(path: Path) -> PgmImage:
    data = path.read_bytes()
    pos = 0
    magic, pos = _next_token(data, pos)
    if magic not in (b"P2", b"P5"):
        raise ValueError(f"{path}: expected P2 or P5 PGM, got {magic!r}")

    width_token, pos = _next_token(data, pos)
    height_token, pos = _next_token(data, pos)
    max_token, pos = _next_token(data, pos)
    width = int(width_token)
    height = int(height_token)
    max_value = int(max_token)
    if width <= 0 or height <= 0:
        raise ValueError(f"{path}: invalid image size {width}x{height}")
    if not 0 < max_value <= 255:
        raise ValueError(f"{path}: only max_value 1..255 is supported, got {max_value}")

    expected = width * height
    if magic == b"P5":
        if pos < len(data) and data[pos] in WHITESPACE:
            pos += 1
        pixels = bytearray(data[pos : pos + expected])
        if len(pixels) != expected:
            raise ValueError(f"{path}: expected {expected} pixels, got {len(pixels)}")
        return PgmImage(width, height, max_value, pixels)

    pixels = bytearray()
    for _ in range(expected):
        token, pos = _next_token(data, pos)
        value = int(token)
        if not 0 <= value <= max_value:
            raise ValueError(f"{path}: P2 pixel value {value} exceeds max_value {max_value}")
        pixels.append(value)
    return PgmImage(width, height, max_value, pixels)


def write_pgm(path: Path, image: PgmImage) -> None:
    header = f"P5\n{image.width} {image.height}\n{image.max_value}\n".encode("ascii")
    path.write_bytes(header + bytes(image.pixels))


def iter_neighbors(index: int, width: int, height: int, connectivity: int) -> Iterable[int]:
    y, x = divmod(index, width)
    for dy, dx in ((-1, 0), (0, -1), (0, 1), (1, 0)):
        ny = y + dy
        nx = x + dx
        if 0 <= nx < width and 0 <= ny < height:
            yield ny * width + nx

    if connectivity == 8:
        for dy, dx in ((-1, -1), (-1, 1), (1, -1), (1, 1)):
            ny = y + dy
            nx = x + dx
            if 0 <= nx < width and 0 <= ny < height:
                yield ny * width + nx


def find_components(mask: bytearray, width: int, height: int, connectivity: int) -> list[Component]:
    seen = bytearray(len(mask))
    components: list[Component] = []

    for start, occupied in enumerate(mask):
        if not occupied or seen[start]:
            continue

        queue: deque[int] = deque([start])
        seen[start] = 1
        pixels: list[int] = []
        min_x = width
        min_y = height
        max_x = 0
        max_y = 0

        while queue:
            index = queue.popleft()
            pixels.append(index)
            y, x = divmod(index, width)
            min_x = min(min_x, x)
            min_y = min(min_y, y)
            max_x = max(max_x, x)
            max_y = max(max_y, y)

            for neighbor in iter_neighbors(index, width, height, connectivity):
                if mask[neighbor] and not seen[neighbor]:
                    seen[neighbor] = 1
                    queue.append(neighbor)

        components.append(Component(pixels, min_x, min_y, max_x, max_y))

    return components


def should_keep_component(
    component: Component,
    width: int,
    height: int,
    border_margin: int,
    keep_border: bool,
    min_keep_area: int,
    min_keep_span: int,
    keep_rule: str,
) -> tuple[bool, str]:
    touches_border = (
        component.min_x <= border_margin
        or component.min_y <= border_margin
        or component.max_x >= width - 1 - border_margin
        or component.max_y >= height - 1 - border_margin
    )
    if keep_border and touches_border:
        return True, "border"
    if keep_rule == "area-or-span" and component.area >= min_keep_area:
        return True, "area"
    if component.span >= min_keep_span:
        return True, "span"
    return False, "small"


def replacement_value(
    component: Component,
    image: PgmImage,
    occupied: bytearray,
    connectivity: int,
    fill_mode: str,
    free_value: int,
    unknown_value: int,
) -> int:
    if fill_mode == "free":
        return free_value
    if fill_mode == "unknown":
        return unknown_value

    component_pixels = set(component.pixels)
    surrounding = Counter()
    for index in component.pixels:
        for neighbor in iter_neighbors(index, image.width, image.height, connectivity):
            if neighbor in component_pixels or occupied[neighbor]:
                continue
            surrounding[image.pixels[neighbor]] += 1

    if not surrounding:
        return free_value
    return surrounding.most_common(1)[0][0]


def copy_yaml(src: Path, dst: Path, output_pgm: Path) -> None:
    lines = src.read_text(encoding="utf-8").splitlines(keepends=True)
    replacement = f"image: {output_pgm.name}\n"
    updated = []
    replaced = False
    for line in lines:
        if line.lstrip().startswith("image:"):
            indent = line[: len(line) - len(line.lstrip())]
            updated.append(indent + replacement)
            replaced = True
        else:
            updated.append(line)
    if not replaced:
        updated.insert(0, replacement)
    dst.write_text("".join(updated), encoding="utf-8")


def clean_map(args: argparse.Namespace) -> dict[str, int]:
    image = read_pgm(args.input)
    occupied = bytearray(1 if value <= args.occupied_max else 0 for value in image.pixels)
    components = find_components(occupied, image.width, image.height, args.connectivity)
    kept_wall_mask = bytearray(len(image.pixels))

    kept = 0
    removed = 0
    removed_pixels = 0
    binarized_pixels = 0
    reasons = Counter()

    for component in components:
        keep, reason = should_keep_component(
            component,
            image.width,
            image.height,
            args.border_margin,
            args.keep_border,
            args.min_keep_area,
            args.min_keep_span,
            args.keep_rule,
        )
        reasons[reason] += 1
        if keep:
            kept += 1
            if args.binary:
                for index in component.pixels:
                    kept_wall_mask[index] = 1
            continue

        removed += 1
        removed_pixels += component.area
        value = replacement_value(
            component,
            image,
            occupied,
            args.connectivity,
            args.fill_mode,
            args.free_value,
            args.unknown_value,
        )
        for index in component.pixels:
            image.pixels[index] = value

    if args.binary:
        for index, is_wall in enumerate(kept_wall_mask):
            value = 0 if is_wall else args.free_value
            if image.pixels[index] != value:
                binarized_pixels += 1
            image.pixels[index] = value

    if args.dry_run:
        output_written = 0
    else:
        if args.output.exists() and not args.force:
            raise FileExistsError(f"{args.output} exists; pass --force to overwrite it")
        write_pgm(args.output, image)
        output_written = 1
        if args.copy_yaml:
            if not args.copy_yaml.exists():
                raise FileNotFoundError(args.copy_yaml)
            yaml_output = args.output.with_suffix(".yaml")
            if yaml_output.exists() and not args.force:
                raise FileExistsError(f"{yaml_output} exists; pass --force to overwrite it")
            copy_yaml(args.copy_yaml, yaml_output, args.output)

    return {
        "width": image.width,
        "height": image.height,
        "components": len(components),
        "kept_components": kept,
        "removed_components": removed,
        "removed_pixels": removed_pixels,
        "kept_by_border": reasons["border"],
        "kept_by_area": reasons["area"],
        "kept_by_span": reasons["span"],
        "binarized_pixels": binarized_pixels,
        "output_written": output_written,
    }


def apply_mode_defaults(args: argparse.Namespace) -> None:
    if args.wall_only:
        args.binary = True
        args.fill_mode = "free"
        if args.free_value is None:
            args.free_value = 255
        if args.keep_rule is None:
            args.keep_rule = "span-only"
        if args.keep_border is None:
            args.keep_border = False
        if args.min_keep_area is None:
            args.min_keep_area = 1_000_000_000
        if args.min_keep_span is None:
            args.min_keep_span = 45
        return

    if args.keep_rule is None:
        args.keep_rule = "area-or-span"
    if args.free_value is None:
        args.free_value = 254
    if args.keep_border is None:
        args.keep_border = True
    if args.min_keep_area is None:
        args.min_keep_area = 90
    if args.min_keep_span is None:
        args.min_keep_span = 30


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Remove small floating obstacles from a PGM occupancy map while "
            "preserving likely wall/edge components, or extract wall-only maps."
        )
    )
    parser.add_argument("input", type=Path, help="input .pgm file")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="output .pgm file; defaults to <input>_cleaned.pgm",
    )
    parser.add_argument(
        "--copy-yaml",
        type=Path,
        help="copy a ROS map YAML and update its image field to the cleaned PGM",
    )
    parser.add_argument(
        "--occupied-max",
        type=int,
        default=100,
        help="pixels <= this value are treated as occupied obstacles (default: 100)",
    )
    parser.add_argument(
        "--wall-only",
        action="store_true",
        help=(
            "extract only long wall-like occupied components; also enables "
            "--binary and fills removed areas with white"
        ),
    )
    parser.add_argument(
        "--binary",
        action="store_true",
        help="write only black and white pixels: kept walls become 0, everything else becomes --free-value",
    )
    parser.add_argument(
        "--keep-rule",
        choices=("area-or-span", "span-only"),
        help=(
            "component keep rule; default is area-or-span, while --wall-only "
            "defaults to span-only"
        ),
    )
    parser.add_argument(
        "--min-keep-area",
        type=int,
        help="keep occupied components with at least this many cells (default: 90)",
    )
    parser.add_argument(
        "--min-keep-span",
        type=int,
        help=(
            "keep occupied components whose width or height reaches this many cells "
            "(default: 30, or 45 with --wall-only)"
        ),
    )
    parser.add_argument(
        "--border-margin",
        type=int,
        default=2,
        help="keep components touching this many cells from the map border (default: 2)",
    )
    border_group = parser.add_mutually_exclusive_group()
    border_group.add_argument(
        "--keep-border",
        dest="keep_border",
        action="store_true",
        help="keep occupied components near the map border",
    )
    border_group.add_argument(
        "--no-keep-border",
        dest="keep_border",
        action="store_false",
        help="do not keep components just because they touch the map border",
    )
    parser.set_defaults(keep_border=None)
    parser.add_argument(
        "--connectivity",
        type=int,
        choices=(4, 8),
        default=8,
        help="connected-component neighborhood (default: 8)",
    )
    parser.add_argument(
        "--fill-mode",
        choices=("neighbor", "free", "unknown"),
        default="neighbor",
        help="replacement value for removed obstacles (default: neighbor)",
    )
    parser.add_argument(
        "--free-value",
        type=int,
        help="free-space pixel value used by --fill-mode free (default: 254, or 255 with --wall-only)",
    )
    parser.add_argument(
        "--unknown-value",
        type=int,
        default=205,
        help="unknown-space pixel value used by --fill-mode unknown (default: 205)",
    )
    parser.add_argument("--dry-run", action="store_true", help="print statistics without writing output")
    parser.add_argument("--force", action="store_true", help="overwrite existing output files")
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    apply_mode_defaults(args)
    if args.output is None:
        suffix = "walls" if args.wall_only else "cleaned"
        args.output = args.input.with_name(f"{args.input.stem}_{suffix}{args.input.suffix}")

    if args.input.resolve() == args.output.resolve():
        parser.error("input and output must be different files")
    for value_name in ("occupied_max", "free_value", "unknown_value"):
        value = getattr(args, value_name)
        if not 0 <= value <= 255:
            parser.error(f"--{value_name.replace('_', '-')} must be between 0 and 255")
    if args.min_keep_area < 1:
        parser.error("--min-keep-area must be >= 1")
    if args.min_keep_span < 1:
        parser.error("--min-keep-span must be >= 1")
    if args.border_margin < 0:
        parser.error("--border-margin must be >= 0")

    stats = clean_map(args)
    print(f"map: {stats['width']}x{stats['height']}")
    print(
        "components: "
        f"{stats['components']} total, "
        f"{stats['kept_components']} kept, "
        f"{stats['removed_components']} removed"
    )
    print(
        "kept by: "
        f"border={stats['kept_by_border']}, "
        f"area={stats['kept_by_area']}, "
        f"span={stats['kept_by_span']}"
    )
    print(f"removed pixels: {stats['removed_pixels']}")
    if args.binary:
        print(f"binarized pixels: {stats['binarized_pixels']}")
    if stats["output_written"]:
        print(f"wrote: {args.output}")
        if args.copy_yaml:
            print(f"wrote: {args.output.with_suffix('.yaml')}")


if __name__ == "__main__":
    main()
