#!/usr/bin/env python3
"""Real-time 2D occupancy map streamer for the app.

Subscribes to Point-LIO's registered point cloud, incrementally projects it
into a 2D occupancy grid using the same height bands as
``pcd_to_occupancy_grid.py``, renders the grid to PNG and pushes it to
connected app clients over WebSocket.

Run directly:
    python3 tools/map_ws_streamer.py

App receives JSON text frames:
    {"type": "map", "resolution": 0.1, "origin": [x, y],
     "width": W, "height": H, "stamp": 169..., "png_base64": "..."}
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import io
import json
import threading

import numpy as np
import rclpy
from PIL import Image
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

try:
    import websockets
except ImportError as exc:  # pragma: no cover
    raise SystemExit("pip install websockets") from exc


class MapStreamer(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("map_ws_streamer")
        self.res = args.resolution
        self.origin_x = args.origin_x
        self.origin_y = args.origin_y
        self.width = int(round(args.size_x / self.res))
        self.height = int(round(args.size_y / self.res))
        self.occ_z = (args.occupied_z_min, args.occupied_z_max)
        self.free_z = (args.free_z_min, args.free_z_max)
        self.dilation = args.occupied_dilation

        # Persistent accumulators; never recomputed from scratch.
        self.occ_counts = np.zeros((self.height, self.width), dtype=np.uint32)
        self.free_counts = np.zeros((self.height, self.width), dtype=np.uint32)
        self.lock = threading.Lock()
        self.latest_frame: str | None = None

        # Sensor data QoS: best effort matches Point-LIO publisher.
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(PointCloud2, args.cloud_topic, self.on_cloud, qos)
        self.create_timer(args.publish_period, self.on_render)

        # WebSocket server runs in its own thread + asyncio loop.
        self.host = args.host
        self.port = args.port
        self.clients: set = set()
        self.loop: asyncio.AbstractEventLoop | None = None
        threading.Thread(target=self._run_ws, daemon=True).start()
        self.get_logger().info(
            f"map_ws_streamer up: ws://{self.host}:{self.port}  "
            f"grid {self.width}x{self.height} @ {self.res} m/cell"
        )

    def on_cloud(self, msg: PointCloud2) -> None:
        pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"))
        if pts.size == 0:
            return
        xyz = pts[np.isfinite(pts).all(axis=1)]
        ix = np.floor((xyz[:, 0] - self.origin_x) / self.res).astype(np.int64)
        iy = np.floor((xyz[:, 1] - self.origin_y) / self.res).astype(np.int64)
        keep = (ix >= 0) & (ix < self.width) & (iy >= 0) & (iy < self.height)
        ix, iy, z = ix[keep], iy[keep], xyz[keep, 2]

        occ = (z >= self.occ_z[0]) & (z <= self.occ_z[1])
        free = (z >= self.free_z[0]) & (z <= self.free_z[1])
        with self.lock:
            np.add.at(self.occ_counts, (iy[occ], ix[occ]), 1)
            np.add.at(self.free_counts, (iy[free], ix[free]), 1)

    def _build_grid(self) -> np.ndarray:
        with self.lock:
            occ = self.occ_counts > 0
            free = self.free_counts > 0
        grid = np.full((self.height, self.width), 205, dtype=np.uint8)  # unknown
        grid[free] = 254  # free
        for _ in range(max(self.dilation, 0)):
            expanded = occ.copy()
            expanded[1:, :] |= occ[:-1, :]
            expanded[:-1, :] |= occ[1:, :]
            expanded[:, 1:] |= occ[:, :-1]
            expanded[:, :-1] |= occ[:, 1:]
            occ = expanded
        grid[occ] = 0  # occupied
        return grid

    def on_render(self) -> None:
        grid = self._build_grid()
        # ROS map origin is bottom-left; image origin is top-left.
        image = np.flipud(grid)
        buf = io.BytesIO()
        Image.fromarray(image, mode="L").save(buf, format="PNG")
        frame = json.dumps(
            {
                "type": "map",
                "resolution": self.res,
                "origin": [self.origin_x, self.origin_y],
                "width": self.width,
                "height": self.height,
                "stamp": self.get_clock().now().nanoseconds,
                "png_base64": base64.b64encode(buf.getvalue()).decode("ascii"),
            }
        )
        self.latest_frame = frame
        if self.loop is not None:
            asyncio.run_coroutine_threadsafe(self._broadcast(frame), self.loop)

    async def _broadcast(self, frame: str) -> None:
        if not self.clients:
            return
        await asyncio.gather(
            *(self._safe_send(c, frame) for c in list(self.clients)),
            return_exceptions=True,
        )

    @staticmethod
    async def _safe_send(client, frame: str) -> None:
        try:
            await client.send(frame)
        except Exception:
            pass

    async def _handler(self, client) -> None:
        self.clients.add(client)
        try:
            if self.latest_frame is not None:
                await client.send(self.latest_frame)
            await client.wait_closed()
        finally:
            self.clients.discard(client)

    def _run_ws(self) -> None:
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)

        async def serve() -> None:
            async with websockets.serve(self._handler, self.host, self.port):
                await asyncio.Future()

        self.loop.run_until_complete(serve())


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cloud-topic", default="/cloud_registered")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", default=8765, type=int)
    parser.add_argument("--publish-period", default=1.0, type=float, help="seconds")
    parser.add_argument("--resolution", default=0.10, type=float)
    parser.add_argument("--size-x", default=60.0, type=float, help="map width in meters")
    parser.add_argument("--size-y", default=60.0, type=float, help="map height in meters")
    parser.add_argument("--origin-x", default=-30.0, type=float)
    parser.add_argument("--origin-y", default=-30.0, type=float)
    parser.add_argument("--occupied-z-min", default=-0.8, type=float)
    parser.add_argument("--occupied-z-max", default=0.3, type=float)
    parser.add_argument("--free-z-min", default=-1.30, type=float)
    parser.add_argument("--free-z-max", default=-0.35, type=float)
    parser.add_argument("--occupied-dilation", default=1, type=int)
    args = parser.parse_args()

    rclpy.init()
    node = MapStreamer(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
