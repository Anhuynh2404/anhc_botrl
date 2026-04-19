#!/usr/bin/env python3
"""Relay OccupancyGrid cost bands to an RGB ``sensor_msgs/Image`` for RViz."""

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image

_MAP_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Blend toward luminance to lower saturation (0 = unchanged, 1 = grayscale).
_DESAT_STRENGTH = 0.48


def _desaturate(r: int, g: int, b: int, strength: float = _DESAT_STRENGTH) -> tuple[int, int, int]:
    y = 0.299 * r + 0.587 * g + 0.114 * b
    t = max(0.0, min(1.0, strength))
    nr = (1.0 - t) * r + t * y
    ng = (1.0 - t) * g + t * y
    nb = (1.0 - t) * b + t * y
    return (int(round(nr)), int(round(ng)), int(round(nb)))


class CostmapColorRelay(Node):
    """Map discrete cost values to pastel RGB (matches Phase 2 costmap bands)."""

    def __init__(self) -> None:
        super().__init__('costmap_color_relay')
        self.create_subscription(
            OccupancyGrid, '/costmap/global', self._cb, _MAP_QOS
        )
        self._pub = self.create_publisher(Image, '/costmap/visual', _MAP_QOS)
        self.get_logger().info(
            'costmap_color_relay: /costmap/global -> /costmap/visual (rgb8)'
        )

    def _cb(self, msg: OccupancyGrid) -> None:
        info = msg.info
        w, h = int(info.width), int(info.height)
        if w <= 0 or h <= 0:
            return
        occ = np.asarray(msg.data, dtype=np.int16).reshape((h, w))
        rgb = np.zeros((h, w, 3), dtype=np.uint8)
        rgb[:] = (255, 255, 255)
        rgb[occ < 0] = _desaturate(64, 64, 72, 0.35)
        rgb[occ == 100] = _desaturate(220, 100, 100)
        rgb[occ == 80] = _desaturate(150, 220, 180)
        rgb[occ == 60] = _desaturate(100, 180, 230)
        rgb[occ == 40] = _desaturate(100, 130, 210)
        rgb[occ == 20] = _desaturate(170, 150, 220)
        rgb[occ == 0] = (255, 255, 255)
        other = (occ >= 0) & (occ != 0) & (occ != 20) & (occ != 40)
        other &= (occ != 60) & (occ != 80) & (occ != 100)
        rgb[other] = _desaturate(200, 200, 210, 0.25)

        out = Image()
        out.header = msg.header
        out.height = h
        out.width = w
        out.encoding = 'rgb8'
        out.is_bigendian = 0
        out.step = w * 3
        out.data = rgb.tobytes()
        self._pub.publish(out)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CostmapColorRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
