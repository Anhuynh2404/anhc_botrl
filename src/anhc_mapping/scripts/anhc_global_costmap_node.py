#!/usr/bin/env python3
"""Global static costmap node.

Subscribes:  /map                    (OccupancyGrid — SLAM map, best-effort)
Publishes:   /costmap/global         (OccupancyGrid — inflated global costmap)
             /costmap/global/metadata (String — JSON band config)

Re-inflates every time the SLAM map is updated.
When use_discrete_bands=true, produces a banded gradient:
  ≤ robot_radius         → 100 (lethal)
  robot_radius .. band_1 →  80
  band_1 .. band_2       →  60
  band_2 .. band_3       →  40
  band_3 .. band_4       →  20
  > band_4               →   0 (free)
"""

import json

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
from std_msgs.msg import String

# Match nav2 map_server (reliable + transient_local) so late subscribers still receive data.
_MAP_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

try:
    from scipy.ndimage import distance_transform_edt
    _SCIPY_OK = True
except ImportError:
    _SCIPY_OK = False


class AhncGlobalCostmapNode(Node):

    def __init__(self):
        super().__init__('anhc_global_costmap_node')

        self.declare_parameter('robot_radius', 0.30)
        self.declare_parameter('inflation_band_1', 0.45)
        self.declare_parameter('inflation_band_2', 0.70)
        self.declare_parameter('inflation_band_3', 1.00)
        self.declare_parameter('inflation_band_4', 1.20)
        self.declare_parameter('use_discrete_bands', True)
        self.declare_parameter('cost_scaling_factor', 3.0)
        # Legacy parameter kept for backward compatibility
        self.declare_parameter('inflation_radius', 0.45)

        self._sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, _MAP_QOS)
        self._pub = self.create_publisher(
            OccupancyGrid, '/costmap/global', _MAP_QOS)
        self._meta_pub = self.create_publisher(
            String, '/costmap/global/metadata', _MAP_QOS)

        self._last_logged_params: tuple | None = None
        _p0 = self._read_params()
        self._emit_band_log(*_p0)
        self._last_logged_params = _p0
        self.get_logger().info('anhc_global_costmap_node started')

    # ------------------------------------------------------------------
    def _read_params(self):
        """Read all band parameters from the parameter server."""
        robot_r = self.get_parameter('robot_radius').value
        band_1 = self.get_parameter('inflation_band_1').value
        band_2 = self.get_parameter('inflation_band_2').value
        band_3 = self.get_parameter('inflation_band_3').value
        band_4 = self.get_parameter('inflation_band_4').value
        use_bands = self.get_parameter('use_discrete_bands').value
        scale = self.get_parameter('cost_scaling_factor').value
        infl_r = self.get_parameter('inflation_radius').value
        return robot_r, band_1, band_2, band_3, band_4, use_bands, scale, infl_r

    def _emit_band_log(
        self,
        robot_r,
        b1,
        b2,
        b3,
        b4,
        use_bands,
        scale,
        infl_r,
    ):
        if use_bands:
            self.get_logger().info(
                f'Costmap bands: lethal={robot_r}m | 80={b1}m | 60={b2}m | '
                f'40={b3}m | 20={b4}m'
            )
        else:
            self.get_logger().info(
                f'Costmap mode: exponential decay (cost_scaling_factor={scale}, '
                f'inflation_radius={infl_r}m)'
            )

    def _publish_metadata(self, robot_r, b1, b2, b3, b4):
        meta = {
            'robot_radius': robot_r,
            'bands': [b1, b2, b3, b4],
            'costs': [100, 80, 60, 40, 20, 0],
        }
        msg = String()
        msg.data = json.dumps(meta)
        self._meta_pub.publish(msg)

    # ------------------------------------------------------------------
    def _map_cb(self, msg: OccupancyGrid) -> None:
        # Re-read parameters on every callback so runtime ros2 param set takes effect immediately
        robot_r, band_1, band_2, band_3, band_4, use_bands, scale, infl_r = self._read_params()
        log_key = (
            robot_r,
            band_1,
            band_2,
            band_3,
            band_4,
            use_bands,
            scale,
            infl_r,
        )
        if log_key != self._last_logged_params:
            self._emit_band_log(
                robot_r,
                band_1,
                band_2,
                band_3,
                band_4,
                use_bands,
                scale,
                infl_r,
            )
            self._last_logged_params = log_key

        info = msg.info
        w, h, res = info.width, info.height, info.resolution

        grid = np.array(msg.data, dtype=np.int8).reshape((h, w))
        obstacles = grid >= 65

        cost = np.zeros((h, w), dtype=np.int8)

        if use_bands and _SCIPY_OK:
            dist_m = distance_transform_edt(~obstacles) * res
            cost[dist_m <= robot_r] = 100
            cost[(dist_m > robot_r) & (dist_m <= band_1)] = 80
            cost[(dist_m > band_1) & (dist_m <= band_2)] = 60
            cost[(dist_m > band_2) & (dist_m <= band_3)] = 40
            cost[(dist_m > band_3) & (dist_m <= band_4)] = 20

        elif _SCIPY_OK:
            # Legacy exponential fallback
            dist_m = distance_transform_edt(~obstacles) * res
            cost[dist_m < robot_r] = 100
            band = (dist_m >= robot_r) & (dist_m < infl_r)
            if band.any():
                d = dist_m[band]
                raw = (99.0 * np.exp(-scale * (d - robot_r))).astype(np.int8)
                cost[band] = np.clip(raw, 1, 99)

        else:
            # scipy not available — simple dilation fallback
            cost[obstacles] = 100
            cells = max(1, int(infl_r / res))
            for dy in range(-cells, cells + 1):
                for dx in range(-cells, cells + 1):
                    if dy == 0 and dx == 0:
                        continue
                    rolled = np.roll(np.roll(obstacles, dy, axis=0), dx, axis=1)
                    cost[(rolled > 0) & (cost == 0)] = 50

        # Preserve unknown cells (-1 from SLAM)
        cost[grid < 0] = -1

        out = OccupancyGrid()
        out.header = msg.header
        out.info = info
        out.data = cost.flatten().tolist()
        self._pub.publish(out)

        self._publish_metadata(robot_r, band_1, band_2, band_3, band_4)


def main(args=None):
    rclpy.init(args=args)
    node = AhncGlobalCostmapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
