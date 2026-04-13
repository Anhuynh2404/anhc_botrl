#!/usr/bin/env python3
"""Global static costmap node.

Subscribes:  /map           (OccupancyGrid — SLAM map, best-effort)
Publishes:   /costmap/global (OccupancyGrid — inflated global costmap)

Re-inflates every time the SLAM map is updated.
"""

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

        self.declare_parameter('inflation_radius', 0.45)
        self.declare_parameter('cost_scaling_factor', 3.0)
        self.declare_parameter('robot_radius', 0.30)

        self._infl_r = self.get_parameter('inflation_radius').value
        self._scale = self.get_parameter('cost_scaling_factor').value
        self._robot_r = self.get_parameter('robot_radius').value

        self._sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, _MAP_QOS)
        self._pub = self.create_publisher(
            OccupancyGrid, '/costmap/global', _MAP_QOS)

        self.get_logger().info('anhc_global_costmap_node started')

    # ------------------------------------------------------------------
    def _map_cb(self, msg: OccupancyGrid) -> None:
        info = msg.info
        w, h, res = info.width, info.height, info.resolution

        grid = np.array(msg.data, dtype=np.int8).reshape((h, w))
        obstacles = grid >= 65

        cost = np.zeros((h, w), dtype=np.int8)

        if _SCIPY_OK:
            dist_m = distance_transform_edt(~obstacles) * res
            cost[dist_m < self._robot_r] = 100
            band = (dist_m >= self._robot_r) & (dist_m < self._infl_r)
            if band.any():
                d = dist_m[band]
                raw = (99.0 * np.exp(
                    -self._scale * (d - self._robot_r))).astype(np.int8)
                cost[band] = np.clip(raw, 1, 99)
        else:
            cost[obstacles] = 100
            cells = max(1, int(self._infl_r / res))
            for dy in range(-cells, cells + 1):
                for dx in range(-cells, cells + 1):
                    if dy == 0 and dx == 0:
                        continue
                    rolled = np.roll(np.roll(obstacles, dy, axis=0), dx, axis=1)
                    cost[(rolled > 0) & (cost == 0)] = 50

        cost[grid < 0] = -1

        out = OccupancyGrid()
        out.header = msg.header
        out.info = info
        out.data = cost.flatten().tolist()
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = AhncGlobalCostmapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
