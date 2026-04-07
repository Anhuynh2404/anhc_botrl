#!/usr/bin/env python3
"""Dynamic costmap node.

Subscribes:
  /map                    (OccupancyGrid — from slam_toolbox)
  /perception/obstacles   (MarkerArray — from obstacle detector)
Publishes:
  /costmap/grid           (OccupancyGrid — inflated costmap)
  /costmap/metadata       (MapMetaData)

Inflation uses scipy distance_transform_edt when available,
with a numpy fallback.
"""

import numpy as np
import rclpy
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray

try:
    from scipy.ndimage import distance_transform_edt
    _SCIPY_OK = True
except ImportError:
    _SCIPY_OK = False


class AhncCostmapNode(Node):

    def __init__(self):
        super().__init__('anhc_costmap_node')

        self.declare_parameter('inflation_radius', 0.45)
        self.declare_parameter('cost_scaling_factor', 3.0)
        self.declare_parameter('robot_radius', 0.30)
        self.declare_parameter('update_rate', 2.0)

        self._infl_r = self.get_parameter('inflation_radius').value
        self._scale = self.get_parameter('cost_scaling_factor').value
        self._robot_r = self.get_parameter('robot_radius').value
        rate = self.get_parameter('update_rate').value

        self._base_map: OccupancyGrid | None = None
        self._dyn_obstacles: list[tuple[float, float]] = []

        self._sub_map = self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, 1)
        self._sub_obs = self.create_subscription(
            MarkerArray, '/perception/obstacles', self._obs_cb, 10)
        self._pub_cost = self.create_publisher(
            OccupancyGrid, '/costmap/grid', 1)
        self._pub_meta = self.create_publisher(
            MapMetaData, '/costmap/metadata', 1)

        self._timer = self.create_timer(1.0 / rate, self._publish_costmap)

        if not _SCIPY_OK:
            self.get_logger().warn(
                'scipy not found — using simplified inflation.')
        self.get_logger().info('anhc_costmap_node started')

    # ------------------------------------------------------------------
    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._base_map = msg

    def _obs_cb(self, msg: MarkerArray) -> None:
        self._dyn_obstacles = [
            (m.pose.position.x, m.pose.position.y)
            for m in msg.markers
            if m.action not in (Marker.DELETE, Marker.DELETEALL)  # noqa: F821
        ]

    # ------------------------------------------------------------------
    def _publish_costmap(self) -> None:
        if self._base_map is None:
            return

        info = self._base_map.info
        w, h, res = info.width, info.height, info.resolution

        grid = np.array(self._base_map.data, dtype=np.int8).reshape((h, w))
        obstacles = grid >= 65

        # Stamp dynamic obstacles onto the binary obstacle map
        ox, oy = info.origin.position.x, info.origin.position.y
        blob_r = max(1, int(0.25 / res))
        for px, py in self._dyn_obstacles:
            cx = int((px - ox) / res)
            cy = int((py - oy) / res)
            for dy in range(-blob_r, blob_r + 1):
                for dx in range(-blob_r, blob_r + 1):
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < w and 0 <= ny < h:
                        obstacles[ny, nx] = True

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

        cost[grid < 0] = -1   # preserve unknown

        msg_out = OccupancyGrid()
        msg_out.header = self._base_map.header
        msg_out.info = info
        msg_out.data = cost.flatten().tolist()
        self._pub_cost.publish(msg_out)
        self._pub_meta.publish(info)


# Marker action constants (avoid importing visualization_msgs at module level)
class Marker:
    DELETE = 2
    DELETEALL = 3


def main(args=None):
    rclpy.init(args=args)
    node = AhncCostmapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
