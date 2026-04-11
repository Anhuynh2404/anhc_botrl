#!/usr/bin/env python3
"""Costmap node producing inflated global and local costmaps.

Subscribes:
  /map   (nav_msgs/OccupancyGrid)
  /scan  (sensor_msgs/LaserScan)
Publishes:
  /costmap/global                 (nav_msgs/OccupancyGrid)
  /costmap/global/costmap_updates (map_msgs/OccupancyGridUpdate)
  /costmap/local                  (nav_msgs/OccupancyGrid)
"""

import math
from copy import deepcopy
from typing import Optional

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from map_msgs.msg import OccupancyGridUpdate
    _MAP_UPDATES_OK = True
except ImportError:
    _MAP_UPDATES_OK = False

try:
    from scipy.ndimage import distance_transform_edt
    _SCIPY_OK = True
except ImportError:
    _SCIPY_OK = False


class AhncCostmapNode(Node):
    """Inflation-based costmap generator with dynamic scan obstacle stamping."""

    def __init__(self) -> None:
        super().__init__("anhc_costmap_node")

        # Global params
        self.declare_parameter("global_costmap.global_costmap.ros__parameters.use_sim_time", True)
        self.declare_parameter("global_costmap.global_costmap.ros__parameters.resolution", 0.05)
        self.declare_parameter("global_costmap.global_costmap.ros__parameters.robot_radius", 0.3)
        self.declare_parameter("global_costmap.global_costmap.ros__parameters.publish_frequency", 1.0)
        self.declare_parameter("global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.obstacle_max_range", 15.0)
        self.declare_parameter("global_costmap.global_costmap.ros__parameters.inflation_layer.inflation_radius", 0.55)
        self.declare_parameter("global_costmap.global_costmap.ros__parameters.inflation_layer.cost_scaling_factor", 3.0)

        # Local params
        self.declare_parameter("local_costmap.local_costmap.ros__parameters.width", 4.0)
        self.declare_parameter("local_costmap.local_costmap.ros__parameters.height", 4.0)
        self.declare_parameter("local_costmap.local_costmap.ros__parameters.publish_frequency", 5.0)

        self._inflation_radius = float(
            self.get_parameter(
                "global_costmap.global_costmap.ros__parameters.inflation_layer.inflation_radius"
            ).value
        )
        self._cost_scaling = float(
            self.get_parameter(
                "global_costmap.global_costmap.ros__parameters.inflation_layer.cost_scaling_factor"
            ).value
        )
        self._robot_radius = float(
            self.get_parameter("global_costmap.global_costmap.ros__parameters.robot_radius").value
        )
        self._scan_max_range = float(
            self.get_parameter(
                "global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.obstacle_max_range"
            ).value
        )
        self._global_hz = float(
            self.get_parameter("global_costmap.global_costmap.ros__parameters.publish_frequency").value
        )
        self._local_hz = float(
            self.get_parameter("local_costmap.local_costmap.ros__parameters.publish_frequency").value
        )
        self._local_width = float(
            self.get_parameter("local_costmap.local_costmap.ros__parameters.width").value
        )
        self._local_height = float(
            self.get_parameter("local_costmap.local_costmap.ros__parameters.height").value
        )

        self._latest_map: Optional[OccupancyGrid] = None
        self._latest_scan: Optional[LaserScan] = None
        self._last_global: Optional[OccupancyGrid] = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(OccupancyGrid, "/map", self._cb_map, 10)
        self.create_subscription(LaserScan, "/scan", self._cb_scan, 10)
        self._pub_global = self.create_publisher(OccupancyGrid, "/costmap/global", 10)
        self._pub_local = self.create_publisher(OccupancyGrid, "/costmap/local", 10)
        self._pub_updates = None
        if _MAP_UPDATES_OK:
            self._pub_updates = self.create_publisher(
                OccupancyGridUpdate, "/costmap/global/costmap_updates", 10
            )

        self.create_timer(1.0 / max(0.1, self._global_hz), self._publish_global)
        self.create_timer(1.0 / max(0.1, self._local_hz), self._publish_local)
        self.get_logger().info("[anhc_costmap_node] started")

    def _cb_map(self, msg: OccupancyGrid) -> None:
        self._latest_map = msg

    def _cb_scan(self, msg: LaserScan) -> None:
        self._latest_scan = msg

    def _publish_global(self) -> None:
        if self._latest_map is None:
            return
        out = self._build_global_costmap(self._latest_map)
        self._pub_global.publish(out)
        self._publish_update(out)
        self._last_global = out

    def _publish_local(self) -> None:
        if self._latest_map is None:
            return
        global_cost = self._build_global_costmap(self._latest_map)
        local = self._extract_local_window(global_cost)
        if local is not None:
            self._pub_local.publish(local)

    def _build_global_costmap(self, map_msg: OccupancyGrid) -> OccupancyGrid:
        info = map_msg.info
        h, w = int(info.height), int(info.width)
        res = float(info.resolution)
        raw = np.array(map_msg.data, dtype=np.int16).reshape((h, w))

        lethal = raw >= 100
        unknown = raw < 0
        lethal = self._stamp_scan_obstacles(lethal, info)

        costs = np.zeros((h, w), dtype=np.uint8)
        costs[lethal] = 254

        if _SCIPY_OK:
            dist = distance_transform_edt(~lethal) * res
            band = (~lethal) & (dist <= self._inflation_radius)
            scaled = 252.0 * np.exp(
                -self._cost_scaling * np.maximum(0.0, dist - self._robot_radius)
            )
            inf_vals = np.clip(scaled, 1, 252).astype(np.uint8)
            costs[band] = np.maximum(costs[band], inf_vals[band])
        else:
            cells = max(1, int(self._inflation_radius / max(1e-6, res)))
            for dy in range(-cells, cells + 1):
                for dx in range(-cells, cells + 1):
                    if dx == 0 and dy == 0:
                        continue
                    shifted = np.roll(np.roll(lethal, dy, axis=0), dx, axis=1)
                    costs[(shifted) & (~lethal)] = np.maximum(
                        costs[(shifted) & (~lethal)], np.uint8(150)
                    )

        costs[unknown] = 255

        out = OccupancyGrid()
        out.header = map_msg.header
        # Copy so _extract_local_window cannot mutate /map metadata via shared refs.
        out.info = deepcopy(map_msg.info)
        # nav_msgs/OccupancyGrid stores int8; reinterpret uint8 cost bytes.
        out.data = costs.view(np.int8).flatten().tolist()
        return out

    def _stamp_scan_obstacles(self, lethal: np.ndarray, info) -> np.ndarray:
        if self._latest_scan is None:
            return lethal

        try:
            tf = self._tf_buffer.lookup_transform("map", "lidar_link", rclpy.time.Time())
            sx = tf.transform.translation.x
            sy = tf.transform.translation.y
            q = tf.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        except TransformException:
            return lethal

        w = int(info.width)
        h = int(info.height)
        ox = float(info.origin.position.x)
        oy = float(info.origin.position.y)
        res = float(info.resolution)
        out = lethal.copy()

        angle = self._latest_scan.angle_min
        for r in self._latest_scan.ranges:
            if not math.isfinite(r):
                angle += self._latest_scan.angle_increment
                continue
            if r < self._latest_scan.range_min or r > min(self._latest_scan.range_max, self._scan_max_range):
                angle += self._latest_scan.angle_increment
                continue
            wx = sx + r * math.cos(yaw + angle)
            wy = sy + r * math.sin(yaw + angle)
            cx = int((wx - ox) / res)
            cy = int((wy - oy) / res)
            if 0 <= cx < w and 0 <= cy < h:
                out[cy, cx] = True
            angle += self._latest_scan.angle_increment
        return out

    def _extract_local_window(self, global_map: OccupancyGrid) -> Optional[OccupancyGrid]:
        try:
            tf = self._tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
            rx = tf.transform.translation.x
            ry = tf.transform.translation.y
        except TransformException:
            return None

        info = global_map.info
        res = float(info.resolution)
        ox = float(info.origin.position.x)
        oy = float(info.origin.position.y)
        gw, gh = int(info.width), int(info.height)
        data = np.array(global_map.data, dtype=np.int16).reshape((gh, gw))

        hw_cells = int((self._local_width * 0.5) / res)
        hh_cells = int((self._local_height * 0.5) / res)
        cx = int((rx - ox) / res)
        cy = int((ry - oy) / res)

        x0 = max(0, cx - hw_cells)
        y0 = max(0, cy - hh_cells)
        x1 = min(gw, cx + hw_cells)
        y1 = min(gh, cy + hh_cells)
        if x1 <= x0 or y1 <= y0:
            return None

        sub = data[y0:y1, x0:x1]
        out = OccupancyGrid()
        out.header = global_map.header
        out.info = deepcopy(global_map.info)
        out.info.width = int(x1 - x0)
        out.info.height = int(y1 - y0)
        out.info.origin.position.x = ox + x0 * res
        out.info.origin.position.y = oy + y0 * res
        out.data = sub.flatten().tolist()
        return out

    def _publish_update(self, current: OccupancyGrid) -> None:
        if self._pub_updates is None:
            return
        upd = OccupancyGridUpdate()
        upd.header = current.header
        upd.x = 0
        upd.y = 0
        upd.width = current.info.width
        upd.height = current.info.height
        upd.data = current.data
        self._pub_updates.publish(upd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AhncCostmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
