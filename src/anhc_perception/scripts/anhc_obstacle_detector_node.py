#!/usr/bin/env python3
"""Obstacle detector node.

Clusters /perception/cloud_obstacles with DBSCAN (scikit-learn).
Publishes a MarkerArray (bounding boxes) and a Float32MultiArray
with the flat obstacle list: [cx, cy, cz, dx, dy, dz, ...].
"""

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

try:
    import sensor_msgs_py.point_cloud2 as pc2
    _PC2_OK = True
except ImportError:
    _PC2_OK = False

try:
    from sklearn.cluster import DBSCAN
    _SKLEARN_OK = True
except ImportError:
    _SKLEARN_OK = False


class AhncObstacleDetectorNode(Node):

    def __init__(self):
        super().__init__('anhc_obstacle_detector_node')

        self.declare_parameter('dbscan_eps', 0.5)
        self.declare_parameter('dbscan_min_samples', 5)
        self.declare_parameter('marker_lifetime_sec', 0.5)

        self._eps = self.get_parameter('dbscan_eps').value
        self._min_samples = int(self.get_parameter('dbscan_min_samples').value)
        self._lifetime = self.get_parameter('marker_lifetime_sec').value

        self._sub = self.create_subscription(
            PointCloud2, '/perception/cloud_obstacles', self._cb, 10)
        self._pub_markers = self.create_publisher(
            MarkerArray, '/perception/obstacles', 10)
        self._pub_list = self.create_publisher(
            Float32MultiArray, '/perception/obstacle_list', 10)

        if not _SKLEARN_OK:
            self.get_logger().warn(
                'scikit-learn not found — DBSCAN clustering disabled.')
        self.get_logger().info('anhc_obstacle_detector_node started')

    # ------------------------------------------------------------------
    def _cb(self, msg: PointCloud2) -> None:
        if not _PC2_OK or not _SKLEARN_OK:
            self._publish_empty()
            return

        try:
            gen = pc2.read_points(
                msg, field_names=('x', 'y', 'z'), skip_nans=True)
            pts = np.array(
                [(r[0], r[1], r[2]) for r in gen], dtype=np.float32)
        except Exception as exc:
            self.get_logger().error(f'Cloud parse error: {exc}')
            return

        if len(pts) < self._min_samples:
            self._publish_empty()
            return

        labels = DBSCAN(
            eps=self._eps, min_samples=self._min_samples
        ).fit(pts[:, :3]).labels_

        unique_labels = [lb for lb in set(labels) if lb != -1]

        ma = MarkerArray()
        # Delete-all sentinel so stale markers vanish
        del_m = Marker()
        del_m.action = Marker.DELETEALL
        ma.markers.append(del_m)

        flat: list[float] = []

        for cid in unique_labels:
            cluster = pts[labels == cid]
            centroid = cluster.mean(axis=0)
            dims = cluster.max(axis=0) - cluster.min(axis=0)

            m = Marker()
            m.header = msg.header
            m.ns = 'anhc_obstacles'
            m.id = int(cid) + 1          # id=0 reserved by DELETEALL
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = float(centroid[0])
            m.pose.position.y = float(centroid[1])
            m.pose.position.z = float(centroid[2])
            m.pose.orientation.w = 1.0
            m.scale.x = max(float(dims[0]), 0.1)
            m.scale.y = max(float(dims[1]), 0.1)
            m.scale.z = max(float(dims[2]), 0.1)
            m.color.r = 1.0
            m.color.g = 0.25
            m.color.b = 0.1
            m.color.a = 0.75
            m.lifetime = Duration(seconds=self._lifetime).to_msg()
            ma.markers.append(m)

            flat += [
                float(centroid[0]), float(centroid[1]), float(centroid[2]),
                float(dims[0]), float(dims[1]), float(dims[2]),
            ]

        self._pub_markers.publish(ma)
        list_msg = Float32MultiArray()
        list_msg.data = flat
        self._pub_list.publish(list_msg)

    def _publish_empty(self) -> None:
        ma = MarkerArray()
        dm = Marker()
        dm.action = Marker.DELETEALL
        ma.markers.append(dm)
        self._pub_markers.publish(ma)
        self._pub_list.publish(Float32MultiArray())


def main(args=None):
    rclpy.init(args=args)
    node = AhncObstacleDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
