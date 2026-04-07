#!/usr/bin/env python3
"""Point cloud processor node.

Pipeline:
  /point_cloud (PointCloud2)
    → passthrough filter (z clamp)
    → voxel-grid downsampling (numpy, no open3d)
    → /perception/cloud_filtered   (all surviving points)
    → /perception/cloud_obstacles  (z in obstacle band)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

try:
    import sensor_msgs_py.point_cloud2 as pc2
    _PC2_OK = True
except ImportError:
    _PC2_OK = False


class AhncPointCloudProcessorNode(Node):

    def __init__(self):
        super().__init__('anhc_pointcloud_processor_node')

        self.declare_parameter('voxel_leaf_size', 0.05)
        self.declare_parameter('passthrough_min_z', -0.1)
        self.declare_parameter('passthrough_max_z', 2.0)
        self.declare_parameter('obstacle_min_z', 0.1)
        self.declare_parameter('obstacle_max_z', 1.5)

        self._leaf = self.get_parameter('voxel_leaf_size').value
        self._pmin = self.get_parameter('passthrough_min_z').value
        self._pmax = self.get_parameter('passthrough_max_z').value
        self._omin = self.get_parameter('obstacle_min_z').value
        self._omax = self.get_parameter('obstacle_max_z').value

        self._sub = self.create_subscription(
            PointCloud2, '/point_cloud', self._cb, 10)
        self._pub_filt = self.create_publisher(
            PointCloud2, '/perception/cloud_filtered', 10)
        self._pub_obs = self.create_publisher(
            PointCloud2, '/perception/cloud_obstacles', 10)

        if not _PC2_OK:
            self.get_logger().warn(
                'sensor_msgs_py not found — point cloud processing disabled.')
        self.get_logger().info('anhc_pointcloud_processor_node started')

    # ------------------------------------------------------------------
    def _voxel_downsample(self, pts: np.ndarray) -> np.ndarray:
        """Voxel grid centroid downsampling (pure numpy)."""
        if len(pts) == 0:
            return pts
        leaf = self._leaf
        min_b = pts.min(axis=0)
        vi = ((pts - min_b) / leaf).astype(np.int32)
        max_i = vi.max(axis=0) + 1
        flat = (vi[:, 0].astype(np.int64) * int(max_i[1]) * int(max_i[2])
                + vi[:, 1].astype(np.int64) * int(max_i[2])
                + vi[:, 2].astype(np.int64))
        order = np.argsort(flat)
        sflat = flat[order]
        spts = pts[order]
        # Boundaries between voxels
        mask = np.concatenate([[True], sflat[1:] != sflat[:-1]])
        starts = np.where(mask)[0]
        ends = np.append(starts[1:], len(spts))
        centroids = np.array([spts[s:e].mean(axis=0)
                               for s, e in zip(starts, ends)], dtype=np.float32)
        return centroids

    def _cb(self, msg: PointCloud2) -> None:
        if not _PC2_OK:
            return
        try:
            gen = pc2.read_points(
                msg, field_names=('x', 'y', 'z'), skip_nans=True)
            pts = np.array(
                [(r[0], r[1], r[2]) for r in gen], dtype=np.float32)
        except Exception as exc:
            self.get_logger().error(f'PointCloud2 parse error: {exc}')
            return

        if pts.size == 0:
            return

        # Passthrough filter
        mask = (pts[:, 2] >= self._pmin) & (pts[:, 2] <= self._pmax)
        pts = pts[mask]
        if len(pts) == 0:
            return

        # Voxel downsampling
        if len(pts) > 200:
            pts = self._voxel_downsample(pts)

        hdr = Header(frame_id=msg.header.frame_id, stamp=msg.header.stamp)

        # Publish filtered cloud
        self._pub_filt.publish(
            pc2.create_cloud_xyz32(hdr, pts.tolist()))

        # Obstacle band
        obs_mask = (pts[:, 2] >= self._omin) & (pts[:, 2] <= self._omax)
        obs = pts[obs_mask]
        if len(obs) > 0:
            self._pub_obs.publish(
                pc2.create_cloud_xyz32(hdr, obs.tolist()))


def main(args=None):
    rclpy.init(args=args)
    node = AhncPointCloudProcessorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
