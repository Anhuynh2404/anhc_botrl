#!/usr/bin/env python3
"""Camera processor node.

RGB pipeline  : Canny edge detection (OpenCV) → /perception/image_processed
Depth pipeline: min-distance check in centre ROI → /perception/depth_obstacles
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

try:
    import cv2
    from cv_bridge import CvBridge
    _CV_OK = True
except ImportError:
    _CV_OK = False


class AhncCameraProcessorNode(Node):

    def __init__(self):
        super().__init__('anhc_camera_processor_node')

        self.declare_parameter('canny_low', 50)
        self.declare_parameter('canny_high', 150)
        self.declare_parameter('depth_obstacle_threshold', 1.5)
        self.declare_parameter('blur_kernel', 5)

        self._lo = int(self.get_parameter('canny_low').value)
        self._hi = int(self.get_parameter('canny_high').value)
        self._depth_thr = float(
            self.get_parameter('depth_obstacle_threshold').value)
        k = int(self.get_parameter('blur_kernel').value)
        self._blur_k = k if k % 2 == 1 else k + 1

        if _CV_OK:
            self._bridge = CvBridge()
        else:
            self.get_logger().warn(
                'cv2/cv_bridge not found — camera processing disabled.')

        self._sub_rgb = self.create_subscription(
            Image, '/camera/image_raw', self._rgb_cb, 10)
        self._sub_depth = self.create_subscription(
            Image, '/camera/depth', self._depth_cb, 10)
        self._pub_img = self.create_publisher(
            Image, '/perception/image_processed', 10)
        self._pub_obs = self.create_publisher(
            Bool, '/perception/depth_obstacles', 10)

        self.get_logger().info('anhc_camera_processor_node started')

    # ------------------------------------------------------------------
    def _rgb_cb(self, msg: Image) -> None:
        if not _CV_OK:
            return
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(
                gray, (self._blur_k, self._blur_k), 0)
            edges = cv2.Canny(blurred, self._lo, self._hi)
            edge_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
            result = cv2.addWeighted(bgr, 0.7, edge_bgr, 0.3, 0)
            out = self._bridge.cv2_to_imgmsg(result, encoding='bgr8')
            out.header = msg.header
            self._pub_img.publish(out)
        except Exception as exc:
            self.get_logger().debug(f'RGB processing error: {exc}')

    def _depth_cb(self, msg: Image) -> None:
        if not _CV_OK:
            return
        try:
            depth = self._bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough').astype(np.float32)
            h, w = depth.shape[:2]
            cy0, cy1 = h // 4, 3 * h // 4
            cx0, cx1 = w // 4, 3 * w // 4
            roi = depth[cy0:cy1, cx0:cx1]
            valid = roi[np.isfinite(roi) & (roi > 0.05)]
            obstacle = bool(len(valid) > 0 and float(valid.min()) < self._depth_thr)
            self._pub_obs.publish(Bool(data=obstacle))
        except Exception as exc:
            self.get_logger().debug(f'Depth processing error: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = AhncCameraProcessorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
