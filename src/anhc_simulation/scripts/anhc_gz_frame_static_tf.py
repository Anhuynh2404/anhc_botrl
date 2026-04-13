#!/usr/bin/env python3
"""Publish Gazebo sensor frame aliases on /tf_static (identity, stamp zero).

The C++ ``static_transform_publisher`` executable spins and republishes on /tf using
``rclcpp::Clock::now()`` as the transform stamp. Bridged ``/odom`` often carries a
slightly older ``header.stamp`` from the physics step. tf2 then sees a newer transform
followed by an older one and clears the whole buffer (``jump back in time``), which
breaks slam_toolbox and RViz.

Static extrinsics belong on ``/tf_static`` with zero time; they do not compete with
dynamic odometry timestamps.
"""

from __future__ import annotations

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def _identity(parent: str, child: str) -> TransformStamped:
    t = TransformStamped()
    t.header.stamp = Time(sec=0, nanosec=0)
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.rotation.w = 1.0
    return t


class GzFrameStaticBridges(Node):
    def __init__(self) -> None:
        super().__init__("anhc_gz_frame_static_bridges")
        broadcaster = StaticTransformBroadcaster(self)
        transforms = [
            _identity("lidar_link", "anhc_bot/base_footprint/anhc_lidar"),
            _identity("camera_link", "anhc_bot/base_footprint/anhc_rgb_camera"),
            _identity("camera_link", "anhc_bot/base_footprint/anhc_depth_camera"),
            _identity("imu_link", "anhc_bot/base_footprint/anhc_imu"),
        ]
        broadcaster.sendTransform(transforms)
        self.get_logger().info("Published %d identity transforms on /tf_static" % len(transforms))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GzFrameStaticBridges()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
