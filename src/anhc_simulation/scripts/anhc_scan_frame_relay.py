#!/usr/bin/env python3
"""Republish Gazebo LaserScan on /scan with URDF frame_id (lidar_link).

Gazebo gpu_lidar keeps a scoped frame name; message_filters in slam_toolbox / RViz
often overflow when resolving that frame. Rewriting the header avoids the extra TF hop
for scan only (point_cloud etc. still use static TF bridges).
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan

# Subscribe like a sensor subscriber (Gazebo→ROS laser is usually best-effort).
_SUB_QOS = qos_profile_sensor_data

# Downstream (slam_toolbox, RViz) typically expects a deeper reliable queue.
_PUB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=50,
)


class ScanFrameRelay(Node):
    def __init__(self) -> None:
        super().__init__("anhc_scan_frame_relay")
        self.declare_parameter("input_topic", "/scan_gz")
        self.declare_parameter("output_topic", "/scan")
        self.declare_parameter("output_frame_id", "lidar_link")

        in_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        out_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self._frame = self.get_parameter("output_frame_id").get_parameter_value().string_value

        self._pub = self.create_publisher(LaserScan, out_topic, _PUB_QOS)
        self.create_subscription(LaserScan, in_topic, self._cb, _SUB_QOS)
        self.get_logger().info(
            f"Relay LaserScan {in_topic!r} -> {out_topic!r} frame_id={self._frame!r}"
        )

    def _cb(self, msg: LaserScan) -> None:
        msg.header.frame_id = self._frame
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = ScanFrameRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
