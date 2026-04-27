#!/usr/bin/env python3
"""Relay /scan_gz -> /scan and normalize LaserScan header timing/frame.

Does not transform ranges or touch TF; only fixes header fields.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan

# Match GZ→ROS laser bridge (typically best-effort); default depth-10 reliable sub sees 0 Hz.
_PUB_SCAN_QOS = QoSProfile(
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

        self._pub = self.create_publisher(LaserScan, out_topic, _PUB_SCAN_QOS)
        self.create_subscription(LaserScan, in_topic, self._cb, qos_profile_sensor_data)

        self.get_logger().info(
            f"Relay {in_topic!r} -> {out_topic!r} frame_id={self._frame!r} (header only)"
        )

    def _cb(self, msg: LaserScan) -> None:
        now = self.get_clock().now()
        st = Time.from_msg(msg.header.stamp)
        # Gazebo bridge can emit zero stamp during startup.
        # Keep startup alive by stamping only that case; otherwise preserve original
        # sensor time to keep scan<->TF matching deterministic for slam_toolbox.
        if st.nanoseconds == 0:
            msg.header.stamp = now.to_msg()
        msg.header.frame_id = self._frame
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanFrameRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
