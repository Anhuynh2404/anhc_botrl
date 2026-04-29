#!/usr/bin/env python3
"""Relay /scan_gz -> /scan and normalize LaserScan header timing/frame.

Does not transform ranges or touch TF; only fixes header fields.
"""

from __future__ import annotations

import math

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
        self.declare_parameter("min_valid_range", 0.22)
        self.declare_parameter("self_filter_enabled", True)
        self.declare_parameter("robot_half_length", 0.40)
        self.declare_parameter("robot_half_width", 0.40)
        self.declare_parameter("self_filter_margin", 0.03)
        self.declare_parameter("lidar_offset_x", 0.10)
        self.declare_parameter("lidar_offset_y", 0.00)

        in_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        out_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self._frame = self.get_parameter("output_frame_id").get_parameter_value().string_value
        self._min_valid_range = (
            self.get_parameter("min_valid_range").get_parameter_value().double_value
        )
        self._self_filter_enabled = (
            self.get_parameter("self_filter_enabled").get_parameter_value().bool_value
        )
        self._half_len = self.get_parameter("robot_half_length").get_parameter_value().double_value
        self._half_wid = self.get_parameter("robot_half_width").get_parameter_value().double_value
        self._self_margin = (
            self.get_parameter("self_filter_margin").get_parameter_value().double_value
        )
        self._lidar_off_x = self.get_parameter("lidar_offset_x").get_parameter_value().double_value
        self._lidar_off_y = self.get_parameter("lidar_offset_y").get_parameter_value().double_value

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
        # Filter near-field returns that often come from robot body / self-occlusion and
        # paint small occupied speckles ("black traces") while teleop mapping.
        out = []
        angle = msg.angle_min
        for r in msg.ranges:
            keep = math.isfinite(r) and r >= self._min_valid_range
            if keep and self._self_filter_enabled:
                # Convert measured endpoint from lidar frame to base_footprint frame
                # using static lidar offset. Reject points inside robot footprint.
                x_l = r * math.cos(angle)
                y_l = r * math.sin(angle)
                x_b = x_l + self._lidar_off_x
                y_b = y_l + self._lidar_off_y
                in_body = (
                    abs(x_b) <= (self._half_len + self._self_margin)
                    and abs(y_b) <= (self._half_wid + self._self_margin)
                )
                if in_body:
                    keep = False
            out.append(r if keep else float("inf"))
            angle += msg.angle_increment
        msg.ranges = out
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
