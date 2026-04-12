#!/usr/bin/env python3
"""Republish Gazebo LaserScan on /scan with URDF frame_id (lidar_link).

Publishes odom -> base_footprint TF from /odom (Gazebo does not bridge gz /tf).

Uses the laser's own header.stamp from Gazebo (same clock as simulation). Do *not* re-stamp
with ApproximateTime-matched odom: wrong pairs misalign pose vs ranges and break SLAM/RViz.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

_SUB_SCAN_QOS = qos_profile_sensor_data
_ODOM_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=50,
)
_PUB_SCAN_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=50,
)


def _clone_scan(msg: LaserScan, frame_id: str) -> LaserScan:
    out = LaserScan()
    out.header.stamp = msg.header.stamp
    out.header.frame_id = frame_id
    out.angle_min = msg.angle_min
    out.angle_max = msg.angle_max
    out.angle_increment = msg.angle_increment
    out.time_increment = msg.time_increment
    out.scan_time = msg.scan_time
    out.range_min = msg.range_min
    out.range_max = msg.range_max
    out.ranges = list(msg.ranges)
    if msg.intensities:
        out.intensities = list(msg.intensities)
    return out


def _odom_to_tf(msg: Odometry) -> TransformStamped:
    t = TransformStamped()
    t.header = msg.header
    t.child_frame_id = msg.child_frame_id
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z
    t.transform.rotation = msg.pose.pose.orientation
    return t


class ScanFrameRelay(Node):
    def __init__(self) -> None:
        super().__init__("anhc_scan_frame_relay")
        self.declare_parameter("input_topic", "/scan_gz")
        self.declare_parameter("output_topic", "/scan")
        self.declare_parameter("output_frame_id", "lidar_link")
        self.declare_parameter("odom_topic", "/odom")

        in_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        out_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self._frame = self.get_parameter("output_frame_id").get_parameter_value().string_value
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value

        self._tf_broadcaster = TransformBroadcaster(self)
        self._pub = self.create_publisher(LaserScan, out_topic, _PUB_SCAN_QOS)

        self.create_subscription(LaserScan, in_topic, self._cb_scan, _SUB_SCAN_QOS)
        self.create_subscription(Odometry, odom_topic, self._cb_odom_tf, _ODOM_QOS)

        self.get_logger().info(
            f"Relay LaserScan {in_topic!r} -> {out_topic!r} frame_id={self._frame!r}; "
            f"TF from {odom_topic!r}"
        )

    def _cb_odom_tf(self, msg: Odometry) -> None:
        self._tf_broadcaster.sendTransform(_odom_to_tf(msg))

    def _cb_scan(self, msg: LaserScan) -> None:
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            return
        self._pub.publish(_clone_scan(msg, self._frame))


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
