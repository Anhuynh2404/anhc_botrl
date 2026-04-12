#!/usr/bin/env python3
"""Publish odom -> base_footprint TF from bridged /odom.

Default stamp is receive-time sim clock so this TF stays consistent with
joint_state_publisher / robot_state_publisher (avoids tf2 'jump back' vs Gazebo odom
headers). Set parameter ``use_odometry_msg_stamp`` true to match ``Odometry.header``
(e.g. bag replay).
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from tf2_ros import TransformBroadcaster

_ODOM_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=50,
)


class OdomTFPublisher(Node):
    def __init__(self) -> None:
        super().__init__("anhc_odom_tf_publisher")
        self.declare_parameter("odom_topic", "/odom")
        # True: TF time matches Odometry pose (required for slam_toolbox / message_filters).
        # False: re-stamp with receive-time sim clock — fixes rare jsp vs odom ordering but
        # assigns stale poses to new times and breaks scan↔TF consistency (inflated map, bad loops).
        self.declare_parameter("use_odometry_msg_stamp", True)

        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self._use_odom_stamp = (
            self.get_parameter("use_odometry_msg_stamp").get_parameter_value().bool_value
        )
        self._tf_broadcaster = TransformBroadcaster(self)
        self._last_tf_stamp: Time | None = None
        self.create_subscription(Odometry, odom_topic, self._odom_cb, _ODOM_QOS)
        src = "Odometry.header.stamp" if self._use_odom_stamp else "receive-time sim clock"
        self.get_logger().info(f"Publishing TF odom->base_footprint from {odom_topic!r} ({src})")

    def _odom_cb(self, msg: Odometry) -> None:
        if self._use_odom_stamp:
            stamp = Time.from_msg(msg.header.stamp)
        else:
            stamp = self.get_clock().now()
        if self._last_tf_stamp is not None and stamp < self._last_tf_stamp:
            return
        self._last_tf_stamp = stamp
        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._tf_broadcaster.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomTFPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
