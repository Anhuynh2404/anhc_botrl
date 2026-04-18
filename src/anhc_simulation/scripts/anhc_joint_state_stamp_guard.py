#!/usr/bin/env python3
"""Republish /joint_states with strictly non-decreasing header.stamp (GZ time).

Gazebo Model→JointState can reorder messages; robot_state_publisher warns and tf2
buffers clear. Stamps are min-changed from the incoming message.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState

_IN_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=50,
)
_OUT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=50,
)


class JointStateStampGuard(Node):
    def __init__(self) -> None:
        super().__init__("anhc_joint_state_stamp_guard")
        self.declare_parameter("input_topic", "/joint_states_raw")
        self.declare_parameter("output_topic", "/joint_states")
        in_t = self.get_parameter("input_topic").get_parameter_value().string_value
        out_t = self.get_parameter("output_topic").get_parameter_value().string_value
        self._last_ns: int = -1
        self._pub = self.create_publisher(JointState, out_t, _OUT_QOS)
        self.create_subscription(JointState, in_t, self._cb, _IN_QOS)
        self.get_logger().info(f"JointState stamp guard {in_t!r} -> {out_t!r}")

    def _cb(self, msg: JointState) -> None:
        st_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if st_ns <= self._last_ns:
            st_ns = self._last_ns + 1
        self._last_ns = st_ns
        sec, nsec = divmod(st_ns, 1_000_000_000)
        msg.header.stamp.sec = int(sec)
        msg.header.stamp.nanosec = int(nsec)
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointStateStampGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
