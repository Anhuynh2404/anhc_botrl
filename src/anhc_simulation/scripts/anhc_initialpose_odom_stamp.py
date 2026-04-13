#!/usr/bin/env python3
"""Bridge RViz /initialpose → AMCL with odom-aligned header stamp.

RViz stamps the pose with sim ``now()``; bridged ``/odom`` (and thus odom→base TF) can
lag by a tick. AMCL then looks up base_footprint→odom at the pose stamp and tf2 warns
\"extrapolation into the future\". Re-stamping with the latest ``/odom`` message time
keeps that lookup inside published TF data.
"""

from __future__ import annotations

import copy

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time


class InitialPoseOdomStamp(Node):
    def __init__(self) -> None:
        super().__init__("anhc_initialpose_odom_stamp")
        self.declare_parameter("input_topic", "/initialpose")
        self.declare_parameter("output_topic", "/initialpose_synced")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("stamp_backtrack_ns", 45_000_000)

        in_t = self.get_parameter("input_topic").get_parameter_value().string_value
        out_t = self.get_parameter("output_topic").get_parameter_value().string_value
        odom_t = self.get_parameter("odom_topic").get_parameter_value().string_value
        self._back_ns = int(
            self.get_parameter("stamp_backtrack_ns").get_parameter_value().integer_value
        )

        self._last_odom_stamp = None
        self._pub = self.create_publisher(PoseWithCovarianceStamped, out_t, 10)
        self.create_subscription(Odometry, odom_t, self._odom_cb, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, in_t, self._pose_cb, 10)
        self.get_logger().info(
            f"Republish {in_t!r} → {out_t!r} with stamp from latest {odom_t!r}"
        )

    def _odom_cb(self, msg: Odometry) -> None:
        self._last_odom_stamp = msg.header.stamp

    def _pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        out = copy.deepcopy(msg)
        if self._last_odom_stamp is not None:
            t = Time.from_msg(self._last_odom_stamp) - Duration(
                nanoseconds=max(0, self._back_ns)
            )
            out.header.stamp = t.to_msg()
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InitialPoseOdomStamp()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
