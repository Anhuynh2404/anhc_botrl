#!/usr/bin/env python3
"""Publish odom -> base_link TF from bridged /odom.

Gazebo diff-drive publishes TF only on gz transport; ros_gz_bridge does not forward
/tf (see bridge_params.yaml). SLAM and RViz need odom->base in the ROS tf2 buffer.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):
    def __init__(self) -> None:
        super().__init__("anhc_odom_tf_broadcaster")
        self.declare_parameter("odom_topic", "/odom")
        topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self._tf_broadcaster = TransformBroadcaster(self)
        # Match ros_gz_bridge (reliable); sensor_data QoS would not match.
        self.create_subscription(Odometry, topic, self._cb, 50)
        self.get_logger().info(f"Publishing TF from Odometry topic {topic!r}")

    def _cb(self, msg: Odometry) -> None:
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._tf_broadcaster.sendTransform(t)


def main() -> None:
    rclpy.init()
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
