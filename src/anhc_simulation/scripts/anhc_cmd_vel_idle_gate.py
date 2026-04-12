#!/usr/bin/env python3
"""Forward /cmd_vel to the gz bridge while publishing zeros when idle.

Gazebo Sim 8 DiffDrive keeps the last Twist forever — it does not implement
cmd_vel_timeout (the XML tag is ignored). Any stale non-zero command causes
continuous motion. This node re-sends zero Twist while no command arrives for
idle_sec (sim mapping with no teleop stays stopped).
"""

from __future__ import annotations

from copy import copy

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

# Reliable + depth for ros_gz_bridge parameter_bridge (default ROS subscriber QoS is KeepLast(10)).
_CMD_VEL_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=50,
)


class CmdVelIdleGate(Node):
    def __init__(self) -> None:
        super().__init__("anhc_cmd_vel_idle_gate")
        self.declare_parameter("cmd_vel_in", "/cmd_vel")
        self.declare_parameter("cmd_vel_out", "/cmd_vel_gz")
        # Keyboard teleop (Jazzy teleop_twist_keyboard) often has ~300–500 ms before key-repeat;
        # shorter idle windows zero out gz cmd_vel between repeats → robot appears not to move.
        self.declare_parameter("idle_sec", 0.65)
        self.declare_parameter("tick_hz", 20.0)

        in_topic = self.get_parameter("cmd_vel_in").get_parameter_value().string_value
        out_topic = self.get_parameter("cmd_vel_out").get_parameter_value().string_value
        idle_s = self.get_parameter("idle_sec").get_parameter_value().double_value
        self._idle = Duration(seconds=idle_s)
        tick_hz = self.get_parameter("tick_hz").get_parameter_value().double_value
        tick = 1.0 / max(1.0, tick_hz)

        # Never do (sim_now - large_duration): sim time can start near 0 → negative Time → crash.
        # None means "no /cmd_vel yet" → treat as idle and publish zeros until first command.
        self._last_cmd_time: Time | None = None
        self._last_twist = Twist()
        self._pub = self.create_publisher(Twist, out_topic, _CMD_VEL_QOS)
        self.create_subscription(Twist, in_topic, self._cb, _CMD_VEL_QOS)
        self.create_timer(tick, self._tick)

        self.get_logger().info(f"Gate {in_topic!r} -> {out_topic!r} idle={idle_s}s")

    def _cb(self, msg: Twist) -> None:
        self._last_cmd_time = self.get_clock().now()
        self._last_twist = copy(msg)
        self._pub.publish(msg)

    def _tick(self) -> None:
        now = self.get_clock().now()
        if self._last_cmd_time is None or now - self._last_cmd_time > self._idle:
            self._pub.publish(Twist())
        else:
            # Re-publish last command at tick_hz so a single keypress still feeds the bridge
            # until idle_sec elapses (Jazzy teleop_twist_keyboard does not repeat Twist on its own).
            self._pub.publish(copy(self._last_twist))


def main() -> None:
    rclpy.init()
    node = CmdVelIdleGate()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
