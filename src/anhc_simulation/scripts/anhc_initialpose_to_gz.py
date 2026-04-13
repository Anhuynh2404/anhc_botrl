#!/usr/bin/env python3
"""Apply RViz 2D Pose Estimate to the Gazebo model pose (teleport).

AMCL only updates ``map``→``odom``; it does not move the simulated robot. This node
calls ``ros_gz_interfaces/srv/SetEntityPose`` so the model pose matches the estimate.

``gz_world`` must match the ``<world name="...">`` in the loaded SDF (not the launch
file basename). For ``anhc_indoor.sdf`` that name is ``anhc_indoor_world``.
"""

from __future__ import annotations

import copy

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose


class InitialPoseToGz(Node):
    def __init__(self) -> None:
        super().__init__("anhc_initialpose_to_gz")
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("gz_world", "anhc_indoor_world")
        self.declare_parameter("model_name", "anhc_bot")
        self.declare_parameter("fixed_z", 0.2)
        self.declare_parameter("use_pose_z", False)

        world = self.get_parameter("gz_world").get_parameter_value().string_value
        self._model = self.get_parameter("model_name").get_parameter_value().string_value
        self._fixed_z = float(self.get_parameter("fixed_z").get_parameter_value().double_value)
        self._use_pose_z = (
            self.get_parameter("use_pose_z").get_parameter_value().bool_value
        )

        srv = f"/world/{world}/set_pose"
        self._client = self.create_client(SetEntityPose, srv)
        topic = self.get_parameter("initialpose_topic").get_parameter_value().string_value
        self.create_subscription(PoseWithCovarianceStamped, topic, self._pose_cb, 10)
        self._warn_sec = 0.0
        self.get_logger().info(
            f"Teleport model {self._model!r} via {srv!r} on {topic!r} (map frame = world)"
        )

    def _pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        if not self._client.service_is_ready():
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self._warn_sec > 5.0:
                self.get_logger().warn(
                    "Gazebo set_pose service not ready — skip teleport (will retry)"
                )
                self._warn_sec = now
            return

        req = SetEntityPose.Request()
        req.entity = Entity()
        req.entity.name = self._model
        req.entity.type = Entity.MODEL
        req.pose = copy.deepcopy(msg.pose.pose)
        if self._use_pose_z and abs(msg.pose.pose.position.z) > 1e-3:
            pass
        else:
            req.pose.position.z = float(self._fixed_z)

        fut = self._client.call_async(req)
        fut.add_done_callback(self._done)

    def _done(self, fut) -> None:
        try:
            ok = fut.result().success
        except Exception as e:
            self.get_logger().error(f"set_pose failed: {e}")
            return
        if ok:
            self.get_logger().info(
                f"Gazebo teleport OK for model {self._model!r}"
            )
        else:
            self.get_logger().warn("set_pose returned success=false")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InitialPoseToGz()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
