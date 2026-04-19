"""Spawn and move a simple box in Gazebo Sim for dynamic-obstacle benchmark scenarios.

Expects ``ros_gz_bridge`` ``parameter_bridge`` mappings for the active world::

    /world/<gz_world_name>/create@ros_gz_interfaces/srv/SpawnEntity
    /world/<gz_world_name>/remove@ros_gz_interfaces/srv/DeleteEntity
    /world/<gz_world_name>/set_pose@ros_gz_interfaces/srv/SetEntityPose

Commands (``std_msgs/String`` JSON on ``/benchmark/gz_obstacle_cmd``)::

    {"op":"spawn_moving","gz_world":"anhc_office_v2_world","model_name":"anhc_bm_dyn_x",
     "x":4.0,"y":1.0,"z":0.25,"yaw":0.0,"lx":0.35,"ly":0.35,"lz":0.5,"vx":0.0,"vy":-0.4}
    {"op":"despawn","gz_world":"anhc_office_v2_world","model_name":"anhc_bm_dyn_x"}
"""

from __future__ import annotations

import copy
import json
import math
import threading
from typing import Any, Optional

import rclpy
from rclpy.client import Client
from geometry_msgs.msg import Pose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity, EntityFactory
from ros_gz_interfaces.srv import DeleteEntity, SetEntityPose, SpawnEntity
from std_msgs.msg import String


def _yaw_to_pose(x: float, y: float, z: float, yaw: float) -> Pose:
    p = Pose()
    p.position.x = float(x)
    p.position.y = float(y)
    p.position.z = float(z)
    half = float(yaw) * 0.5
    p.orientation.z = math.sin(half)
    p.orientation.w = math.cos(half)
    return p


def _box_sdf(model_name: str, lx: float, ly: float, lz: float) -> str:
    return f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="{model_name}">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>2.0</mass>
        <inertia><ixx>0.02</ixx><iyy>0.02</iyy><izz>0.02</izz></inertia>
      </inertial>
      <collision name="c">
        <geometry><box><size>{lx} {ly} {lz}</size></box></geometry>
      </collision>
      <visual name="v">
        <geometry><box><size>{lx} {ly} {lz}</size></box></geometry>
        <material><diffuse>0.9 0.2 0.1 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""


class AnhcBenchmarkGzObstacleNode(Node):
    """Teleports a spawned box along vx/vy in the Gazebo world frame (≈ map in sim)."""

    def __init__(self) -> None:
        super().__init__("anhc_benchmark_gz_obstacle")
        self._grp = ReentrantCallbackGroup()
        self.create_subscription(
            String,
            "/benchmark/gz_obstacle_cmd",
            self._cmd_cb,
            10,
            callback_group=self._grp,
        )

        self._lock = threading.Lock()
        self._world: str = ""
        self._model: str = ""
        self._pose: list[float] = [0.0, 0.0, 0.25, 0.0]
        self._vel: list[float] = [0.0, 0.0]
        self._timer = None
        self._spawn_clients: dict[str, Client] = {}
        self._remove_clients: dict[str, Client] = {}
        self._pose_clients: dict[str, Client] = {}

        self.get_logger().info(
            "Listening on /benchmark/gz_obstacle_cmd (JSON spawn_moving | despawn)"
        )

    def _get_spawn_client(self, world: str) -> Client:
        if world not in self._spawn_clients:
            srv = f"/world/{world}/create"
            self._spawn_clients[world] = self.create_client(
                SpawnEntity, srv, callback_group=self._grp
            )
        return self._spawn_clients[world]

    def _get_remove_client(self, world: str) -> Client:
        if world not in self._remove_clients:
            srv = f"/world/{world}/remove"
            self._remove_clients[world] = self.create_client(
                DeleteEntity, srv, callback_group=self._grp
            )
        return self._remove_clients[world]

    def _get_set_pose_client(self, world: str) -> Client:
        if world not in self._pose_clients:
            srv = f"/world/{world}/set_pose"
            self._pose_clients[world] = self.create_client(
                SetEntityPose, srv, callback_group=self._grp
            )
        return self._pose_clients[world]

    def _cmd_cb(self, msg: String) -> None:
        try:
            data: dict[str, Any] = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("gz_obstacle_cmd: invalid JSON")
            return
        op = str(data.get("op", ""))
        if op == "spawn_moving":
            self._handle_spawn_moving(data)
        elif op == "despawn":
            self._handle_despawn(data)
        else:
            self.get_logger().warn(f"gz_obstacle_cmd: unknown op {op!r}")

    def _cancel_timer(self) -> None:
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

    def _handle_spawn_moving(self, data: dict[str, Any]) -> None:
        world = str(data.get("gz_world", ""))
        model = str(data.get("model_name", ""))
        if not world or not model:
            self.get_logger().warn("spawn_moving: need gz_world and model_name")
            return
        lx = float(data.get("lx", 0.35))
        ly = float(data.get("ly", 0.35))
        lz = float(data.get("lz", 0.5))
        x = float(data.get("x", 0.0))
        y = float(data.get("y", 0.0))
        z = float(data.get("z", 0.25))
        yaw = float(data.get("yaw", 0.0))
        vx = float(data.get("vx", 0.0))
        vy = float(data.get("vy", 0.0))

        with self._lock:
            self._cancel_timer()
            self._world = world
            self._model = model
            self._pose = [x, y, z, yaw]
            self._vel = [vx, vy]

        cli = self._get_spawn_client(world)
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"SpawnEntity service not available: {cli.srv_name}")
            return

        req = SpawnEntity.Request()
        req.entity_factory = EntityFactory()
        req.entity_factory.name = model
        req.entity_factory.allow_renaming = False
        req.entity_factory.sdf = _box_sdf(model, lx, ly, lz)
        req.entity_factory.sdf_filename = ""
        req.entity_factory.clone_name = ""
        req.entity_factory.pose = _yaw_to_pose(x, y, z, yaw)
        req.entity_factory.relative_to = ""

        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        try:
            ok = fut.result() is not None and fut.result().success
        except Exception as exc:
            self.get_logger().error(f"spawn failed: {exc}")
            return
        if not ok:
            self.get_logger().warn("SpawnEntity returned success=false")
            return

        self.get_logger().info(f"spawned moving obstacle {model!r} in {world!r}")
        self._timer = self.create_timer(0.05, self._tick, callback_group=self._grp)

    def _tick(self) -> None:
        with self._lock:
            world = self._world
            model = self._model
            dt = 0.05
            self._pose[0] += self._vel[0] * dt
            self._pose[1] += self._vel[1] * dt
            pose = _yaw_to_pose(self._pose[0], self._pose[1], self._pose[2], self._pose[3])

        cli = self._get_set_pose_client(world)
        if not cli.service_is_ready():
            return
        req = SetEntityPose.Request()
        req.entity = Entity()
        req.entity.name = model
        req.entity.type = Entity.MODEL
        req.pose = copy.deepcopy(pose)
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)

    def _handle_despawn(self, data: dict[str, Any]) -> None:
        world = str(data.get("gz_world", ""))
        model = str(data.get("model_name", ""))
        with self._lock:
            if not world:
                world = self._world
            if not model:
                model = self._model
            self._cancel_timer()

        if not world or not model:
            return
        cli = self._get_remove_client(world)
        if not cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(f"DeleteEntity service not ready: {cli.srv_name}")
            return
        req = DeleteEntity.Request()
        req.entity = Entity()
        req.entity.name = model
        req.entity.type = Entity.MODEL
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        try:
            ok = fut.result() is not None and fut.result().success
        except Exception as exc:
            self.get_logger().warn(f"despawn: {exc}")
            return
        if ok:
            self.get_logger().info(f"despawned {model!r}")
        else:
            self.get_logger().warn(f"DeleteEntity returned success=false for {model!r}")


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = AnhcBenchmarkGzObstacleNode()
    exe = MultiThreadedExecutor(num_threads=4)
    exe.add_node(node)
    try:
        exe.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
