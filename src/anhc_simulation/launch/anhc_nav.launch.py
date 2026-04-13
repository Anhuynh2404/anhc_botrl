"""Navigation launch: Gazebo sim + saved map + global planner + path follower + RViz2.

Stack order (high level):
  1) :py:mod:`anhc_sim.launch` — Gazebo, ``robot_state_publisher``, gz bridge,
     odom TF, scan relay, spawn, static GZ→URDF frames (nodes 1–4 in the spec).
  2) IMU filter + EKF → ``/odometry/filtered`` (required by the path follower).
  3) ``map_server`` + lifecycle manager.
  4) ``anhc_global_costmap_node`` → ``/costmap/global``.
  5) Localization: ``slam_toolbox`` *localization* if ``<map>.posegraph`` and
     ``<map>.data`` exist beside the map YAML; otherwise ``nav2_amcl`` (typical
     for ``.pgm`` / ``.yaml`` maps).
  6) ``anhc_global_planner`` + ``anhc_path_follower``.
  7) RViz2 (optional) with ``anhc_nav.rviz`` (2D Goal Pose → ``/goal_pose``).
"""

from __future__ import annotations

import os
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def _localization_and_lifecycle(context):
    """Localization (slam_toolbox or AMCL) + lifecycle_manager for Nav2 lifecycle nodes.

    ``map_server`` and ``amcl`` are Nav2 lifecycle nodes: without configure/activate they
    never publish (AMCL would not broadcast ``map``→``odom``, breaking Fixed Frame ``map``).
    ``map_server`` is launched before this function; order here is localization node(s)
    then ``lifecycle_manager`` so ``amcl`` exists when autostart runs.
    """
    raw_map = LaunchConfiguration("map_file")
    map_path = Path(os.path.expanduser(context.perform_substitution(raw_map))).resolve()
    if map_path.suffix.lower() in (".yaml", ".yml"):
        serial_base = map_path.with_suffix("")
    else:
        serial_base = map_path

    pg = Path(str(serial_base) + ".posegraph")
    dat = Path(str(serial_base) + ".data")
    serial_ok = pg.is_file() and dat.is_file()

    share_loc = get_package_share_directory("anhc_localization")
    slam_params = os.path.join(share_loc, "config", "anhc_slam_localization.yaml")
    amcl_params = os.path.join(share_loc, "config", "anhc_amcl_nav.yaml")

    if serial_ok:
        with open(slam_params, encoding="utf-8") as f:
            slam_cfg = yaml.safe_load(f)
        st = slam_cfg.setdefault("slam_toolbox", {}).setdefault("ros__parameters", {})
        st["map_file_name"] = str(serial_base)
        fd, merged_slam = tempfile.mkstemp(prefix="anhc_slam_loc_", suffix=".yaml")
        with os.fdopen(fd, "w", encoding="utf-8") as tmp:
            yaml.safe_dump(slam_cfg, tmp, sort_keys=False)

        slam_node = LifecycleNode(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[
                merged_slam,
                {
                    "use_sim_time": True,
                    "use_lifecycle_manager": False,
                },
            ],
        )
        configure = EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=matches_action(slam_node),
                transition_id=Transition.TRANSITION_CONFIGURE,
            ),
            condition=IfCondition(
                AndSubstitution(
                    LaunchConfiguration("slam_autostart"),
                    NotSubstitution(LaunchConfiguration("slam_use_lifecycle_manager")),
                )
            ),
        )
        activate = RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=slam_node,
                start_state="configuring",
                goal_state="inactive",
                entities=[
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_action(slam_node),
                            transition_id=Transition.TRANSITION_ACTIVATE,
                        )
                    )
                ],
            ),
            condition=IfCondition(
                AndSubstitution(
                    LaunchConfiguration("slam_autostart"),
                    NotSubstitution(LaunchConfiguration("slam_use_lifecycle_manager")),
                )
            ),
        )
        lm = Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_map",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "autostart": True,
                    "node_names": ["map_server"],
                }
            ],
        )
        return [
            LogInfo(
                msg=(
                    "[anhc_nav] Localization: slam_toolbox (serialized map at "
                    f"{serial_base})"
                )
            ),
            slam_node,
            configure,
            activate,
            lm,
        ]

    initial_pose_sync = Node(
        package="anhc_simulation",
        executable="anhc_initialpose_odom_stamp.py",
        name="anhc_initialpose_odom_stamp",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[amcl_params, {"use_sim_time": True}],
        remappings=[("initialpose", "initialpose_synced")],
    )
    lm = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "node_names": ["map_server", "amcl"],
            }
        ],
    )
    return [
        LogInfo(
            msg=(
                "[anhc_nav] Localization: nav2_amcl (no slam_toolbox "
                f".posegraph/.data next to {map_path}; place serial maps or use "
                "this stack as-is)."
            )
        ),
        initial_pose_sync,
        amcl_node,
        lm,
    ]


def _gz_set_pose_service_bridge(context):
    """Expose Gazebo ``/world/<name>/set_pose`` on ROS so ``anhc_initialpose_to_gz`` works."""
    world = context.perform_substitution(LaunchConfiguration("gz_world_name"))
    arg = f"/world/{world}/set_pose@ros_gz_interfaces/srv/SetEntityPose"
    return [
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_set_pose_bridge",
            arguments=[arg],
            parameters=[{"use_sim_time": True}],
            output="screen",
        )
    ]


def generate_launch_description() -> LaunchDescription:
    default_map = PathJoinSubstitution(
        [FindPackageShare("anhc_simulation"), "maps", "anhc_indoor_map.yaml"]
    )
    planner_params = PathJoinSubstitution(
        [FindPackageShare("anhc_planning"), "config", "planner_params.yaml"]
    )
    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("anhc_viz"), "rviz", "anhc_nav.rviz"]
    )
    ekf_params = PathJoinSubstitution(
        [FindPackageShare("anhc_localization"), "config", "ekf_params.yaml"]
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("anhc_simulation"), "launch", "anhc_sim.launch.py"]
                )
            ]
        ),
        launch_arguments={
            "sim_use_rviz": "false",
            "gz_extra_args": LaunchConfiguration("gz_extra_args"),
            "world": LaunchConfiguration("world"),
        }.items(),
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4"),
            DeclareLaunchArgument(
                "map_file",
                default_value=default_map,
                description="Absolute path to map YAML for map_server.",
            ),
            DeclareLaunchArgument(
                "algorithm",
                default_value="astar",
                description="Global planner algorithm: astar | dijkstra.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz2 with anhc_nav.rviz.",
            ),
            DeclareLaunchArgument(
                "world",
                default_value="anhc_indoor",
                description="Gazebo world name (without .sdf).",
            ),
            DeclareLaunchArgument(
                "gz_world_name",
                default_value="anhc_indoor_world",
                description="Must match <world name=\"...\"> in the SDF (for set_pose service).",
            ),
            DeclareLaunchArgument(
                "gz_extra_args",
                default_value="",
                description="Extra gz sim args (empty = GUI; use -s for headless).",
            ),
            DeclareLaunchArgument(
                "slam_autostart",
                default_value="true",
                description="Auto configure/activate slam_toolbox lifecycle (if used).",
            ),
            DeclareLaunchArgument(
                "slam_use_lifecycle_manager",
                default_value="false",
                description="Must stay false for bundled slam_toolbox launch pattern.",
            ),
            sim_launch,
            OpaqueFunction(function=_gz_set_pose_service_bridge),
            Node(
                package="anhc_simulation",
                executable="anhc_initialpose_to_gz.py",
                name="anhc_initialpose_to_gz",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    {
                        "gz_world": ParameterValue(
                            LaunchConfiguration("gz_world_name"), value_type=str
                        )
                    },
                    {"model_name": "anhc_bot"},
                    {"fixed_z": 0.2},
                ],
            ),
            Node(
                package="anhc_localization",
                executable="anhc_imu_filter_node.py",
                name="anhc_imu_filter_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "alpha": 0.98,
                        "accel_lpf_alpha": 0.8,
                    }
                ],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[ekf_params],
                remappings=[
                    ("odometry/filtered", "/odometry/filtered"),
                ],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "yaml_filename": LaunchConfiguration("map_file"),
                    }
                ],
            ),
            OpaqueFunction(function=_localization_and_lifecycle),
            Node(
                package="anhc_mapping",
                executable="anhc_global_costmap_node.py",
                name="anhc_global_costmap_node",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="anhc_planning",
                executable="anhc_global_planner_node",
                name="anhc_global_planner",
                output="screen",
                parameters=[
                    planner_params,
                    {
                        "use_sim_time": True,
                        "algorithm": LaunchConfiguration("algorithm"),
                    },
                ],
            ),
            Node(
                package="anhc_planning",
                executable="anhc_path_follower_node",
                name="anhc_path_follower",
                output="screen",
                parameters=[planner_params, {"use_sim_time": True}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_cfg],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("use_rviz")),
                output="screen",
            ),
        ]
    )
