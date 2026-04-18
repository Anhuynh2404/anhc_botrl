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
    SetLaunchConfiguration,
    TimerAction,
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


_AUTO = "__auto__"

# Maps world argument → (gz_world_name, map YAML basename).
# The gz_world_name must match <world name="..."> in the corresponding SDF file.
_WORLD_DEFAULTS: dict[str, tuple[str, str]] = {
    "anhc_indoor": ("anhc_indoor_world", "anhc_indoor_map.yaml"),
    "anhc_office_v2": ("anhc_office_v2_world", ""),
    "anhc_office": ("anhc_office_world", "anhc_office_map.yaml"),
}


def _apply_office_v2_shorthand(context):
    """When use_office_v2:=true, override world routing only.

    Do not force use_slam here: callers may explicitly run with use_slam:=false
    and a saved map_file for localization / navigation.
    """
    if context.perform_substitution(LaunchConfiguration("use_office_v2")) != "true":
        return []
    return [
        SetLaunchConfiguration("world", "anhc_office_v2"),
        SetLaunchConfiguration("gz_world_name", "anhc_office_v2_world"),
    ]


def _apply_world_defaults(context):
    """Derive gz_world_name and map_file from 'world' when still set to sentinel '__auto__'.

    Runs after ``_apply_office_v2_shorthand`` so explicit overrides from that
    function (or from the command line) are never clobbered.
    """
    world = context.perform_substitution(LaunchConfiguration("world"))
    gz_world_name = context.perform_substitution(LaunchConfiguration("gz_world_name"))
    map_file = context.perform_substitution(LaunchConfiguration("map_file"))

    share = get_package_share_directory("anhc_simulation")
    gz_name_default, map_basename_default = _WORLD_DEFAULTS.get(
        world, (f"{world}_world", f"{world}_map.yaml")
    )
    actions = []

    if gz_world_name == _AUTO:
        actions.append(SetLaunchConfiguration("gz_world_name", gz_name_default))

    if map_file == _AUTO and map_basename_default:
        actions.append(
            SetLaunchConfiguration(
                "map_file", os.path.join(share, "maps", map_basename_default)
            )
        )

    return actions


def _localization_and_lifecycle(context):
    """Localization (slam_toolbox or AMCL) + lifecycle_manager for Nav2 lifecycle nodes.

    ``map_server`` and ``amcl`` are Nav2 lifecycle nodes: without configure/activate they
    never publish (AMCL would not broadcast ``map``→``odom``, breaking Fixed Frame ``map``).
    ``map_server`` is launched before this function; order here is localization node(s)
    then ``lifecycle_manager`` so ``amcl`` exists when autostart runs.
    """
    use_slam = context.perform_substitution(LaunchConfiguration("use_slam")) == "true"
    if use_slam:
        share_loc = get_package_share_directory("anhc_localization")
        # Online new-map SLAM (e.g. office_v2): mapping mode + /scan. Localization yaml is for
        # serialized slam_toolbox maps in the use_slam:=false branch.
        slam_params = os.path.join(share_loc, "config", "anhc_slam_mapping.yaml")
        slam_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py")]
            ),
            launch_arguments={
                "slam_params_file": slam_params,
                "use_sim_time": "true",
                "autostart": "true",
            }.items(),
        )
        # Start slam_toolbox after /clock is flowing (see log: lifecycle activate must not
        # beat the ros_gz bridge’s first /clock subscription or tf2 buffers thrash).
        return [
            LogInfo(msg="[anhc_nav] Localization: slam_toolbox online SLAM (new map)."),
            TimerAction(period=1.5, actions=[slam_launch]),
        ]

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
                "use_office_v2",
                default_value="false",
                description=(
                    "Shorthand: when true, sets world:=anhc_office_v2, "
                    "use_slam:=true, and gz_world_name:=anhc_office_v2_world."
                ),
            ),
            DeclareLaunchArgument(
                "use_slam",
                default_value="false",
                description=(
                    "If true, run slam_toolbox online SLAM and do not start map_server/AMCL. "
                    "Use this for worlds without a pre-scanned map."
                ),
            ),
            DeclareLaunchArgument(
                "map_file",
                default_value=_AUTO,
                description=(
                    "Absolute path to map YAML for map_server. "
                    "Used when use_slam:=false; defaults to the map matching the 'world' argument."
                ),
            ),
            DeclareLaunchArgument(
                "algorithm",
                default_value="astar",
                description=(
                    "Global planner: astar, dijkstra, greedy_bfs, theta_star, jps, "
                    "rrt_star, dstar_lite, prm, rl"
                ),
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
                default_value=_AUTO,
                description=(
                    "Must match <world name=\"...\"> in the SDF (for set_pose service). "
                    "Defaults to the world name matching the 'world' argument."
                ),
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
            OpaqueFunction(function=_apply_office_v2_shorthand),
            OpaqueFunction(function=_apply_world_defaults),
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
                condition=IfCondition(NotSubstitution(LaunchConfiguration("use_slam"))),
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
