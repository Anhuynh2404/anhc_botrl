"""Master launch file for the anhc autonomous vehicle system.

Brings up the complete stack in order:
  1. Simulation (Gazebo + robot_state_publisher + bridge + spawn)
  2. Perception pipeline (localization → perception → mapping)
  3. Planning stack (global planner + path follower)
  4. [optional] RViz2 with the anhc_full_system.rviz configuration
  5. [optional] Benchmark framework
  6. Live terminal dashboard (always)

Usage examples
--------------
  # Full GUI with A* (default)
  ros2 launch anhc_simulation anhc_master.launch.py

  # Headless with Dijkstra, run benchmark
  ros2 launch anhc_simulation anhc_master.launch.py \\
    algorithm:=dijkstra use_rviz:=false run_benchmark:=true gz_extra_args:=-s

  # Custom world
  ros2 launch anhc_simulation anhc_master.launch.py world:=anhc_outdoor
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    # ── arguments ──────────────────────────────────────────────────────────────
    algorithm_arg = DeclareLaunchArgument(
        "algorithm",
        default_value="astar",
        description="Path planning algorithm. Options: astar, dijkstra, rrt_star, dstar_lite, rl",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz2 with the anhc_full_system configuration.",
    )
    run_benchmark_arg = DeclareLaunchArgument(
        "run_benchmark",
        default_value="false",
        description="Include the benchmark framework (runner + live_metrics + analyzer).",
    )
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="anhc_indoor",
        description="Gazebo world name (without .sdf extension).",
    )
    gz_extra_args_arg = DeclareLaunchArgument(
        "gz_extra_args",
        default_value="",
        description="Extra arguments forwarded to gz sim. Use '-s' for headless mode.",
    )
    scenario_file_arg = DeclareLaunchArgument(
        "scenario_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("anhc_benchmark"),
            "config",
            "anhc_benchmark_scenarios.yaml",
        ]),
        description="Benchmark scenario YAML (used only when run_benchmark:=true).",
    )

    # ── helper ─────────────────────────────────────────────────────────────────
    def _inc(pkg: str, launch_file: str, extra_args: dict | None = None, condition=None):
        kwargs = {}
        if condition is not None:
            kwargs["condition"] = condition
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare(pkg), "launch", launch_file])
            ]),
            launch_arguments=(extra_args or {}).items(),
            **kwargs,
        )

    # ── 1. simulation ──────────────────────────────────────────────────────────
    sim_launch = _inc(
        "anhc_simulation",
        "anhc_sim.launch.py",
        {
            "world": LaunchConfiguration("world"),
            "gz_extra_args": LaunchConfiguration("gz_extra_args"),
            # Pass "sim_use_rviz" (not "use_rviz") to avoid overriding the master's
            # own "use_rviz" flag in the shared LaunchConfiguration namespace.
            "sim_use_rviz": "false",
        },
    )

    # ── 2. perception (localization + sensors + mapping) ───────────────────────
    perception_launch = _inc("anhc_perception", "anhc_perception_full.launch.py")

    # ── 3. planning ────────────────────────────────────────────────────────────
    planning_launch = _inc(
        "anhc_planning",
        "anhc_planning.launch.py",
        {"algorithm": LaunchConfiguration("algorithm")},
    )

    # ── 4. RViz2 (conditional) ─────────────────────────────────────────────────
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("anhc_viz"), "rviz", "anhc_full_system.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        output="screen",
    )

    # ── 5. benchmark (conditional) ─────────────────────────────────────────────
    benchmark_launch = _inc(
        "anhc_benchmark",
        "anhc_benchmark.launch.py",
        {"scenario_file": LaunchConfiguration("scenario_file")},
        condition=IfCondition(LaunchConfiguration("run_benchmark")),
    )

    # ── 6. live dashboard (always) ─────────────────────────────────────────────
    dashboard_node = Node(
        package="anhc_viz",
        executable="anhc_dashboard_panel",
        name="anhc_dashboard_panel",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription([
        algorithm_arg,
        use_rviz_arg,
        run_benchmark_arg,
        world_arg,
        gz_extra_args_arg,
        scenario_file_arg,
        sim_launch,
        perception_launch,
        planning_launch,
        rviz_node,
        benchmark_launch,
        dashboard_node,
    ])
