"""Full system launch: simulation + perception + planning.

Includes (in order):
  1. anhc_sim.launch.py            — Gazebo simulation + robot state publisher
  2. anhc_perception_full.launch.py — localization + perception + mapping
  3. anhc_planning.launch.py        — global planner + path follower
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    algorithm_arg = DeclareLaunchArgument(
        "algorithm",
        default_value="astar",
        description="Path planning algorithm forwarded to anhc_planning. "
                    "Options: astar, dijkstra, rrt_star, dstar_lite, rl",
    )

    gz_extra_args_arg = DeclareLaunchArgument(
        "gz_extra_args",
        default_value="",
        description="Extra arguments forwarded to gz sim. "
                    "Use '-s' for headless/server-only mode (no GUI). "
                    "Default '' launches the full Gazebo GUI.",
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Launch RViz2 alongside the simulation.",
    )

    def _inc(pkg: str, launch_file: str, extra_args: dict | None = None):
        args = extra_args.items() if extra_args else []
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution(
                    [FindPackageShare(pkg), "launch", launch_file]
                )
            ]),
            launch_arguments=args,
        )

    sim_launch = _inc(
        "anhc_simulation",
        "anhc_sim.launch.py",
        {
            "gz_extra_args": LaunchConfiguration("gz_extra_args"),
            "use_rviz": LaunchConfiguration("use_rviz"),
        },
    )

    perception_launch = _inc("anhc_perception", "anhc_perception_full.launch.py")

    planning_launch = _inc(
        "anhc_planning",
        "anhc_planning.launch.py",
        {"algorithm": LaunchConfiguration("algorithm")},
    )

    return LaunchDescription([
        algorithm_arg,
        gz_extra_args_arg,
        use_rviz_arg,
        sim_launch,
        perception_launch,
        planning_launch,
    ])
