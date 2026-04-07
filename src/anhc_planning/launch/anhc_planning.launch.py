"""Launch the anhc_planning stack (global planner + path follower)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    algorithm_arg = DeclareLaunchArgument(
        "algorithm",
        default_value="astar",
        description="Planning algorithm to use. Options: astar, dijkstra, rrt_star, dstar_lite, rl",
    )

    params_file = PathJoinSubstitution(
        [FindPackageShare("anhc_planning"), "config", "planner_params.yaml"]
    )

    global_planner_node = Node(
        package="anhc_planning",
        executable="anhc_global_planner_node",
        name="anhc_global_planner",
        parameters=[
            params_file,
            {"algorithm": LaunchConfiguration("algorithm")},
        ],
        output="screen",
    )

    path_follower_node = Node(
        package="anhc_planning",
        executable="anhc_path_follower_node",
        name="anhc_path_follower",
        parameters=[params_file],
        output="screen",
    )

    return LaunchDescription([
        algorithm_arg,
        global_planner_node,
        path_follower_node,
    ])
