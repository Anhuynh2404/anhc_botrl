"""Launch file for the anhc benchmark framework.

Starts:
  1. anhc_benchmark_runner_node  — orchestrates scenarios, writes CSV
  2. anhc_live_metrics_node      — publishes /benchmark/live at 2 Hz

When the runner exits, automatically launches:
  3. anhc_results_analyzer_node  — reads CSV, publishes /benchmark/summary,
                                    writes ~/anhc_benchmark_results/report.md

Usage
-----
  ros2 launch anhc_benchmark anhc_benchmark.launch.py \\
    scenario_file:=<path/to/scenarios.yaml>
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    scenario_file_arg = DeclareLaunchArgument(
        "scenario_file",
        default_value="",
        description="Absolute path to the YAML benchmark scenario file.",
    )

    output_dir_arg = DeclareLaunchArgument(
        "output_dir",
        default_value="~/anhc_benchmark_results",
        description="Directory where CSV results and report.md are written.",
    )

    runner = Node(
        package="anhc_benchmark",
        executable="anhc_benchmark_runner_node",
        name="anhc_benchmark_runner",
        output="screen",
            parameters=[
                {
                    "scenario_file": LaunchConfiguration("scenario_file"),
                    "output_dir": LaunchConfiguration("output_dir"),
                    "goal_tolerance": 0.30,
                    "planning_timeout_s": 15.0,
                    "execution_timeout_s": 90.0,
                }
            ],
    )

    live_metrics = Node(
        package="anhc_benchmark",
        executable="anhc_live_metrics_node",
        name="anhc_live_metrics",
        output="screen",
    )

    results_analyzer = Node(
        package="anhc_benchmark",
        executable="anhc_results_analyzer_node",
        name="anhc_results_analyzer",
        output="screen",
        parameters=[{"output_dir": LaunchConfiguration("output_dir")}],
    )

    # Auto-launch the analyzer once the runner process exits.
    launch_analyzer_on_runner_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=runner,
            on_exit=[results_analyzer],
        )
    )

    return LaunchDescription([
        scenario_file_arg,
        output_dir_arg,
        runner,
        live_metrics,
        launch_analyzer_on_runner_exit,
    ])
