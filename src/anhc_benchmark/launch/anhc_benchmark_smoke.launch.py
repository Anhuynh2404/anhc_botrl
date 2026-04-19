"""Smoke launch: 1 scenario × 2 algorithms × 2 trials (see config/anhc_benchmark_scenarios_smoke.yaml).

Use after ``anhc_nav.launch.py`` is up. Default ``output_dir`` is ``<workspace>/bench_results``
(CSV under ``raw/``, ``report.md`` beside it). Override with ``output_dir:=...`` if needed.
"""

import os

from ament_index_python.packages import get_package_share_directory
from anhc_benchmark.bench_paths import default_benchmark_output_dir
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    _smoke_yaml = os.path.join(
        get_package_share_directory("anhc_benchmark"),
        "config",
        "anhc_benchmark_scenarios_smoke.yaml",
    )
    scenario_file_arg = DeclareLaunchArgument(
        "scenario_file",
        default_value=_smoke_yaml,
        description="Smoke scenario YAML (override for custom smoke).",
    )
    _default_out = default_benchmark_output_dir()
    output_dir_arg = DeclareLaunchArgument(
        "output_dir",
        default_value=_default_out,
        description="Base directory: raw/benchmark_*.csv, report.md (default: <ws>/bench_results).",
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
                "execution_timeout_s": 120.0,
                "startup_delay_s": 8.0,
            }
        ],
    )

    live_metrics = Node(
        package="anhc_benchmark",
        executable="anhc_live_metrics_node",
        name="anhc_live_metrics",
        output="screen",
    )

    gz_obstacle = Node(
        package="anhc_benchmark",
        executable="anhc_benchmark_gz_obstacle_node",
        name="anhc_benchmark_gz_obstacle",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    results_analyzer = Node(
        package="anhc_benchmark",
        executable="anhc_results_analyzer_node",
        name="anhc_results_analyzer",
        output="screen",
        parameters=[{"output_dir": LaunchConfiguration("output_dir")}],
    )

    hint = LogInfo(
        msg=(
            "[anhc_benchmark_smoke] CSV → <output_dir>/raw/benchmark_<ts>.csv; "
            "report → <output_dir>/report.md (default output_dir: workspace/bench_results)."
        )
    )

    launch_analyzer_on_runner_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=runner,
            on_exit=[results_analyzer],
        )
    )

    return LaunchDescription(
        [
            hint,
            scenario_file_arg,
            output_dir_arg,
            runner,
            live_metrics,
            gz_obstacle,
            launch_analyzer_on_runner_exit,
        ]
    )
