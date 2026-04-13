"""Simulation + SLAM mapping launch (Phase 1).

LiDAR Gazebo↔URDF static TF is published from ``anhc_sim.launch.py`` (included
below) so this file stays a thin orchestrator and ``anhc_master`` gets the same TF.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
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
            # Empty value disables '-s' default, so Gazebo GUI is shown.
            "gz_extra_args": "",
        }.items(),
    )

    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("anhc_localization"),
                        "launch",
                        "anhc_mapping_phase.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_rviz": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "Drive the robot around the entire environment, then run: "
                    "bash scripts/anhc_save_map.sh"
                )
            ),
            sim_launch,
            mapping_launch,
        ]
    )
