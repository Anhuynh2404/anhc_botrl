"""Simulation + SLAM mapping launch (Phase 1).

``anhc_sim.launch.py`` publishes Gazebo↔URDF static TF, ``/scan`` relay, and
``odom``→``base_footprint`` TF so this file only adds slam_toolbox + RViz.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    world = LaunchConfiguration("world")
    use_rviz = LaunchConfiguration("use_rviz")
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
            "world": world,
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
            "use_rviz": use_rviz,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="anhc_office_v2",
                description="Gazebo world name (without .sdf).",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz2 for mapping visualization.",
            ),
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
