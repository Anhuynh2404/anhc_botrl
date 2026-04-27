"""Simulation + SLAM mapping launch (Phase 1).

``anhc_sim.launch.py`` publishes Gazebo↔URDF static TF, ``/scan`` relay, and
``odom``→``base_footprint`` TF so this file only adds slam_toolbox + RViz.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

_AUTO = "__auto__"


def _apply_slam_params_default(context):
    world = context.perform_substitution(LaunchConfiguration("world"))
    slam_params = context.perform_substitution(LaunchConfiguration("slam_params_file"))
    if slam_params != _AUTO:
        return []
    if world == "anhc_factory":
        return [
            SetLaunchConfiguration(
                "slam_params_file",
                PathJoinSubstitution(
                    [
                        FindPackageShare("anhc_localization"),
                        "config",
                        "anhc_slam_mapping_factory.yaml",
                    ]
                ),
            )
        ]
    return [
        SetLaunchConfiguration(
            "slam_params_file",
            PathJoinSubstitution(
                [FindPackageShare("anhc_localization"), "config", "anhc_slam_mapping.yaml"]
            ),
        )
    ]


def generate_launch_description() -> LaunchDescription:
    world = LaunchConfiguration("world")
    use_rviz = LaunchConfiguration("use_rviz")
    slam_params_file = LaunchConfiguration("slam_params_file")
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
            "slam_params_file": slam_params_file,
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
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=_AUTO,
                description=(
                    "SLAM Toolbox mapping params YAML. '__auto__' selects "
                    "anhc_slam_mapping_factory.yaml for world:=anhc_factory, "
                    "otherwise anhc_slam_mapping.yaml."
                ),
            ),
            LogInfo(
                msg=(
                    "Drive the robot around the entire environment, then run: "
                    "bash scripts/anhc_save_map.sh"
                )
            ),
            OpaqueFunction(function=_apply_slam_params_default),
            sim_launch,
            mapping_launch,
        ]
    )
