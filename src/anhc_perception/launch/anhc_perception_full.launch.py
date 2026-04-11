"""Full perception stack launch.

Includes (in order):
  1. anhc_localization  — SLAM + IMU filter + EKF
  2. anhc_perception    — point cloud processor + obstacle detector + camera
  3. anhc_mapping       — dynamic costmap + global costmap
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_file = LaunchConfiguration("map_file")
    use_slam = LaunchConfiguration("use_slam")
    default_map_file = PathJoinSubstitution(
        [EnvironmentVariable("HOME"), "maps", "anhc_indoor_map.yaml"]
    )

    def _inc(pkg: str, launch_file: str, extra_args: dict | None = None) -> IncludeLaunchDescription:
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution(
                    [FindPackageShare(pkg), 'launch', launch_file])
            ]),
            launch_arguments=(extra_args or {}).items(),
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            "map_file",
            default_value=default_map_file,
            description="Absolute path to saved map YAML file for map_server.",
        ),
        DeclareLaunchArgument(
            "use_slam",
            default_value="true",
            description="If true, run slam_toolbox; if false, load map_file via map_server.",
        ),
        _inc(
            'anhc_localization',
            'anhc_localization.launch.py',
            {"map_file": map_file, "use_slam": use_slam},
        ),
        _inc('anhc_perception',   'anhc_perception.launch.py'),
        _inc('anhc_mapping',      'anhc_mapping.launch.py'),
    ])
