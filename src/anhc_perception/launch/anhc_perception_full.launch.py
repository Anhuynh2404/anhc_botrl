"""Full perception stack launch.

Includes (in order):
  1. anhc_localization  — SLAM + IMU filter + EKF
  2. anhc_perception    — point cloud processor + obstacle detector + camera
  3. anhc_mapping       — dynamic costmap + global costmap
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    def _inc(pkg: str, launch_file: str) -> IncludeLaunchDescription:
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution(
                    [FindPackageShare(pkg), 'launch', launch_file])
            ])
        )

    return LaunchDescription([
        _inc('anhc_localization', 'anhc_localization.launch.py'),
        _inc('anhc_perception',   'anhc_perception.launch.py'),
        _inc('anhc_mapping',      'anhc_mapping.launch.py'),
    ])
