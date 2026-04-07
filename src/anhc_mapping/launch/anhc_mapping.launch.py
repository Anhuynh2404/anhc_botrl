from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution(
        [FindPackageShare('anhc_mapping'), 'config', 'costmap_params.yaml'])

    return LaunchDescription([
        Node(
            package='anhc_mapping',
            executable='anhc_costmap_node.py',
            name='anhc_costmap_node',
            parameters=[params],
            output='screen',
        ),
        Node(
            package='anhc_mapping',
            executable='anhc_global_costmap_node.py',
            name='anhc_global_costmap_node',
            parameters=[params],
            output='screen',
        ),
    ])
