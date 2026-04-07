from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution(
        [FindPackageShare('anhc_perception'), 'config', 'perception_params.yaml'])

    return LaunchDescription([
        Node(
            package='anhc_perception',
            executable='anhc_pointcloud_processor_node.py',
            name='anhc_pointcloud_processor_node',
            parameters=[params],
            output='screen',
        ),
        Node(
            package='anhc_perception',
            executable='anhc_obstacle_detector_node.py',
            name='anhc_obstacle_detector_node',
            parameters=[params],
            output='screen',
        ),
        Node(
            package='anhc_perception',
            executable='anhc_camera_processor_node.py',
            name='anhc_camera_processor_node',
            parameters=[params],
            output='screen',
        ),
    ])
