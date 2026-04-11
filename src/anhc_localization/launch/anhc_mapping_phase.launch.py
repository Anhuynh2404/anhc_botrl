from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_rviz = LaunchConfiguration("use_rviz")
    slam_params_file = LaunchConfiguration("slam_params_file")

    default_slam_params = PathJoinSubstitution(
        [FindPackageShare("anhc_localization"), "config", "anhc_slam_mapping.yaml"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("anhc_viz"), "rviz", "anhc_mapping.rviz"]
    )
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"]
                )
            ]
        ),
        launch_arguments={
            "slam_params_file": slam_params_file,
            "use_sim_time": "true",
            "autostart": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz2 for mapping visualization.",
            ),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=default_slam_params,
                description="Path to the slam_toolbox mapping parameters YAML.",
            ),
            slam_launch,
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_mapping",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
