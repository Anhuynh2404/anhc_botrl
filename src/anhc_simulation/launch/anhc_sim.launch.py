from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    world_file = PathJoinSubstitution(
        [FindPackageShare("anhc_simulation"), "worlds", "anhc_indoor.sdf"]
    )
    bridge_config = PathJoinSubstitution(
        [FindPackageShare("anhc_simulation"), "config", "bridge_params.yaml"]
    )
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("anhc_description"), "urdf", "anhc_bot.urdf.xacro"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("anhc_description"), "rviz", "anhc_bot.rviz"]
    )

    robot_description = {"robot_description": Command(["xacro ", xacro_file])}

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
                )
            ]
        ),
        launch_arguments={"gz_args": ["-r ", world_file, " ", LaunchConfiguration("gz_extra_args")]}.items(),
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "anhc_bot",
            "-topic",
            "robot_description",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.15",
            "-Y",
            "0.0",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "gz_extra_args",
                default_value="-s",
                description="Extra args for gz sim. Default -s runs server-only (headless).",
            ),
            # Avoid FastDDS SHM init failures on some host setups.
            SetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4"),
            gz_launch,
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description],
                output="screen",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                parameters=[{"config_file": bridge_config}],
                output="screen",
            ),
            spawn_entity,
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
