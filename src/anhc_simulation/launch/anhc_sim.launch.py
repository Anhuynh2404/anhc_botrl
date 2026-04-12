from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Use "sim_use_rviz" rather than "use_rviz" to avoid clobbering the
    # parent launch's "use_rviz" flag when included via IncludeLaunchDescription.
    # ROS2 launch shares LaunchConfiguration values globally across all included
    # files, so reusing the same name would override the parent's value.
    use_rviz = LaunchConfiguration("sim_use_rviz")
    world_name = LaunchConfiguration("world")
    world_file = PathJoinSubstitution(
        [FindPackageShare("anhc_simulation"), "worlds",
         [world_name, ".sdf"]]
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

    # joint_state_publisher (Jazzy) needs robot_description as a real string param;
    # a bare Command() is not always coerced and the node then waits forever on the
    # /robot_description topic → no /joint_states → incomplete TF → SLAM/RViz queues overflow.
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", xacro_file]),
            value_type=str,
        ),
        "use_sim_time": True,
    }

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

    # Entity name must match bridge_params.yaml gz_topic_name (/model/<name>/cmd_vel).
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
            "0.20",
            "-Y",
            "0.0",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sim_use_rviz",
                default_value="true",
                description="Launch RViz2 with the basic sim config. "
                            "Set false when using anhc_master.launch.py "
                            "(which provides its own full RViz2 config).",
            ),
            DeclareLaunchArgument(
                "gz_extra_args",
                default_value="-s",
                description="Extra args for gz sim. Default -s runs server-only (headless).",
            ),
            DeclareLaunchArgument(
                "world",
                default_value="anhc_indoor",
                description="Gazebo world name (without .sdf). Must exist under worlds/.",
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
            # Publish joint states for non-fixed joints (e.g. left/right wheels)
            # so that robot_state_publisher can generate full TFs for all links.
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                parameters=[robot_description, {"rate": 50.0}],
                output="screen",
            ),
            Node(
                package="anhc_simulation",
                executable="anhc_cmd_vel_idle_gate.py",
                name="anhc_cmd_vel_idle_gate",
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                parameters=[
                    {"config_file": bridge_config, "use_sim_time": True},
                ],
                output="screen",
            ),
            # Scan relay + odom->base_footprint TF (see anhc_scan_frame_relay.py docstring).
            Node(
                package="anhc_simulation",
                executable="anhc_scan_frame_relay.py",
                name="anhc_scan_frame_relay",
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
            spawn_entity,

            # ── Gazebo ↔ ROS frame-ID bridge (static transforms) ────────────────────
            # LaserScan /scan is republished as lidar_link (see anhc_scan_frame_relay).
            # Keep lidar identity for point_cloud and any topic still using Gazebo frame.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="lidar_frame_bridge",
                parameters=[{"use_sim_time": True}],
                arguments=[
                    "--frame-id",
                    "lidar_link",
                    "--child-frame-id",
                    "anhc_bot/base_footprint/anhc_lidar",
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--roll",
                    "0",
                    "--pitch",
                    "0",
                    "--yaw",
                    "0",
                ],
                output="screen",
            ),
            # Camera / IMU may still use Gazebo scoped names → identity bridges below.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="gz_camera_rgb_frame_bridge",
                parameters=[{"use_sim_time": True}],
                arguments=[
                    "--frame-id",
                    "camera_link",
                    "--child-frame-id",
                    "anhc_bot/base_footprint/anhc_rgb_camera",
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--roll",
                    "0",
                    "--pitch",
                    "0",
                    "--yaw",
                    "0",
                ],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="gz_camera_depth_frame_bridge",
                parameters=[{"use_sim_time": True}],
                arguments=[
                    "--frame-id",
                    "camera_link",
                    "--child-frame-id",
                    "anhc_bot/base_footprint/anhc_depth_camera",
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--roll",
                    "0",
                    "--pitch",
                    "0",
                    "--yaw",
                    "0",
                ],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="gz_imu_frame_bridge",
                parameters=[{"use_sim_time": True}],
                arguments=[
                    "--frame-id",
                    "imu_link",
                    "--child-frame-id",
                    "anhc_bot/base_footprint/anhc_imu",
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--roll",
                    "0",
                    "--pitch",
                    "0",
                    "--yaw",
                    "0",
                ],
                output="screen",
            ),

            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
