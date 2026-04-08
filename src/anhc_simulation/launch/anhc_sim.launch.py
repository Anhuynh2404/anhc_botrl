from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                parameters=[{"config_file": bridge_config}],
                output="screen",
            ),
            spawn_entity,

            # ── Gazebo ↔ ROS frame-ID bridge (static transforms) ──────────────────
            # Gazebo Harmonic flattens all fixed URDF joints into the root link
            # (base_footprint) when building its internal model.  As a result,
            # every sensor that was declared on a child fixed-link (lidar_link,
            # camera_link, imu_link) gets a Gazebo scoped frame_id of the form:
            #   {model_name}/{root_link}/{sensor_name}
            # e.g. "anhc_bot/base_footprint/anhc_lidar"
            #
            # These Gazebo-style frame IDs do NOT appear in the ROS TF tree (which
            # is published by robot_state_publisher using the original URDF link
            # names).  The mismatch causes:
            #   1. SLAM toolbox to fail silently (cannot look up scan frame → no map)
            #   2. RViz "Could not transform … to [map]" errors for sensor displays
            #
            # Fix: publish identity static transforms connecting each Gazebo sensor
            # frame to its corresponding URDF link.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="gz_lidar_frame_bridge",
                arguments=[
                    "0", "0", "0", "0", "0", "0",
                    "lidar_link",                         # URDF frame (in TF tree)
                    "anhc_bot/base_footprint/anhc_lidar", # Gazebo sensor frame
                ],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="gz_camera_rgb_frame_bridge",
                arguments=[
                    "0", "0", "0", "0", "0", "0",
                    "camera_link",
                    "anhc_bot/base_footprint/anhc_rgb_camera",
                ],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="gz_camera_depth_frame_bridge",
                arguments=[
                    "0", "0", "0", "0", "0", "0",
                    "camera_link",
                    "anhc_bot/base_footprint/anhc_depth_camera",
                ],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="gz_imu_frame_bridge",
                arguments=[
                    "0", "0", "0", "0", "0", "0",
                    "imu_link",
                    "anhc_bot/base_footprint/anhc_imu",
                ],
                output="screen",
            ),

            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
