import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Gazebo Harmonic converts package:// URIs to model:// when parsing URDF→SDF.
    # GZ_SIM_RESOURCE_PATH must contain the parent of the package share directory
    # (i.e. install/share) so Gazebo can resolve model://anhc_description/meshes/...
    _pkg_share = get_package_share_directory("anhc_description")
    _gz_extra_path = os.path.dirname(_pkg_share)  # install/share → resolves model://anhc_description/...
    # Also add the obj/ texture directory so OGRE2 can resolve map_Kd PNG paths from MTL files.
    _obj_tex_path = os.path.join(_pkg_share, "meshes", "anhc_botrl", "visual", "obj")
    _existing_gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    _gz_resource_path = (
        _gz_extra_path + ":" + _obj_tex_path
        + (":" + _existing_gz_path if _existing_gz_path else "")
    )

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
            # Allow Gazebo to resolve model://anhc_description/meshes/... URIs
            # (package:// is converted to model:// during URDF→SDF parsing).
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", _gz_resource_path),
            gz_launch,
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description],
                output="screen",
            ),
            # Fallback joint state source for RViz RobotModel TF stability. This keeps
            # wheel transforms available even when Gazebo joint-state bridging is absent.
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                parameters=[{"use_sim_time": True, "rate": 30.0}],
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
            Node(
                package="anhc_simulation",
                executable="anhc_odom_tf_node.py",
                name="anhc_odom_tf_publisher",
                parameters=[
                    {"use_sim_time": True, "use_odometry_msg_stamp": True},
                ],
                output="screen",
            ),
            # Scan relay: /scan_gz -> /scan, header.frame_id only; odom TF from anhc_odom_tf_node.
            Node(
                package="anhc_simulation",
                executable="anhc_scan_frame_relay.py",
                name="anhc_scan_frame_relay",
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
            spawn_entity,

            # Gazebo scoped sensor frames → URDF names on /tf_static (stamp 0). Avoids
            # C++ static_transform_publisher republishing on /tf with clock.now() ahead of
            # bridged /odom stamps (tf2 "jump back in time" → buffer clear).
            Node(
                package="anhc_simulation",
                executable="anhc_gz_frame_static_tf.py",
                name="anhc_gz_frame_static_bridges",
                parameters=[{"use_sim_time": True}],
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







