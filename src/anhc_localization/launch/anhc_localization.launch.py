from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_slam = LaunchConfiguration('use_slam')
    map_file = LaunchConfiguration('map_file')

    slam_params = PathJoinSubstitution(
        [FindPackageShare('anhc_localization'), 'config', 'anhc_slam_mapping.yaml'])
    ekf_params = PathJoinSubstitution(
        [FindPackageShare('anhc_localization'), 'config', 'ekf_params.yaml'])
    default_map_file = PathJoinSubstitution(
        [EnvironmentVariable('HOME'), 'maps', 'anhc_indoor_map.yaml']
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py',
            ])]
        ),
        launch_arguments={
            # Our repo's params file (frames, scan topic, etc.)
            'slam_params_file': slam_params,
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items(),
        condition=IfCondition(use_slam),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_slam', default_value='true'),
        DeclareLaunchArgument(
            'map_file',
            default_value=default_map_file,
            description='Absolute path to saved map YAML file.',
        ),

        # 1. IMU complementary filter (always on — lightweight)
        Node(
            package='anhc_localization',
            executable='anhc_imu_filter_node.py',
            name='anhc_imu_filter_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'alpha': 0.98,
                'accel_lpf_alpha': 0.8,
            }],
        ),

        # 2. slam_toolbox — online async SLAM producing /map and map→odom TF
        slam_launch,

        # 3. static map server for localization-only mode (disabled in SLAM mode)
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='anhc_map_server',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'yaml_filename': map_file,
            }],
            condition=UnlessCondition(use_slam),
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='anhc_map_lifecycle_manager',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['anhc_map_server'],
            }],
            condition=UnlessCondition(use_slam),
        ),

        # 4. robot_localization EKF — fuses /odom + /imu/data → /odometry/filtered
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_params],
            output='screen',
            remappings=[
                ('odometry/filtered', '/odometry/filtered'),
            ],
        ),
    ])
