import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('biped_bike_runtime')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    map_yaml = os.path.join(package_share, 'maps', 'maze_half.yaml')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': map_yaml,
        }],
    )

    map_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    # The saved simulation map was built using exact Gazebo odometry, so its
    # map frame is aligned with odom. AMCL is reserved for real hardware.
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'map', '--child-frame-id', 'odom',
        ],
        output='screen',
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                package_share,
                'launch',
                'nav2_slam_navigation.launch.py',
            )
        )
    )

    # Give the map server time to activate before global costmap startup.
    delayed_navigation = TimerAction(
        period=3.0,
        actions=[navigation],
    )

    return LaunchDescription([
        map_to_odom,
        map_server,
        map_lifecycle_manager,
        delayed_navigation,
    ])
