import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'biped_bike_runtime'
    pkg_share = get_package_share_directory(package_name)
    urdf_file = os.path.join(pkg_share, 'urdf', 'biped_bike_runtime.urdf')

    with open(urdf_file, 'r') as infp:
        robot_description_config = infp.read()

    # Xacro is not used, so manually resolve the $(find package) macro for the controllers yaml
    robot_description_config = robot_description_config.replace(
        '$(find biped_bike_runtime)', pkg_share)

    gz_resource_path = os.path.dirname(pkg_share)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': ['-r ', LaunchConfiguration('world')]
        }.items(),
    )

    gz_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'biped_bike_runtime',
            '-topic', '/robot_description',
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', LaunchConfiguration('spawn_z'),
            '-Y', LaunchConfiguration('spawn_yaw'),
        ],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': True,
        }],
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/model/biped_bike_runtime/ground_truth_odometry'
            '@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        remappings=[
            (
                '/model/biped_bike_runtime/ground_truth_odometry',
                '/gazebo/ground_truth_odom',
            ),
        ],
        output='screen',
    )

    cmd_vel_to_wheels = Node(
        package='biped_bike_runtime',
        executable='cmd_vel_to_wheels.py',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    ground_truth_attitude = Node(
        package='biped_bike_runtime',
        executable='ground_truth_attitude.py',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ros2_control 스포너 노드
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    wheel_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["wheel_velocity_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_share, 'worlds', 'flat_autonomy.sdf'),
        ),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.40'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),
        gz_sim,
        robot_state_publisher,
        gz_spawn,
        gz_bridge,
        cmd_vel_to_wheels,
        ground_truth_attitude,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        wheel_velocity_controller_spawner,
    ])
