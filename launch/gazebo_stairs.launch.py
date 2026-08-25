import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory("biped_bike_runtime")
    base_launch = os.path.join(package_share, "launch", "gazebo.launch.py")
    stair_world = os.path.join(package_share, "worlds", "stairs.sdf")

    return LaunchDescription(
        [
            # Keep the initial feet clear of the first riser.  The foot meshes
            # extend forward from the base frame, so larger X values can cause
            # contact and topple the robot before a trajectory is commanded.
            DeclareLaunchArgument("robot_x", default_value="0.04"),
            DeclareLaunchArgument("robot_y", default_value="0.0"),
            DeclareLaunchArgument("robot_z", default_value="0.40"),
            DeclareLaunchArgument("robot_yaw", default_value="0.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(base_launch),
                launch_arguments={
                    "world": stair_world,
                    "spawn_x": LaunchConfiguration("robot_x"),
                    "spawn_y": LaunchConfiguration("robot_y"),
                    "spawn_z": LaunchConfiguration("robot_z"),
                    "spawn_yaw": LaunchConfiguration("robot_yaw"),
                }.items(),
            )
        ]
    )
