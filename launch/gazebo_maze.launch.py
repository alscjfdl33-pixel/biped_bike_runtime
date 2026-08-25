import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = get_package_share_directory('biped_bike_runtime')
    base_launch = os.path.join(pkg_share, 'launch', 'gazebo.launch.py')
    maze_world = os.path.join(pkg_share, 'worlds', 'simple_maze.sdf')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            # The transformation shifts base_footprint about +0.39 m in X.
            # Spawn left of center so the transformed bike settles at x ~= 0,
            # midway between the internal walls at x = +/-0.6 m.
            launch_arguments={
                'world': maze_world,
                'spawn_x': '-0.39',
            }.items(),
        ),
    ])
