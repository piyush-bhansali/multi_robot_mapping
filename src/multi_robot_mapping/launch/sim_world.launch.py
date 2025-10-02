import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('multi_robot_mapping')

    # Use the SDF we created (self-contained, SDF 1.9)
    world_file = os.path.join(pkg_share, 'worlds', 'park.sdf')

    # ros_gz_sim (Gazebo Sim) launcher
    gz_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    )

    # Pass gz_args: -r (run) + world path
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={
            'gz_args': f'-r "{world_file}"',
            'on_exit_shutdown': 'true'
        }.items()
    )

    return LaunchDescription([gazebo])
