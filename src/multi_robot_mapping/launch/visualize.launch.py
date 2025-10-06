import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Package directory
    pkg_share = get_package_share_directory('multi_robot_mapping')
    
    # RViz config file paths
    rviz_config_tb3_1 = os.path.join(pkg_share, 'rviz', 'tb3_1.rviz')
    rviz_config_tb3_2 = os.path.join(pkg_share, 'rviz', 'tb3_2.rviz')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # RViz node for tb3_1
    rviz_tb3_1 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_tb3_1',
        arguments=['-d', rviz_config_tb3_1],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # RViz node for tb3_2
    rviz_tb3_2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_tb3_2',
        arguments=['-d', rviz_config_tb3_2],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        
        # RViz instances
        rviz_tb3_1,
        rviz_tb3_2,
    ])