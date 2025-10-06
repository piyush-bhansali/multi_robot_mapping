import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Package directories
    pkg_this  = get_package_share_directory('multi_robot_mapping')
    pkg_gzsim = get_package_share_directory('ros_gz_sim')

    # --- File Paths ---
    world_default = os.path.join(pkg_this, 'worlds', 'maze.sdf')
    tb3_sdf       = os.path.join(pkg_this, 'models', 'turtlebot3_waffle_pi', 'model.sdf')
    tb3_urdf      = os.path.join(pkg_this, 'urdf', 'turtlebot3_waffle_pi.urdf')
    bridge_config_robot = os.path.join(pkg_this, 'config', 'tb3_bridge.yaml')
    bridge_config_common = os.path.join(pkg_this, 'config', 'tb3_bridge_common.yaml')
    # RViz config file paths
    rviz_config_tb3_1 = os.path.join(pkg_this, 'rviz', 'tb3_1.rviz')
    rviz_config_tb3_2 = os.path.join(pkg_this, 'rviz', 'tb3_2.rviz')

    # --- Set Gazebo Resource Path for mesh loading ---
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=pkg_this
    )

    # --- Launch Arguments ---
    world_arg        = DeclareLaunchArgument('world_path',  default_value=world_default)
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    gui_arg          = DeclareLaunchArgument('gui',          default_value='true')

    # --- Configurations ---
    world_path   = LaunchConfiguration('world_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui          = LaunchConfiguration('gui')

    # --- Start Gazebo Harmonic ---
    gz_launch = os.path.join(pkg_gzsim, 'launch', 'gz_sim.launch.py')
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={
            'gz_args': ['-r ', world_path],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # World name - must match <world name="..."> in your maze.sdf
    world_name = 'maze_world'

    # Read the robot SDF file content (template)
    with open(tb3_sdf, 'r') as sdf_file:
        robot_sdf_template = sdf_file.read()

    # Replace package:// URIs with file:// URIs for Gazebo
    mesh_path = os.path.join(pkg_this, 'meshes')
    robot_sdf_template = robot_sdf_template.replace(
        'package://multi_robot_mapping/meshes',
        f'file://{mesh_path}'
    )

    # --- Function to spawn robot with namespace-aware joint names ---
    def spawn_robot(robot_name, x, y, yaw):
        # Clone and modify SDF for this specific robot
        robot_sdf = robot_sdf_template
        
        # Replace actual joint definitions (the <joint name="..."> tags)
        robot_sdf = robot_sdf.replace(
            '<joint name="wheel_left_joint"',
            f'<joint name="{robot_name}/wheel_left_joint"'
        )
        robot_sdf = robot_sdf.replace(
            '<joint name="wheel_right_joint"',
            f'<joint name="{robot_name}/wheel_right_joint"'
        )

        # Replace frame IDs in DiffDrive plugin for TF
        robot_sdf = robot_sdf.replace(
            '<frame_id>odom</frame_id>',
            f'<frame_id>{robot_name}/odom</frame_id>'
        )

        # Replace gz_frame_id for sensors
        robot_sdf = robot_sdf.replace(
            '<gz_frame_id>base_scan</gz_frame_id>',
            f'<gz_frame_id>{robot_name}/base_scan</gz_frame_id>'
        )
        
        robot_sdf = robot_sdf.replace(
            '<child_frame_id>base_footprint</child_frame_id>',
            f'<child_frame_id>{robot_name}/base_footprint</child_frame_id>'
        )
        
        # Replace joint names in the DiffDrive plugin
        robot_sdf = robot_sdf.replace(
            '<left_joint>wheel_left_joint</left_joint>',
            f'<left_joint>{robot_name}/wheel_left_joint</left_joint>'
        )
        robot_sdf = robot_sdf.replace(
            '<right_joint>wheel_right_joint</right_joint>',
            f'<right_joint>{robot_name}/wheel_right_joint</right_joint>'
        )
        
        # Replace joint names in the JointStatePublisher plugin
        robot_sdf = robot_sdf.replace(
            '<joint_name>wheel_left_joint</joint_name>',
            f'<joint_name>{robot_name}/wheel_left_joint</joint_name>'
        )
        robot_sdf = robot_sdf.replace(
            '<joint_name>wheel_right_joint</joint_name>',
            f'<joint_name>{robot_name}/wheel_right_joint</joint_name>'
        )
        
        return Node(
            package='ros_gz_sim',
            executable='create',
            name=f'spawn_{robot_name}',
            output='screen',
            arguments=[
                '-world', world_name,
                '-name', robot_name,
                '-string', robot_sdf,
                '-x', str(x),
                '-y', str(y),
                '-z', '0.02',
                '-Y', str(yaw)
            ]
        )

    # --- Gazebo-ROS Bridge Function (robot-specific, NO clock) ---
    def create_bridge(robot_name):
        # Read the robot-specific YAML template (without clock)
        with open(bridge_config_robot, 'r') as f:
            template_content = f.read()
        
        # Replace placeholders with actual robot name and world name
        config_content = template_content.replace('{robot_name}', robot_name)
        config_content = config_content.replace('{world_name}', world_name)
        
        # Write to temporary config file
        temp_config = f'/tmp/{robot_name}_bridge.yaml'
        with open(temp_config, 'w') as f:
            f.write(config_content)
        
        return Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'bridge_{robot_name}',
            output='screen',
            parameters=[{
                'config_file': temp_config,
                'use_sim_time': use_sim_time
            }]
        )

    # --- Clock Bridge (single instance for entire system) ---
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config_common,
            'use_sim_time': False  # Clock bridge itself should NOT use sim time
        }]
    )

    # --- Robot State Publisher Function ---
    def create_robot_state_publisher(robot_name):
        return Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher',
            namespace=robot_name, 
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': Command(['xacro ', tb3_urdf, ' namespace:=', robot_name, '/'])
            }]
        )

    # --- Build and Return Launch Description ---
    return LaunchDescription([
        # Set environment variable FIRST
        set_gz_resource_path,
        
        # Launch arguments
        world_arg,
        use_sim_time_arg,
        gui_arg,
        
        # Start Gazebo
        start_gazebo,

        # Start clock bridge first (at 1 second)
        TimerAction(
            period=1.0,
            actions=[clock_bridge]
        ),

        # Wait for Gazebo and clock to stabilize, then spawn robots and start bridges
        TimerAction(
            period=4.0,
            actions=[
                # Spawn robots
                spawn_robot('tb3_1', -8.0, -8.0, 0.0),
                spawn_robot('tb3_2', 8.0, 8.0, -2.356),
                
                # Start robot-specific bridges (without clock)
                create_bridge('tb3_1'),
                create_bridge('tb3_2'),
                
                # Start robot state publishers
                create_robot_state_publisher('tb3_1'),
                create_robot_state_publisher('tb3_2'),
            ]
        ),

        # Wait for TF tree to be fully populated before starting RViz
        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz_tb3_1',
                    arguments=['-d', rviz_config_tb3_1],
                    parameters=[{'use_sim_time': use_sim_time}],
                    output='screen'
                ),
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz_tb3_2',
                    arguments=['-d', rviz_config_tb3_2],
                    parameters=[{'use_sim_time': use_sim_time}],
                    output='screen'
                ),
            ]
        ),
    ])