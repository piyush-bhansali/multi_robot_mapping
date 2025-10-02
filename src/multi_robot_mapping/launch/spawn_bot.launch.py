import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Package directories
    pkg_this  = get_package_share_directory('multi_robot_mapping')
    pkg_gzsim = get_package_share_directory('ros_gz_sim')
    pkg_tb3d  = get_package_share_directory('turtlebot3_description')

    # --- File Paths ---
    world_default = os.path.join(pkg_this, 'worlds', 'park.sdf')
    tb3_sdf       = os.path.join(pkg_this, 'models', 'turtlebot3_waffle_pi', 'model.sdf')
    tb3_urdf      = os.path.join(pkg_tb3d, 'urdf', 'turtlebot3_waffle_pi.urdf')

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

    # World name - must match <world name="..."> in your park.sdf
    world_name = 'park_world'

    # Read the robot SDF file content
    with open(tb3_sdf, 'r') as sdf_file:
        robot_sdf_content = sdf_file.read()

    # --- Spawn Robot 1 ---
    spawn_tb3_1 = Node(
        package='ros_gz_sim', 
        executable='create', 
        name='spawn_tb3_1', 
        output='screen',
        arguments=[
            '-world', world_name, 
            '-name', 'tb3_1',
            '-string', robot_sdf_content,
            '-x', '-2.0', 
            '-y', '0.0', 
            '-z', '0.02', 
            '-Y', '0.0'
        ]
    )

    # --- Spawn Robot 2 ---
    spawn_tb3_2 = Node(
        package='ros_gz_sim', 
        executable='create', 
        name='spawn_tb3_2', 
        output='screen',
        arguments=[
            '-world', world_name, 
            '-name', 'tb3_2',
            '-string', robot_sdf_content,
            '-x', '2.0', 
            '-y', '0.0', 
            '-z', '0.02', 
            '-Y', '3.14159'
        ]
    )

    # --- Gazebo-ROS Bridge Function ---
    def create_bridge(robot_name):
        base_world = f'/world/{world_name}/model/{robot_name}'
        gz_model   = f'/model/{robot_name}'        # Where Gazebo actually publishes
        ros_ns     = f'/{robot_name}'

        return Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'bridge_{robot_name}',
            output='screen',
            arguments=[
                # Sensors: Gazebo → ROS (use [)
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                f'{base_world}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                f'{base_world}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                f'{base_world}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',

                # Commands: ROS → Gazebo (use ])
                # Bridge on Gazebo's actual topic, remap ROS side below
                f'{gz_model}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            ],
            remappings=[
                # Sensor remappings
                (f'{base_world}/odometry', f'{ros_ns}/odom'),
                (f'{base_world}/imu',      f'{ros_ns}/imu'),
                (f'{base_world}/scan',     f'{ros_ns}/scan'),

                # cmd_vel remapping - THIS IS THE KEY
                (f'{gz_model}/cmd_vel',    f'{ros_ns}/cmd_vel'),
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        )

    # Create bridges for both robots
    bridge_tb3_1 = create_bridge('tb3_1')
    bridge_tb3_2 = create_bridge('tb3_2')

    # --- TF Bridge (for transforms) ---
    tf_bridge = Node(
        package='ros_gz_bridge', 
        executable='parameter_bridge',
        name='tf_bridge', 
        output='screen',
        arguments=[
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # --- Robot State Publishers ---
    # Read URDF file once
    with open(tb3_urdf, 'r') as urdf_file:
        robot_description = urdf_file.read()

    def create_robot_state_publisher(robot_name):
        """
        Publishes robot state and TF transforms for visualization in RViz.
        Each robot gets its own namespaced TF tree.
        """
        return Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher',
            namespace=robot_name, 
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
                'frame_prefix': f'{robot_name}/'
            }]
        )

    # Create robot state publishers for both robots
    rsp_tb3_1 = create_robot_state_publisher('tb3_1')
    rsp_tb3_2 = create_robot_state_publisher('tb3_2')

    # --- Build and Return Launch Description ---
    return LaunchDescription([
        # Launch arguments
        world_arg,
        use_sim_time_arg,
        gui_arg,
        
        # Start Gazebo
        start_gazebo,
        
        # Wait 3 seconds for Gazebo to fully start, then spawn robots
        TimerAction(
            period=3.0,
            actions=[
                spawn_tb3_1,
                spawn_tb3_2,
            ]
        ),
        
        # Bridges (Gazebo <-> ROS2)
        bridge_tb3_1,
        bridge_tb3_2,
        tf_bridge,
        
        # Robot state publishers (for RViz visualization)
        rsp_tb3_1,
        rsp_tb3_2,
        
        # Add your custom algorithm nodes here
        # Example:
        # custom_ekf_1,
        # custom_ekf_2,
        # custom_nav_1,
        # custom_nav_2,
    ])