import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('simulation_pkg')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # Paths
    world_file = os.path.join(pkg_share, 'worlds', 'challenging_cafeteria.sdf')
    robot_file = os.path.join(pkg_share, 'models', 'my_robot', 'model.sdf')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_file = LaunchConfiguration('map')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        description='Full path to map yaml file to load')

    # Set Gazebo resource path
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_share, 'models') + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )

    # 1. Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # 2. Spawn Robot
    spawn = Node(
        package='ros_gz_sim', 
        executable='create', 
        arguments=['-name', 'itu_bot', '-file', robot_file, '-z', '0.1', '-x', '0.0', '-y', '0.0'], 
        output='log'
    )

    # 3. Bridge (FIXED: Unidirectional TF to prevent loop)
    bridge = Node(
        package='ros_gz_bridge', 
        executable='parameter_bridge', 
        arguments=[
            '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ], 
        output='log',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 4. TF Publishers
    # FIXED: Parent=base_footprint, Child=base_link (Matches standard Odom->Footprint->Base)
    tf_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0', 'base_footprint', 'base_link'],
        parameters=[{'use_sim_time': True}],
        output='log'
    )
    # Lidar TF
    tf_scan = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.1','0','0.25','0','0','0', 'base_link', 'itu_bot/chassis/lidar'],
        parameters=[{'use_sim_time': True}],
        output='log'
    )

    # 5. Nav2 Bringup (Event-Driven: Starts after Spawn)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file': nav2_params_file
        }.items()
    )

    # 6. Perception Node
    perception = Node(
        package='perception_pkg',
        executable='gemini_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 7. Patrol Node (The Brain)
    patrol_node = Node(
        package='simulation_pkg',
        executable='patrol_node.py',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 8. Initial Pose Publisher
    initial_pose_node = Node(
        package='simulation_pkg',
        executable='initial_pose_pub.py',
        output='log',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 9. Planner Node
    planner_node = Node(
        package='simulation_pkg',
        executable='sequential_planner.py',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Event Handler: Wait for Spawn to exit, then start Nav2
    # And after a short delay, start the application nodes
    start_robot_ops = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn,
            on_exit=[
                nav2_launch,
                TimerAction(period=15.0, actions=[initial_pose_node]),
                TimerAction(period=20.0, actions=[patrol_node]),
                TimerAction(period=25.0, actions=[planner_node])
            ]
        )
    )

    return LaunchDescription([
        declare_map_yaml_cmd,
        gz_resource_path, 
        gazebo, 
        bridge, 
        tf_base, 
        tf_scan, 
        spawn,
        perception, # Can start parallel
        start_robot_ops
    ])

