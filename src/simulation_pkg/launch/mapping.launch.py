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
    slam_toolbox = get_package_share_directory('slam_toolbox')
    
    # Paths
    world_file = os.path.join(pkg_share, 'worlds', 'challenging_cafeteria.sdf')
    robot_file = os.path.join(pkg_share, 'models', 'my_robot', 'model.sdf')
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'sim_config.rviz')
    
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

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
        output='screen'
    )

    # 3. Bridge
    bridge = Node(
        package='ros_gz_bridge', 
        executable='parameter_bridge', 
        arguments=[
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'
        ], 
        output='screen'
    )
    
    # 4. TF Publishers (Important for SLAM)
    # base_footprint -> base_link
    tf_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    # base_link -> itu_bot/chassis/lidar
    tf_scan = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.1', '0', '0.25', '0', '0', '0', 'base_link', 'itu_bot/chassis/lidar'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 5. SLAM Toolbox
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}]
    )

    # 6. RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 7. Perception (For logging)
    perception = Node(
        package='perception_pkg',
        executable='gemini_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 7.5 Nav2 Navigation (For Auto Explorer)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file
        }.items()
    )
    logger = Node(
        package='simulation_pkg',
        executable='poster_logger.py',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 9. Auto Explorer Node (Our Logic)
    explorer = Node(
        package='simulation_pkg',
        executable='simple_explorer.py',
        name='auto_explorer',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Lifecycle Manager for Nav2
    # Note: Nav2 launch file usually handles this, but since we launch components separately:
    # We rely on nav2_bringup to handle lifecycle usually.
    # Here we just launch the top-level bringup.
    
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file
        }.items()
    )

    return LaunchDescription([
        gz_resource_path, 
        gazebo, 
        spawn, 
        bridge, 
        tf_base, 
        tf_scan, 
        slam,
        nav2,
        rviz,
        perception,
        logger,
        explorer
    ])
