import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('simulation_pkg')
    # World File
    world_file = os.path.join(pkg_share, 'worlds', 'challenging_cafeteria.sdf')
    robot_file = os.path.join(pkg_share, 'models', 'my_robot', 'model.sdf')
    models_path = os.path.join(pkg_share, 'models')
    
    # Set Gazebo resource path to find models
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=models_path + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )
    
    spawn = Node(
        package='ros_gz_sim', 
        executable='create', 
        arguments=['-name', 'itu_bot', '-file', robot_file, '-z', '0.1'], 
        output='screen'
    )
    
    bridge = Node(
        package='ros_gz_bridge', 
        executable='parameter_bridge', 
        arguments=[
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'
        ], 
        output='screen'
    )

    return LaunchDescription([gz_resource_path, gazebo, spawn, bridge])

