#!/usr/bin/env python3
"""
Launch file for custom robot in TurtleBot3 world
Uses the same world as TurtleBot3 but spawns our custom robot
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    pkg_dir = get_package_share_directory('gazebo_differential_drive_robot')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    
    # World file (using husarion_world.sdf)
    world = '/home/metin/localization_and_mapping/src/husarion_gz_worlds/worlds/husarion_world.sdf'
    
    # Our custom robot model
    robot_model = os.path.join(pkg_dir, 'models', 'my_robot', 'model.sdf')
    
    # Bridge config
    bridge_params = os.path.join(pkg_dir, 'config', 'my_robot_bridge.yaml')
    
    # Declare launch arguments
    declare_x = DeclareLaunchArgument('x_pose', default_value='-2.0')
    declare_y = DeclareLaunchArgument('y_pose', default_value='-0.5')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    
    # Set GZ_SIM_RESOURCE_PATH for models
    set_env_vars = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(turtlebot3_gazebo, 'models')
    )
    
    # Gazebo server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -s -v2 {world}',
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # Gazebo client (GUI)
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-g -v2',
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # Spawn custom robot
    spawn_robot_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_robot',
            '-file', robot_model,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.30'
        ],
        output='screen'
    )
    
    # ROS-Gazebo bridge
    bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args', '-p', f'config_file:={bridge_params}'
        ],
        output='screen'
    )
    
    # Robot state publisher (for TF)
    # We need a URDF for robot_state_publisher, but we can use a minimal one
    # For now, let's create static transforms
    static_tf_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_footprint',
            '--child-frame-id', 'base_link'
        ]
    )
    
    static_tf_scan = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.0', '--y', '0', '--z', '0.17',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'base_scan'
        ]
    )
    
    ld = LaunchDescription()
    
    # Add actions
    ld.add_action(declare_x)
    ld.add_action(declare_y)
    ld.add_action(declare_use_sim_time)
    ld.add_action(set_env_vars)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(bridge_cmd)
    ld.add_action(static_tf_base)
    ld.add_action(static_tf_scan)
    
    return ld
