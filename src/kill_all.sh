#!/bin/bash
# Kill all ROS2, Gazebo, RViz and related processes

echo "🛑 Stopping all ROS2, Gazebo, and simulation processes..."

# Kill Gazebo processes
pkill -9 -f gz
pkill -9 -f gazebo
pkill -9 -f gzclient
pkill -9 -f gzserver

# Kill RViz
pkill -9 -f rviz
pkill -9 -f rviz2

# Kill Nav2 and navigation nodes
pkill -9 -f nav2
pkill -9 -f amcl
pkill -9 -f bt_navigator
pkill -9 -f controller_server
pkill -9 -f planner_server
pkill -9 -f map_server
pkill -9 -f recoveries_server
pkill -9 -f waypoint_follower

# Kill Robot State Publisher
pkill -9 -f robot_state_publisher

# Kill Python scripts (auto_explorer, etc.)
pkill -9 -f auto_explorer.py
pkill -9 -f simple_explorer.py
pkill -9 -f random_explorer.py
pkill -9 -f python3

# Kill any remaining ROS2 processes
pkill -9 -f ros2
pkill -9 -f _ros2_daemon

# Wait a moment
sleep 1

echo "✅ All processes killed!"
