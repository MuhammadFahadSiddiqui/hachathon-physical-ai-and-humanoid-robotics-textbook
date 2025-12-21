#!/usr/bin/env python3
"""
Gazebo Demo Launch File for Module 2 - Chapter 1

This launch file demonstrates physics-based simulation of the simple_humanoid
robot in Gazebo Classic. It starts the Gazebo simulator with a custom world file
and spawns the humanoid robot with realistic physics properties.

Usage:
    ros2 launch gazebo_demo.launch.py

Prerequisites:
    - ROS 2 Humble installed
    - Gazebo Classic 11 installed
    - gazebo_ros_pkgs installed (sudo apt install ros-humble-gazebo-ros-pkgs)

Author: AI-Driven Robotics Textbook
Date: 2025-12-19
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Generate the launch description for Gazebo humanoid simulation.

    Returns:
        LaunchDescription: Complete launch configuration
    """

    # Get the directory containing this launch file
    launch_file_dir = os.path.dirname(os.path.realpath(__file__))
    example_dir = os.path.dirname(launch_file_dir)

    # Paths to world file and URDF file (relative to launch file)
    world_file = os.path.join(example_dir, 'simple_humanoid.world')
    urdf_file = os.path.join(example_dir, 'simple_humanoid_gazebo.urdf')

    # Verify files exist
    if not os.path.exists(world_file):
        raise FileNotFoundError(f"World file not found: {world_file}")
    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"URDF file not found: {urdf_file}")

    # Read URDF file content for robot_description parameter
    with open(urdf_file, 'r') as urdf:
        robot_description = urdf.read()

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Start Gazebo server (physics simulation, no GUI by default)
    # To run with GUI, add '-g' flag or use 'gazebo' instead of 'gzserver'
    start_gazebo_server = ExecuteProcess(
        cmd=[
            'gazebo',  # Use 'gazebo' for GUI, 'gzserver' for headless
            '--verbose',  # Print detailed logs for debugging
            world_file,  # Load our custom world file
            '-s', 'libgazebo_ros_init.so',  # ROS integration plugin (initialization)
            '-s', 'libgazebo_ros_factory.so'  # ROS integration plugin (spawn models)
        ],
        output='screen',  # Print Gazebo output to terminal
        shell=False
    )

    # Spawn the simple_humanoid robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'simple_humanoid',  # Robot name in Gazebo
            '-file', urdf_file,  # Path to URDF file
            '-x', '0.0',  # Spawn position: x-coordinate (meters)
            '-y', '0.0',  # Spawn position: y-coordinate (meters)
            '-z', '1.0',  # Spawn position: z-coordinate (1 meter above ground)
            '-R', '0.0',  # Roll angle (radians)
            '-P', '0.0',  # Pitch angle (radians)
            '-Y', '0.0',  # Yaw angle (radians)
        ],
        output='screen'
    )

    # Publish robot state to TF tree (transforms between robot links)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )

    # Publish joint states (optional, for RViz visualization)
    # Gazebo automatically publishes joint states via gazebo_ros plugin,
    # but we include this for completeness
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Combine all launch actions
    return LaunchDescription([
        declare_use_sim_time,
        start_gazebo_server,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
    ])


if __name__ == '__main__':
    generate_launch_description()
