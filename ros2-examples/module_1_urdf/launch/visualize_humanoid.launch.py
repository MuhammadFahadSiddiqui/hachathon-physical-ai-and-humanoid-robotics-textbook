#!/usr/bin/env python3
"""
Launch file for visualizing simple_humanoid.urdf in RViz

This launch file starts three nodes:
1. robot_state_publisher - Publishes TF transforms based on URDF
2. joint_state_publisher_gui - GUI for controlling joint angles
3. rviz2 - 3D visualization tool

Usage:
    ros2 launch module_1_urdf visualize_humanoid.launch.py

Learning objectives:
- Understand launch file structure
- Configure robot_state_publisher with URDF parameter
- Integrate multiple nodes for robot visualization

Author: Physical AI & Humanoid Robotics Textbook
License: Apache 2.0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate launch description for humanoid visualization.

    Returns:
        LaunchDescription: Launch configuration with all nodes
    """

    # Path to URDF file (relative to this launch file)
    urdf_file = os.path.join(
        os.path.dirname(__file__),
        '..',
        'simple_humanoid.urdf'
    )

    # Read URDF file contents
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Path to RViz config file (optional)
    rviz_config_file = os.path.join(
        os.path.dirname(__file__),
        '..',
        'humanoid.rviz'
    )

    # Check if RViz config exists
    use_rviz_config = os.path.exists(rviz_config_file)

    # Nodes to launch
    nodes = [

        # ===== ROBOT STATE PUBLISHER =====
        # Publishes TF transforms for all links based on URDF and joint states
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False  # Use wall clock time (not simulation time)
            }]
        ),

        # ===== JOINT STATE PUBLISHER GUI =====
        # Provides GUI with sliders to manually control joint positions
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # ===== RVIZ2 =====
        # 3D visualization tool
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            # Load RViz config if it exists, otherwise use default
            arguments=['-d', rviz_config_file] if use_rviz_config else []
        ),
    ]

    return LaunchDescription(nodes)
