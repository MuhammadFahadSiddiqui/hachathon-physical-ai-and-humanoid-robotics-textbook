#!/usr/bin/env python3
"""
Nav2 Bipedal Humanoid Launch File.

Launches Nav2 navigation stack configured for bipedal humanoid robot with:
- DWB local planner with circular footprint (radius 0.3m)
- Global costmap for static obstacles
- Local costmap with rolling window for dynamic obstacles
- Behavior tree navigator

Usage:
    ros2 launch nav2_bipedal_humanoid nav2_humanoid.launch.py
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for Nav2 with bipedal humanoid."""

    # Get package directory
    pkg_dir = get_package_share_directory('nav2_bipedal_humanoid')

    # Paths to configuration files
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    costmap_params_file = os.path.join(pkg_dir, 'config', 'costmap_params.yaml')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    params_file = LaunchConfiguration('params_file', default=nav2_params_file)

    # Nav2 bringup launch
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=nav2_params_file,
            description='Full path to the ROS2 parameters file for Nav2'
        ),

        nav2_launch,
    ])
