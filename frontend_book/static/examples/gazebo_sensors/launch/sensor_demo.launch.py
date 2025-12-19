#!/usr/bin/env python3
"""
Sensor Demo Launch File for Module 2 - Chapter 3

This launch file demonstrates sensor simulation in Gazebo for the simple_humanoid robot.
It allows students to launch different sensor configurations (LiDAR, depth camera, or IMU)
and visualize sensor data in RViz.

Usage:
    # Launch with LiDAR sensor
    ros2 launch sensor_demo.launch.py sensor_type:=lidar

    # Launch with depth camera
    ros2 launch sensor_demo.launch.py sensor_type:=depth

    # Launch with IMU
    ros2 launch sensor_demo.launch.py sensor_type:=imu

    # Launch with RViz visualization
    ros2 launch sensor_demo.launch.py sensor_type:=lidar use_rviz:=true

Prerequisites:
    - ROS 2 Humble installed
    - Gazebo Classic 11 installed
    - gazebo_ros_pkgs installed

Author: AI-Driven Robotics Textbook
Module: 2 - The Digital Twin (Gazebo & Unity)
Chapter: 3 - Sensor Simulation
Date: 2025-12-19
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate launch description for sensor simulation.

    Launch Arguments:
        sensor_type (str): Type of sensor to simulate ('lidar', 'depth', or 'imu')
        use_rviz (bool): Whether to launch RViz for visualization (default: false)

    Returns:
        LaunchDescription: Complete launch configuration
    """

    # Get directory containing this launch file
    launch_file_dir = os.path.dirname(os.path.realpath(__file__))
    example_dir = os.path.dirname(launch_file_dir)

    # Map sensor type to URDF file
    sensor_urdf_map = {
        'lidar': 'humanoid_with_lidar.urdf',
        'depth': 'humanoid_with_depth_camera.urdf',
        'imu': 'humanoid_with_imu.urdf'
    }

    # Declare launch arguments
    declare_sensor_type = DeclareLaunchArgument(
        'sensor_type',
        default_value='lidar',
        description='Sensor type to simulate: lidar, depth, or imu'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz for sensor visualization'
    )

    # Get launch configuration
    sensor_type = LaunchConfiguration('sensor_type')
    use_rviz = LaunchConfiguration('use_rviz')

    # Determine URDF file based on sensor type
    # This uses PythonExpression to evaluate sensor_type at runtime
    urdf_file_path = PythonExpression([
        "'", example_dir, "/humanoid_with_lidar.urdf' if '", sensor_type, "' == 'lidar' else ",
        "'", example_dir, "/humanoid_with_depth_camera.urdf' if '", sensor_type, "' == 'depth' else ",
        "'", example_dir, "/humanoid_with_imu.urdf'"
    ])

    # Read URDF file for robot_description parameter
    # Note: In production, use xacro or Python substitution to read dynamically
    # For simplicity, we'll spawn the robot using the file path directly

    # Start Gazebo server with empty world
    start_gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    # Spawn robot based on selected sensor type
    spawn_lidar_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'humanoid_lidar',
            '-file', os.path.join(example_dir, 'humanoid_with_lidar.urdf'),
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5',  # Spawn at ground level
        ],
        output='screen',
        condition=IfCondition(PythonExpression(["'", sensor_type, "' == 'lidar'"]))
    )

    spawn_depth_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'humanoid_depth',
            '-file', os.path.join(example_dir, 'humanoid_with_depth_camera.urdf'),
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5',
        ],
        output='screen',
        condition=IfCondition(PythonExpression(["'", sensor_type, "' == 'depth'"]))
    )

    spawn_imu_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'humanoid_imu',
            '-file', os.path.join(example_dir, 'humanoid_with_imu.urdf'),
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5',
        ],
        output='screen',
        condition=IfCondition(PythonExpression(["'", sensor_type, "' == 'imu'"]))
    )

    # Robot state publisher (publishes TF transforms for sensor frames)
    # We need to read URDF for each sensor type
    robot_state_publisher_lidar = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': open(os.path.join(example_dir, 'humanoid_with_lidar.urdf')).read()
        }],
        condition=IfCondition(PythonExpression(["'", sensor_type, "' == 'lidar'"]))
    )

    robot_state_publisher_depth = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': open(os.path.join(example_dir, 'humanoid_with_depth_camera.urdf')).read()
        }],
        condition=IfCondition(PythonExpression(["'", sensor_type, "' == 'depth'"]))
    )

    robot_state_publisher_imu = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': open(os.path.join(example_dir, 'humanoid_with_imu.urdf')).read()
        }],
        condition=IfCondition(PythonExpression(["'", sensor_type, "' == 'imu'"]))
    )

    # Launch RViz with sensor-specific configuration (optional)
    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(example_dir, 'launch', 'sensor_viz.rviz')],
        condition=IfCondition(use_rviz)
    )

    # Combine all launch actions
    return LaunchDescription([
        declare_sensor_type,
        declare_use_rviz,
        start_gazebo,
        spawn_lidar_robot,
        spawn_depth_robot,
        spawn_imu_robot,
        robot_state_publisher_lidar,
        robot_state_publisher_depth,
        robot_state_publisher_imu,
        start_rviz,
    ])


if __name__ == '__main__':
    generate_launch_description()
