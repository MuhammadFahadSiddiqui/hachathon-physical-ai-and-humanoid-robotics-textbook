#!/usr/bin/env python3
"""
Launch file for Whisper voice command recognition system.

This launch file starts the voice_command_node with configurable parameters
for model size, confidence threshold, and microphone settings.

Usage:
    ros2 launch whisper_ros2 voice_interface.launch.py
    ros2 launch whisper_ros2 voice_interface.launch.py model_size:=small
    ros2 launch whisper_ros2 voice_interface.launch.py confidence_threshold:=0.8

Author: Module 4 - Vision-Language-Action (VLA)
License: MIT
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for voice command system."""

    # Declare launch arguments
    model_size_arg = DeclareLaunchArgument(
        'model_size',
        default_value='base',
        description='Whisper model size (tiny, base, small, medium, large)'
    )

    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.7',
        description='Minimum confidence threshold for publishing commands (0.0-1.0)'
    )

    sample_rate_arg = DeclareLaunchArgument(
        'sample_rate',
        default_value='16000',
        description='Audio sample rate in Hz'
    )

    device_index_arg = DeclareLaunchArgument(
        'device_index',
        default_value='-1',
        description='Microphone device index (-1 for default)'
    )

    recording_duration_arg = DeclareLaunchArgument(
        'recording_duration',
        default_value='3.0',
        description='Duration of each recording in seconds'
    )

    # Voice command node
    voice_command_node = Node(
        package='whisper_ros2',
        executable='voice_command_node',
        name='voice_command_node',
        output='screen',
        parameters=[{
            'model_size': LaunchConfiguration('model_size'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'sample_rate': LaunchConfiguration('sample_rate'),
            'device_index': LaunchConfiguration('device_index'),
            'recording_duration': LaunchConfiguration('recording_duration'),
        }],
        remappings=[
            # Add remappings here if needed
        ]
    )

    return LaunchDescription([
        model_size_arg,
        confidence_threshold_arg,
        sample_rate_arg,
        device_index_arg,
        recording_duration_arg,
        voice_command_node,
    ])
