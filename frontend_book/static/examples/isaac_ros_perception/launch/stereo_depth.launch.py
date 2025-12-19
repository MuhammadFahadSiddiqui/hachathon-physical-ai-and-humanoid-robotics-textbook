#!/usr/bin/env python3
"""Isaac ROS Stereo Depth launch file."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='isaac_ros_stereo_image_proc',
            executable='disparity_node',
            name='disparity',
            parameters=[{
                'backends': 'CUDA',
                'max_disparity': 64.0,
            }],
            remappings=[
                ('left/image_rect', '/camera/left/image_raw'),
                ('right/image_rect', '/camera/right/image_raw'),
                ('left/camera_info', '/camera/left/camera_info'),
                ('right/camera_info', '/camera/right/camera_info'),
            ],
        ),
    ])
