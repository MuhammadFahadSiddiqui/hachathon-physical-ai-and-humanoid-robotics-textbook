#!/usr/bin/env python3
"""
Isaac ROS Visual SLAM launch file.

Launches cuVSLAM with stereo camera topics for 30 Hz odometry and point cloud mapping.

Usage:
    ros2 launch isaac_ros_perception visual_slam.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for Visual SLAM."""

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'stereo_image_left',
            default_value='/camera/left/image_raw',
            description='Left stereo camera topic'
        ),
        DeclareLaunchArgument(
            'stereo_image_right',
            default_value='/camera/right/image_raw',
            description='Right stereo camera topic'
        ),

        # Isaac ROS Visual SLAM node
        Node(
            package='isaac_ros_visual_slam',
            executable='isaac_ros_visual_slam',
            name='visual_slam',
            parameters=[{
                'enable_rectified_pose': True,
                'denoise_input_images': False,
                'rectified_images': True,
                'enable_debug_mode': False,
                'debug_dump_path': '/tmp/cuvslam',
                'enable_slam_visualization': True,
                'enable_landmarks_view': True,
                'enable_observations_view': True,
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'input_left_camera_frame': 'left_camera',
                'input_right_camera_frame': 'right_camera',
            }],
            remappings=[
                ('stereo_camera/left/image', LaunchConfiguration('stereo_image_left')),
                ('stereo_camera/right/image', LaunchConfiguration('stereo_image_right')),
                ('stereo_camera/left/camera_info', '/camera/left/camera_info'),
                ('stereo_camera/right/camera_info', '/camera/right/camera_info'),
            ],
            output='screen'
        ),
    ])
