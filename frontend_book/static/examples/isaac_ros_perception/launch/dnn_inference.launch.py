#!/usr/bin/env python3
"""
Isaac ROS DNN Inference launch file for YOLOv8 TensorRT.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_file_path', description='Path to TensorRT engine'),
        DeclareLaunchArgument('input_image_topic', default_value='/camera/rgb/image_raw'),

        Node(
            package='isaac_ros_dnn_inference',
            executable='isaac_ros_dnn_inference',
            name='dnn_inference',
            parameters=[{
                'model_file_path': LaunchConfiguration('model_file_path'),
                'engine_file_path': LaunchConfiguration('model_file_path'),
                'input_tensor_names': ['images'],
                'input_binding_names': ['images'],
                'output_tensor_names': ['output0'],
                'output_binding_names': ['output0'],
                'verbose': False,
                'force_engine_update': False,
            }],
            remappings=[
                ('tensor_pub', '/detections'),
                ('image', LaunchConfiguration('input_image_topic')),
            ],
        ),
    ])
