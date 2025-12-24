#!/usr/bin/env python3
"""
Launch file for LLM task planner.

Starts the planner_node with configurable LLM backend (OpenAI or Ollama).

Usage:
    ros2 launch llm_planner llm_planner.launch.py
    ros2 launch llm_planner llm_planner.launch.py llm_backend:=openai model:=gpt-4-turbo
    ros2 launch llm_planner llm_planner.launch.py llm_backend:=ollama model:=llama3.1:8b

Author: Module 4 - Vision-Language-Action (VLA)
License: MIT
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    """Generate launch description for LLM planner."""

    # Get OpenAI API key from environment
    openai_api_key = os.getenv('OPENAI_API_KEY', '')

    # Declare launch arguments
    llm_backend_arg = DeclareLaunchArgument(
        'llm_backend',
        default_value='ollama',
        description='LLM backend: "openai" or "ollama"'
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='llama3.1:8b',
        description='Model name (e.g., "gpt-4-turbo", "llama3.1:8b")'
    )

    temperature_arg = DeclareLaunchArgument(
        'temperature',
        default_value='0.0',
        description='LLM temperature (0.0 = deterministic, 1.0 = creative)'
    )

    max_tokens_arg = DeclareLaunchArgument(
        'max_tokens',
        default_value='500',
        description='Maximum response tokens'
    )

    timeout_arg = DeclareLaunchArgument(
        'timeout',
        default_value='30.0',
        description='API call timeout in seconds'
    )

    ollama_host_arg = DeclareLaunchArgument(
        'ollama_host',
        default_value='http://localhost:11434',
        description='Ollama server URL'
    )

    max_retries_arg = DeclareLaunchArgument(
        'max_retries',
        default_value='3',
        description='Max retries for invalid plans'
    )

    # LLM planner node
    planner_node = Node(
        package='llm_planner',
        executable='planner_node',
        name='llm_planner_node',
        output='screen',
        parameters=[{
            'llm_backend': LaunchConfiguration('llm_backend'),
            'model': LaunchConfiguration('model'),
            'temperature': LaunchConfiguration('temperature'),
            'max_tokens': LaunchConfiguration('max_tokens'),
            'timeout': LaunchConfiguration('timeout'),
            'openai_api_key': openai_api_key,
            'ollama_host': LaunchConfiguration('ollama_host'),
            'max_retries': LaunchConfiguration('max_retries'),
        }],
        remappings=[
            # Add remappings here if needed
        ]
    )

    return LaunchDescription([
        llm_backend_arg,
        model_arg,
        temperature_arg,
        max_tokens_arg,
        timeout_arg,
        ollama_host_arg,
        max_retries_arg,
        planner_node,
    ])
