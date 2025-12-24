#!/usr/bin/env python3
"""
VLA Capstone Demo Launch File

Launches complete Vision-Language-Action integration with:
- Voice recognition (Whisper from Chapter 1)
- LLM task planning (from Chapter 2)
- Behavior coordination (state machine)
- Action execution (subsystem dispatcher)
- System monitoring (RViz visualization)
- Gripper control (simulated hardware)

This demonstrates the full VLA pipeline:
Voice Command → LLM Planning → Behavior Coordination → Robot Actions

Author: Module 4 - Vision-Language-Action (VLA)
License: MIT
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for VLA capstone demo."""

    # Declare launch arguments
    declare_max_retries = DeclareLaunchArgument(
        'max_retries',
        default_value='2',
        description='Maximum retry attempts for failed actions'
    )

    declare_idle_timeout = DeclareLaunchArgument(
        'idle_timeout',
        default_value='5.0',
        description='Timeout before returning to IDLE state (seconds)'
    )

    declare_gripper_speed = DeclareLaunchArgument(
        'gripper_speed',
        default_value='0.05',
        description='Gripper motion speed (m/s)'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    # Node 1: Behavior Coordinator (State Machine)
    behavior_coordinator_node = Node(
        package='vla_coordinator',
        executable='behavior_coordinator',
        name='behavior_coordinator',
        output='screen',
        parameters=[{
            'max_retries': LaunchConfiguration('max_retries'),
            'idle_timeout': LaunchConfiguration('idle_timeout'),
        }],
        remappings=[
            # Input topics
            ('task_plan', '/llm/task_plan'),
            # Output topics
            ('execution_status', '/vla/execution_status'),
            ('clarification_request', '/vla/clarification_request'),
        ]
    )

    # Node 2: Action Executor (Subsystem Dispatcher)
    action_executor_node = Node(
        package='vla_coordinator',
        executable='action_executor',
        name='action_executor',
        output='screen',
        remappings=[
            # Future: map to real action servers
            # ('/navigate_to_pose', '/nav2/navigate_to_pose'),
            # ('/detect_objects', '/isaac_ros/detect_objects'),
            # ('/gripper_action', '/gripper/gripper_action'),
        ]
    )

    # Node 3: System Monitor (RViz Visualization)
    system_monitor_node = Node(
        package='vla_coordinator',
        executable='system_monitor',
        name='system_monitor',
        output='screen',
        remappings=[
            # Input topics
            ('execution_status', '/vla/execution_status'),
            ('voice_command', '/whisper/voice_command'),
            ('task_plan', '/llm/task_plan'),
            # Output topics
            ('visualization_markers', '/vla/visualization_markers'),
            ('tts_feedback', '/vla/tts_feedback'),
        ]
    )

    # Node 4: Gripper Controller (Simulated Hardware)
    gripper_controller_node = Node(
        package='vla_coordinator',
        executable='gripper_controller',
        name='gripper_controller',
        output='screen',
        parameters=[{
            'gripper_speed': LaunchConfiguration('gripper_speed'),
            'update_rate': 20.0,
        }],
        remappings=[
            # Input topics
            ('gripper_command', '/vla/gripper_command'),
            # Output topics
            ('gripper_state', '/gripper/joint_states'),
            ('gripper_status', '/gripper/status'),
            ('gripper_marker', '/gripper/marker'),
        ]
    )

    # Optional: RViz for visualization
    # Note: Uncomment when RViz config file is available
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', PathJoinSubstitution([
    #         FindPackageShare('vla_coordinator'),
    #         'config',
    #         'rviz_vla_debug.rviz'
    #     ])],
    #     condition=IfCondition(LaunchConfiguration('use_rviz'))
    # )

    # Informational message
    startup_message = LogInfo(
        msg=[
            '\n',
            '═══════════════════════════════════════════════════════════════\n',
            '  VLA Capstone Demo - Vision-Language-Action Integration\n',
            '═══════════════════════════════════════════════════════════════\n',
            '\n',
            'Launched nodes:\n',
            '  1. behavior_coordinator  - Task execution state machine\n',
            '  2. action_executor       - Subsystem action dispatcher\n',
            '  3. system_monitor        - RViz visualization & TTS\n',
            '  4. gripper_controller    - Simulated gripper hardware\n',
            '\n',
            'Key topics:\n',
            '  Input:  /whisper/voice_command  - Voice commands (Ch1)\n',
            '  Input:  /llm/task_plan          - LLM-generated plans (Ch2)\n',
            '  Output: /vla/execution_status   - Current task state\n',
            '  Output: /vla/visualization_markers - RViz markers\n',
            '\n',
            'To test the system:\n',
            '  1. Publish voice command:\n',
            '     ros2 topic pub /whisper/voice_command std_msgs/String \\\n',
            '       \'data: "Navigate to the kitchen"\'\n',
            '\n',
            '  2. Publish task plan:\n',
            '     ros2 topic pub /llm/task_plan std_msgs/String \\\n',
            '       \'data: "{\\\"plan\\\": [{\\\"action\\\": \\\"navigate\\\", \n',
            '                \\\"parameters\\\": {\\\"target\\\": \\\"kitchen\\\"}, \n',
            '                \\\"expected_duration\\\": 10.0}]}"\'\n',
            '\n',
            '  3. Monitor execution:\n',
            '     ros2 topic echo /vla/execution_status\n',
            '\n',
            'For full integration, launch Chapter 1 & 2 nodes separately:\n',
            '  ros2 launch whisper_ros whisper_node.launch.py\n',
            '  ros2 launch llm_task_planner llm_planner.launch.py\n',
            '\n',
            '═══════════════════════════════════════════════════════════════\n'
        ]
    )

    return LaunchDescription([
        # Launch arguments
        declare_max_retries,
        declare_idle_timeout,
        declare_gripper_speed,
        declare_use_rviz,

        # Startup message
        startup_message,

        # Core VLA nodes
        behavior_coordinator_node,
        action_executor_node,
        system_monitor_node,
        gripper_controller_node,

        # Optional visualization
        # rviz_node,
    ])
