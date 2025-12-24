#!/usr/bin/env python3
"""
System Monitor - RViz Visualization and Progress Tracking

Subscribes to execution status, voice commands, and task plans.
Publishes RViz markers for visual feedback and simulates TTS.

Author: Module 4 - Vision-Language-Action (VLA)
License: MIT
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import json
from typing import Optional, Dict, Any


class SystemMonitor(Node):
    """
    Visualizes VLA system state in RViz and provides TTS feedback.

    Subscribes to:
        /execution_status (std_msgs/String): Current task execution state
        /voice_command (std_msgs/String): Recognized voice commands
        /task_plan (std_msgs/String): Generated task plans from LLM

    Publishes:
        /visualization_markers (visualization_msgs/MarkerArray): RViz overlays
        /tts_feedback (std_msgs/String): Simulated text-to-speech

    Features:
        - Progress bar visualization
        - State indicator (color-coded sphere)
        - Text overlays for commands, plans, and status
        - TTS feedback on state transitions
    """

    def __init__(self):
        super().__init__('system_monitor')

        # Current state tracking
        self.current_state = "IDLE"
        self.current_action = None
        self.progress = 0
        self.total_actions = 0
        self.elapsed_time = 0.0
        self.estimated_remaining = 0.0

        # Recent messages
        self.last_voice_command = None
        self.last_task_plan = None

        # Subscribers
        self.status_sub = self.create_subscription(
            String,
            'execution_status',
            self.status_callback,
            10
        )

        self.voice_sub = self.create_subscription(
            String,
            'voice_command',
            self.voice_callback,
            10
        )

        self.plan_sub = self.create_subscription(
            String,
            'task_plan',
            self.plan_callback,
            10
        )

        # Publishers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            'visualization_markers',
            10
        )

        self.tts_pub = self.create_publisher(
            String,
            'tts_feedback',
            10
        )

        # Visualization update timer (10 Hz)
        self.viz_timer = self.create_timer(0.1, self.publish_markers)

        # State for TTS (avoid repeating same message)
        self.last_tts_state = None

        self.get_logger().info('System Monitor initialized')
        self.get_logger().info('Publishing RViz markers on /visualization_markers')

    def status_callback(self, msg: String):
        """Process execution status updates."""
        try:
            status = json.loads(msg.data)

            # Update state tracking
            old_state = self.current_state
            self.current_state = status.get('state', 'IDLE')
            self.current_action = status.get('current_action')
            self.progress = status.get('progress', 0)
            self.total_actions = status.get('total_actions', 0)
            self.elapsed_time = status.get('elapsed_time', 0.0)
            self.estimated_remaining = status.get('estimated_remaining', 0.0)

            # Log state transitions
            if old_state != self.current_state:
                self.get_logger().info(f'State transition: {old_state} → {self.current_state}')
                self.publish_tts_feedback()

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in execution_status: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing status: {e}')

    def voice_callback(self, msg: String):
        """Process voice command updates."""
        try:
            voice_data = json.loads(msg.data)
            self.last_voice_command = voice_data.get('text', msg.data)
            self.get_logger().info(f'Voice command received: "{self.last_voice_command}"')
        except json.JSONDecodeError:
            # Not JSON, treat as plain text
            self.last_voice_command = msg.data
            self.get_logger().info(f'Voice command received: "{self.last_voice_command}"')

    def plan_callback(self, msg: String):
        """Process task plan updates."""
        try:
            plan_data = json.loads(msg.data)

            if 'error' in plan_data:
                self.last_task_plan = f"ERROR: {plan_data['error']}"
            else:
                plan = plan_data.get('plan', [])
                self.last_task_plan = f"{len(plan)} actions planned"

            self.get_logger().info(f'Task plan received: {self.last_task_plan}')
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in task_plan: {e}')

    def publish_tts_feedback(self):
        """Publish TTS feedback on state changes."""
        # Avoid repeating same state
        if self.last_tts_state == self.current_state:
            return

        self.last_tts_state = self.current_state

        # Generate appropriate feedback
        feedback_text = ""
        if self.current_state == "PLANNING":
            feedback_text = "Planning task sequence"
        elif self.current_state == "EXECUTING":
            feedback_text = f"Executing task: {self.current_action or 'unknown'}"
        elif self.current_state == "COMPLETED":
            feedback_text = f"Task completed successfully in {self.elapsed_time:.1f} seconds"
        elif self.current_state == "FAILED":
            feedback_text = "Task failed. Requesting clarification."
        elif self.current_state == "IDLE":
            feedback_text = "Ready for next command"

        if feedback_text:
            msg = String()
            msg.data = feedback_text
            self.tts_pub.publish(msg)
            self.get_logger().info(f'TTS: "{feedback_text}"')

    def publish_markers(self):
        """Publish RViz visualization markers."""
        marker_array = MarkerArray()

        # Marker 0: State indicator sphere (color-coded)
        state_marker = self.create_state_indicator()
        marker_array.markers.append(state_marker)

        # Marker 1: Progress bar
        progress_marker = self.create_progress_bar()
        marker_array.markers.append(progress_marker)

        # Marker 2: State text overlay
        state_text = self.create_state_text()
        marker_array.markers.append(state_text)

        # Marker 3: Voice command text
        voice_text = self.create_voice_command_text()
        marker_array.markers.append(voice_text)

        # Marker 4: Task plan text
        plan_text = self.create_task_plan_text()
        marker_array.markers.append(plan_text)

        # Marker 5: Progress text (action X/Y)
        progress_text = self.create_progress_text()
        marker_array.markers.append(progress_text)

        self.marker_pub.publish(marker_array)

    def create_state_indicator(self) -> Marker:
        """Create color-coded sphere indicating current state."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "vla_state"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Position above robot
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 2.5
        marker.pose.orientation.w = 1.0

        # Size
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        # Color based on state
        if self.current_state == "IDLE":
            marker.color.r = 0.5
            marker.color.g = 0.5
            marker.color.b = 0.5
            marker.color.a = 0.8
        elif self.current_state == "PLANNING":
            marker.color.r = 1.0
            marker.color.g = 0.8
            marker.color.b = 0.0
            marker.color.a = 0.9
        elif self.current_state == "EXECUTING":
            marker.color.r = 0.0
            marker.color.g = 0.8
            marker.color.b = 1.0
            marker.color.a = 0.9
        elif self.current_state == "COMPLETED":
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.9
        elif self.current_state == "FAILED":
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.9

        return marker

    def create_progress_bar(self) -> Marker:
        """Create progress bar visualization."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "vla_progress"
        marker.id = 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 2.0
        marker.pose.orientation.w = 1.0

        # Calculate progress percentage
        if self.total_actions > 0:
            progress_pct = self.progress / self.total_actions
        else:
            progress_pct = 0.0

        # Scale bar width based on progress
        marker.scale.x = 2.0 * progress_pct  # Max 2m wide
        marker.scale.y = 0.1
        marker.scale.z = 0.05

        # Color: green for progress
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.7

        return marker

    def create_state_text(self) -> Marker:
        """Create text showing current state."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "vla_text"
        marker.id = 2
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # Position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 3.0
        marker.pose.orientation.w = 1.0

        # Text content
        if self.current_action:
            marker.text = f"State: {self.current_state} | Action: {self.current_action}"
        else:
            marker.text = f"State: {self.current_state}"

        # Size and color
        marker.scale.z = 0.2  # Text height
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        return marker

    def create_voice_command_text(self) -> Marker:
        """Create text showing last voice command."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "vla_text"
        marker.id = 3
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # Position
        marker.pose.position.x = -2.0
        marker.pose.position.y = 2.0
        marker.pose.position.z = 2.5
        marker.pose.orientation.w = 1.0

        # Text content
        if self.last_voice_command:
            marker.text = f"Voice: \"{self.last_voice_command}\""
        else:
            marker.text = "Voice: (awaiting command)"

        # Size and color
        marker.scale.z = 0.15
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.9

        return marker

    def create_task_plan_text(self) -> Marker:
        """Create text showing task plan summary."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "vla_text"
        marker.id = 4
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # Position
        marker.pose.position.x = 2.0
        marker.pose.position.y = 2.0
        marker.pose.position.z = 2.5
        marker.pose.orientation.w = 1.0

        # Text content
        if self.last_task_plan:
            marker.text = f"Plan: {self.last_task_plan}"
        else:
            marker.text = "Plan: (no plan yet)"

        # Size and color
        marker.scale.z = 0.15
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.9

        return marker

    def create_progress_text(self) -> Marker:
        """Create text showing progress and timing."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "vla_text"
        marker.id = 5
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # Position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.7
        marker.pose.orientation.w = 1.0

        # Text content
        if self.total_actions > 0:
            marker.text = (
                f"Progress: {self.progress}/{self.total_actions} | "
                f"Elapsed: {self.elapsed_time:.1f}s | "
                f"Remaining: ~{self.estimated_remaining:.1f}s"
            )
        else:
            marker.text = "Progress: 0/0 | Idle"

        # Size and color
        marker.scale.z = 0.12
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.8

        return marker


def main(args=None):
    """Main entry point for the system monitor node."""
    rclpy.init(args=args)

    try:
        node = SystemMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
