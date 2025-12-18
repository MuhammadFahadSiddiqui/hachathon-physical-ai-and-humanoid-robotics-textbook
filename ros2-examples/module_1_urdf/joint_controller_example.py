#!/usr/bin/env python3
"""
Joint Controller Example - Programmatic Joint Control

This node demonstrates how to publish joint states to control
the simple humanoid robot. It animates the arm joints in a wave motion.

Learning objectives:
- Publish sensor_msgs/JointState messages
- Control robot joints programmatically
- Use timers for periodic updates
- Implement smooth animation with sine waves

Usage:
    python3 joint_controller_example.py

Prerequisites:
    - robot_state_publisher must be running
    - URDF must be loaded with simple_humanoid

Author: Physical AI & Humanoid Robotics Textbook
License: Apache 2.0
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math


class JointController(Node):
    """
    A node that publishes joint states to animate the simple humanoid.

    This controller demonstrates programmatic joint control by publishing
    JointState messages with animated joint positions. The arms wave
    up and down using sine wave functions.
    """

    def __init__(self):
        """
        Initialize the joint controller.

        Creates a publisher for joint states and a timer for periodic updates.
        """
        super().__init__('joint_controller')

        # ===== PUBLISHER =====

        # Publish joint states to /joint_states topic
        # robot_state_publisher subscribes to this topic
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

        # ===== TIMER =====

        # Update joint positions at 20 Hz (every 0.05 seconds)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.publish_joint_states)

        # ===== STATE =====

        # Animation time (increments with each update)
        self.time = 0.0

        # Time increment per update
        self.dt = timer_period

        # Log startup
        self.get_logger().info('Joint controller started')
        self.get_logger().info(f'Publishing to: /joint_states at {1/timer_period} Hz')
        self.get_logger().info('Animating simple_humanoid arms...')

    def publish_joint_states(self):
        """
        Timer callback that publishes joint states with animated positions.

        This method:
        1. Creates a JointState message
        2. Sets joint names (must match URDF)
        3. Calculates joint positions using sine waves
        4. Publishes the message
        5. Increments animation time
        """

        # Create JointState message
        msg = JointState()

        # ===== HEADER =====

        # Set timestamp to current time
        msg.header.stamp = self.get_clock().now().to_msg()

        # ===== JOINT NAMES =====

        # Define joint names (must exactly match names in URDF)
        msg.name = [
            'left_shoulder_joint',
            'right_shoulder_joint',
            'left_elbow_joint',
            'right_elbow_joint'
        ]

        # ===== JOINT POSITIONS =====

        # Animate joints with sine waves for smooth motion

        # Shoulders: Oscillate up and down (±0.5 radians ≈ ±29 degrees)
        # Both shoulders move in sync
        shoulder_angle = 0.5 * math.sin(self.time)

        # Elbows: Bend and extend (0 to 1 radian ≈ 0 to 57 degrees)
        # Phase-shifted by π to create a wave effect
        elbow_angle = 0.5 * (1 + math.sin(self.time + math.pi))

        # Set joint positions (order must match names above)
        msg.position = [
            shoulder_angle,   # left_shoulder_joint
            shoulder_angle,   # right_shoulder_joint
            elbow_angle,      # left_elbow_joint
            elbow_angle       # right_elbow_joint
        ]

        # ===== OPTIONAL: VELOCITIES AND EFFORTS =====

        # Not required for visualization, but can be included for completeness
        # msg.velocity = [0.0, 0.0, 0.0, 0.0]
        # msg.effort = [0.0, 0.0, 0.0, 0.0]

        # ===== PUBLISH =====

        self.publisher.publish(msg)

        # Log current state every 2 seconds
        if int(self.time / 2.0) != int((self.time - self.dt) / 2.0):
            self.get_logger().info(
                f'Time: {self.time:.1f}s | '
                f'Shoulder: {math.degrees(shoulder_angle):.1f}° | '
                f'Elbow: {math.degrees(elbow_angle):.1f}°'
            )

        # Increment animation time
        self.time += self.dt


def main(args=None):
    """
    Main entry point for the joint controller node.

    Args:
        args: Command-line arguments
    """
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create the joint controller node
    node = JointController()

    # Spin the node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down')

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


# Run main if executed directly
if __name__ == '__main__':
    main()
