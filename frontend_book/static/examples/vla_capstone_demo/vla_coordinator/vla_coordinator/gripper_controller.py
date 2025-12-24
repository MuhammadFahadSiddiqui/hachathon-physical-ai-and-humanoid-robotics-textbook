#!/usr/bin/env python3
"""
Gripper Controller - Simulated Gripper Action Server

Provides a simple gripper action server for the VLA capstone demo.
In a real system, this would interface with hardware drivers or
Isaac Sim gripper controllers.

Author: Module 4 - Vision-Language-Action (VLA)
License: MIT
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
import json
import time
from typing import Optional


class GripperController(Node):
    """
    Simulated gripper controller with action server interface.

    In a real implementation, this would use ROS 2 actions:
        - Action Type: control_msgs/GripperCommand
        - Provides feedback on gripper position and force

    This simplified version:
        - Subscribes to /gripper_command (std_msgs/String - JSON)
        - Publishes /gripper_state (sensor_msgs/JointState)
        - Publishes /gripper_status (std_msgs/String - JSON)
        - Publishes /gripper_marker (visualization_msgs/Marker)

    Gripper Commands (JSON):
        {"action": "open", "width": 0.08}
        {"action": "close", "force": 10.0}
        {"action": "grasp", "force": 15.0}
        {"action": "release"}
    """

    # Gripper physical limits
    MIN_WIDTH = 0.0    # Fully closed (meters)
    MAX_WIDTH = 0.10   # Fully open (meters)
    MAX_FORCE = 50.0   # Maximum grip force (Newtons)

    def __init__(self):
        super().__init__('gripper_controller')

        # Gripper state
        self.current_width = self.MAX_WIDTH  # Start open
        self.current_force = 0.0
        self.target_width = self.MAX_WIDTH
        self.is_grasping = False
        self.object_detected = False

        # Motion parameters
        self.declare_parameter('gripper_speed', 0.05)  # m/s
        self.declare_parameter('update_rate', 20.0)    # Hz

        self.gripper_speed = self.get_parameter('gripper_speed').value
        self.update_rate = self.get_parameter('update_rate').value

        # Subscribe to gripper commands
        self.command_sub = self.create_subscription(
            String,
            'gripper_command',
            self.command_callback,
            10
        )

        # Publishers
        self.state_pub = self.create_publisher(
            JointState,
            'gripper_state',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            'gripper_status',
            10
        )

        self.marker_pub = self.create_publisher(
            Marker,
            'gripper_marker',
            10
        )

        # Update timer
        self.update_timer = self.create_timer(
            1.0 / self.update_rate,
            self.update_gripper
        )

        # Status publishing timer (2 Hz)
        self.status_timer = self.create_timer(0.5, self.publish_status)

        self.get_logger().info(
            f'Gripper Controller initialized: speed={self.gripper_speed}m/s, '
            f'rate={self.update_rate}Hz'
        )
        self.get_logger().info('Listening for commands on /gripper_command')

    def command_callback(self, msg: String):
        """Process gripper commands."""
        try:
            command = json.loads(msg.data)
            action = command.get('action', '')

            if action == 'open':
                width = command.get('width', self.MAX_WIDTH)
                self.open_gripper(width)
            elif action == 'close':
                force = command.get('force', 10.0)
                self.close_gripper(force)
            elif action == 'grasp':
                force = command.get('force', 15.0)
                self.grasp_object(force)
            elif action == 'release':
                self.release_object()
            else:
                self.get_logger().warn(f'Unknown gripper action: {action}')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in gripper_command: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing gripper command: {e}')

    def open_gripper(self, width: float):
        """Open gripper to specified width."""
        width = max(self.MIN_WIDTH, min(width, self.MAX_WIDTH))
        self.target_width = width
        self.is_grasping = False
        self.current_force = 0.0
        self.get_logger().info(f'Opening gripper to {width:.3f}m')

    def close_gripper(self, force: float):
        """Close gripper with specified force."""
        force = min(force, self.MAX_FORCE)
        self.target_width = self.MIN_WIDTH
        self.current_force = force
        self.is_grasping = False
        self.get_logger().info(f'Closing gripper with {force:.1f}N force')

    def grasp_object(self, force: float):
        """Grasp object with specified force."""
        force = min(force, self.MAX_FORCE)
        self.target_width = 0.03  # Typical grasp width
        self.current_force = force
        self.is_grasping = True
        self.object_detected = True
        self.get_logger().info(f'Grasping object with {force:.1f}N force')

    def release_object(self):
        """Release grasped object."""
        self.target_width = self.MAX_WIDTH
        self.current_force = 0.0
        self.is_grasping = False
        self.object_detected = False
        self.get_logger().info('Releasing object')

    def update_gripper(self):
        """Update gripper state (motion simulation)."""
        # Simulate gripper motion toward target
        if abs(self.current_width - self.target_width) > 0.001:
            dt = 1.0 / self.update_rate
            max_delta = self.gripper_speed * dt

            if self.current_width < self.target_width:
                # Opening
                self.current_width = min(
                    self.current_width + max_delta,
                    self.target_width
                )
            else:
                # Closing
                self.current_width = max(
                    self.current_width - max_delta,
                    self.target_width
                )

        # Publish joint state
        self.publish_joint_state()

        # Publish visualization marker
        self.publish_marker()

    def publish_joint_state(self):
        """Publish gripper joint state."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gripper_base"

        # Two-finger gripper (symmetric)
        msg.name = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        half_width = self.current_width / 2.0
        msg.position = [half_width, -half_width]  # Symmetric
        msg.velocity = [0.0, 0.0]
        msg.effort = [self.current_force, self.current_force]

        self.state_pub.publish(msg)

    def publish_status(self):
        """Publish gripper status (JSON)."""
        status = {
            'width': round(self.current_width, 4),
            'force': round(self.current_force, 2),
            'is_grasping': self.is_grasping,
            'object_detected': self.object_detected,
            'at_target': abs(self.current_width - self.target_width) < 0.001
        }

        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    def publish_marker(self):
        """Publish RViz visualization marker for gripper."""
        marker = Marker()
        marker.header.frame_id = "gripper_base"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "gripper"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        # Draw gripper fingers as lines
        # Left finger
        left_base = [0.0, self.current_width / 2.0, 0.0]
        left_tip = [0.05, self.current_width / 2.0, 0.0]

        # Right finger
        right_base = [0.0, -self.current_width / 2.0, 0.0]
        right_tip = [0.05, -self.current_width / 2.0, 0.0]

        # Add points for LINE_LIST
        for coords in [left_base, left_tip, right_base, right_tip]:
            p = self._make_point(coords)
            marker.points.append(p)

        # Scale (line width)
        marker.scale.x = 0.01

        # Color based on state
        if self.is_grasping:
            # Green when grasping
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        elif self.current_width < 0.02:
            # Red when fully closed
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        else:
            # Blue when open
            marker.color.r = 0.0
            marker.color.g = 0.5
            marker.color.b = 1.0

        marker.color.a = 1.0

        self.marker_pub.publish(marker)

    def _make_point(self, coords):
        """Helper to create Point from [x, y, z]."""
        p = Point()
        p.x = float(coords[0])
        p.y = float(coords[1])
        p.z = float(coords[2])
        return p


def main(args=None):
    """Main entry point for the gripper controller node."""
    rclpy.init(args=args)

    try:
        node = GripperController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
