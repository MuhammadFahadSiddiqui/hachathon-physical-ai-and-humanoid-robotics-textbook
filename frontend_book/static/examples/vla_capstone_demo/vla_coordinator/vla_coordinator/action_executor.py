#!/usr/bin/env python3
"""
Action Executor - Dispatches primitives to subsystems

Receives action primitives from behavior_coordinator and dispatches them to
appropriate ROS 2 action servers (Nav2, Isaac ROS, Gripper).

Author: Module 4 - Vision-Language-Action (VLA)
License: MIT
"""

import rclpy
from rclpy.node import Node
import time
from typing import Dict, Any


class ActionExecutor(Node):
    """
    Dispatches action primitives to subsystem action servers.

    In a real implementation, this would use ROS 2 action clients to call:
    - Nav2: /navigate_to_pose action
    - Isaac ROS: /detect_objects action
    - Gripper: /gripper_action action

    This simplified version simulates action execution for demonstration.
    """

    def __init__(self):
        super().__init__('action_executor')

        # In real implementation, create action clients here:
        # self.nav2_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        # self.detect_client = ActionClient(self, DetectObjects, '/detect_objects')
        # self.gripper_client = ActionClient(self, GripperCommand, '/gripper_action')

        self.get_logger().info('Action Executor initialized (simulation mode)')

    def execute_action(self, action: Dict[str, Any]) -> bool:
        """
        Execute a single action primitive.

        Args:
            action: Action dictionary with 'action', 'parameters', 'expected_duration'

        Returns:
            bool: True if action succeeded, False otherwise
        """
        action_name = action['action']
        parameters = action['parameters']
        expected_duration = action.get('expected_duration', 5.0)

        self.get_logger().info(f'Executing: {action_name} with params {parameters}')

        # Dispatch to appropriate subsystem
        if action_name == 'navigate':
            return self.execute_navigate(parameters, expected_duration)
        elif action_name == 'detect_object':
            return self.execute_detect_object(parameters, expected_duration)
        elif action_name in ['grasp', 'release', 'open_gripper', 'close_gripper']:
            return self.execute_gripper_action(action_name, parameters, expected_duration)
        elif action_name == 'move_arm':
            return self.execute_move_arm(parameters, expected_duration)
        elif action_name == 'wait':
            return self.execute_wait(parameters['duration'])
        elif action_name == 'say':
            return self.execute_say(parameters['text'])
        elif action_name == 'stop':
            return self.execute_stop()
        else:
            self.get_logger().error(f'Unknown action: {action_name}')
            return False

    def execute_navigate(self, params: Dict[str, Any], duration: float) -> bool:
        """Simulate navigation to target."""
        target = params['target']
        self.get_logger().info(f'Navigating to {target}...')
        time.sleep(duration * 0.2)  # Simulate 20% of expected time
        self.get_logger().info(f'Navigation to {target} complete')
        return True

    def execute_detect_object(self, params: Dict[str, Any], duration: float) -> bool:
        """Simulate object detection."""
        object_class = params['object_class']
        max_results = params.get('max_results', 5)
        self.get_logger().info(f'Detecting {object_class} (max {max_results} results)...')
        time.sleep(duration * 0.2)
        self.get_logger().info(f'Detected 1 {object_class} at [2.3, 1.5, 0.8]')
        return True

    def execute_gripper_action(self, action_name: str, params: Dict[str, Any], duration: float) -> bool:
        """Simulate gripper actions."""
        self.get_logger().info(f'Executing gripper action: {action_name}')
        time.sleep(duration * 0.2)

        if action_name == 'grasp':
            force = params.get('force', 10.0)
            self.get_logger().info(f'Grasp successful with {force}N force')
        elif action_name == 'release':
            self.get_logger().info('Gripper released')
        elif action_name == 'open_gripper':
            width = params.get('width', 0.08)
            self.get_logger().info(f'Gripper opened to {width}m')
        elif action_name == 'close_gripper':
            force = params.get('force', 5.0)
            self.get_logger().info(f'Gripper closed with {force}N force')

        return True

    def execute_move_arm(self, params: Dict[str, Any], duration: float) -> bool:
        """Simulate arm movement."""
        pose_name = params['pose_name']
        self.get_logger().info(f'Moving arm to {pose_name} pose...')
        time.sleep(duration * 0.2)
        self.get_logger().info(f'Arm reached {pose_name} pose')
        return True

    def execute_wait(self, duration: float) -> bool:
        """Execute wait primitive."""
        self.get_logger().info(f'Waiting for {duration}s...')
        time.sleep(duration)
        self.get_logger().info('Wait complete')
        return True

    def execute_say(self, text: str) -> bool:
        """Execute text-to-speech."""
        self.get_logger().info(f'TTS: "{text}"')
        return True

    def execute_stop(self) -> bool:
        """Execute emergency stop."""
        self.get_logger().warn('EMERGENCY STOP')
        return True


def main(args=None):
    """Main entry point for the action executor node."""
    rclpy.init(args=args)

    try:
        node = ActionExecutor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
