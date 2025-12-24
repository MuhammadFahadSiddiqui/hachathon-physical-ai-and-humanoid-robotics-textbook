#!/usr/bin/env python3
"""
Behavior Coordinator - State Machine for VLA Task Execution

Orchestrates multi-step task execution using a finite state machine.
Subscribes to task plans from LLM, executes actions sequentially via
action_executor, handles failures with retry logic.

Author: Module 4 - Vision-Language-Action (VLA)
License: MIT
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
from enum import Enum
from typing import List, Dict, Any, Optional


class RobotState(Enum):
    """Finite state machine states."""
    IDLE = 0
    PLANNING = 1
    EXECUTING = 2
    COMPLETED = 3
    FAILED = 4


class BehaviorCoordinator(Node):
    """
    Orchestrates VLA task execution using state machine pattern.

    States:
        IDLE: Waiting for task plan
        PLANNING: Validating received plan
        EXECUTING: Running action sequence
        COMPLETED: Task successful
        FAILED: Task failed (triggers retry or replan)

    Topics:
        Subscribers:
            /task_plan (std_msgs/String): JSON action sequence from LLM
        Publishers:
            /execution_status (std_msgs/String): Current state and progress
            /clarification_request (std_msgs/String): Request user input on failure
    """

    def __init__(self):
        super().__init__('behavior_coordinator')

        # State machine
        self.state = RobotState.IDLE
        self.current_plan = []
        self.current_action_index = 0
        self.retry_count = 0

        # Parameters
        self.declare_parameter('max_retries', 2)
        self.declare_parameter('idle_timeout', 5.0)

        self.max_retries = self.get_parameter('max_retries').value
        self.idle_timeout = self.get_parameter('idle_timeout').value

        # Subscribe to task plans
        self.plan_sub = self.create_subscription(
            String,
            'task_plan',
            self.plan_callback,
            10
        )

        # Publish execution status
        self.status_pub = self.create_publisher(String, 'execution_status', 10)

        # Publish clarification requests
        self.clarify_pub = self.create_publisher(String, 'clarification_request', 10)

        # Status publishing timer (500ms)
        self.status_timer = self.create_timer(0.5, self.publish_status)

        # Task start time
        self.task_start_time = None
        self.current_action_start_time = None

        self.get_logger().info(
            f'Behavior Coordinator started: max_retries={self.max_retries}, '
            f'idle_timeout={self.idle_timeout}s'
        )
        self.get_logger().info('State: IDLE (waiting for task plan)')

    def plan_callback(self, msg: String):
        """Receive new task plan from LLM."""
        if self.state != RobotState.IDLE:
            self.get_logger().warn(
                f'Busy executing task (state={self.state.name}), ignoring new plan'
            )
            return

        try:
            plan_data = json.loads(msg.data)

            # Handle error responses from LLM
            if 'error' in plan_data:
                self.get_logger().error(f'LLM error: {plan_data["error"]}')
                return

            self.current_plan = plan_data.get('plan', [])

            if not self.current_plan:
                self.get_logger().error('Received empty plan')
                return

            self.get_logger().info(
                f'Received task plan with {len(self.current_plan)} actions'
            )

            self.state = RobotState.PLANNING
            self.validate_and_execute_plan()

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in task plan: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing task plan: {e}')

    def validate_and_execute_plan(self):
        """Validate plan structure and start execution."""
        # Basic validation
        if not isinstance(self.current_plan, list):
            self.get_logger().error('Plan must be a list of actions')
            self.transition_to_failed('Invalid plan structure')
            return

        # Check each action has required fields
        for i, action in enumerate(self.current_plan):
            if 'action' not in action:
                self.get_logger().error(f'Action {i} missing "action" field')
                self.transition_to_failed('Invalid action format')
                return

            if 'parameters' not in action:
                self.get_logger().error(f'Action {i} missing "parameters" field')
                self.transition_to_failed('Invalid action format')
                return

        # Plan is valid, start execution
        self.get_logger().info('Plan validation passed')
        self.state = RobotState.EXECUTING
        self.current_action_index = 0
        self.retry_count = 0
        self.task_start_time = time.time()

        self.execute_next_action()

    def execute_next_action(self):
        """Execute next action in sequence."""
        if self.current_action_index >= len(self.current_plan):
            # All actions complete
            elapsed = time.time() - self.task_start_time
            self.get_logger().info(f'Task completed successfully in {elapsed:.1f}s')
            self.transition_to_completed()
            return

        action = self.current_plan[self.current_action_index]
        action_name = action['action']

        self.get_logger().info(
            f'Executing action {self.current_action_index + 1}/'
            f'{len(self.current_plan)}: {action_name}'
        )

        self.current_action_start_time = time.time()

        # Simulate action execution (in real system, would call action_executor)
        success = self.simulate_action_execution(action)

        if success:
            action_elapsed = time.time() - self.current_action_start_time
            self.get_logger().info(
                f'Action {action_name} completed in {action_elapsed:.1f}s'
            )
            self.retry_count = 0
            self.current_action_index += 1
            self.execute_next_action()  # Continue to next action
        else:
            self.handle_action_failure(action)

    def simulate_action_execution(self, action: Dict[str, Any]) -> bool:
        """
        Simulate action execution (placeholder for actual action_executor).

        In real implementation, this would dispatch to action_executor node
        which calls ROS 2 action servers (Nav2, Isaac ROS, Gripper).

        Returns:
            bool: True if action succeeded, False otherwise
        """
        # Simulate execution delay
        expected_duration = action.get('expected_duration', 1.0)
        time.sleep(min(expected_duration * 0.1, 0.5))  # Simulate 10% of expected time

        # Success for all actions except occasional failures for demo purposes
        # In real system, this would be actual action execution result
        return True

    def handle_action_failure(self, action: Dict[str, Any]):
        """Handle failed action with retry or replan."""
        if self.retry_count < self.max_retries:
            self.retry_count += 1
            self.get_logger().warn(
                f'Retrying action {action["action"]} '
                f'(attempt {self.retry_count}/{self.max_retries})'
            )
            # Retry same action (don't increment current_action_index)
            self.execute_next_action()
        else:
            self.get_logger().error(
                f'Action {action["action"]} failed after {self.max_retries} retries'
            )
            self.transition_to_failed(f'Action failed: {action["action"]}')

    def transition_to_completed(self):
        """Transition to COMPLETED state."""
        self.state = RobotState.COMPLETED
        self.get_logger().info('State: COMPLETED')

        # Transition to IDLE after timeout
        self.create_timer(
            self.idle_timeout,
            self.transition_to_idle,
            once=True
        )

    def transition_to_failed(self, reason: str):
        """Transition to FAILED state."""
        self.state = RobotState.FAILED
        self.get_logger().error(f'State: FAILED - {reason}')

        # Publish clarification request
        clarify_msg = String()
        clarify_msg.data = json.dumps({
            'reason': reason,
            'failed_action_index': self.current_action_index,
            'failed_action': self.current_plan[self.current_action_index]
            if self.current_action_index < len(self.current_plan) else None
        })
        self.clarify_pub.publish(clarify_msg)

        # Transition to IDLE after timeout
        self.create_timer(
            self.idle_timeout,
            self.transition_to_idle,
            once=True
        )

    def transition_to_idle(self):
        """Transition to IDLE state (ready for next task)."""
        self.state = RobotState.IDLE
        self.current_plan = []
        self.current_action_index = 0
        self.retry_count = 0
        self.task_start_time = None
        self.get_logger().info('State: IDLE (ready for next task)')

    def publish_status(self):
        """Publish execution status (called every 500ms)."""
        elapsed_time = 0.0
        estimated_remaining = 0.0

        if self.task_start_time:
            elapsed_time = time.time() - self.task_start_time

            # Estimate remaining time
            if self.current_action_index < len(self.current_plan):
                for action in self.current_plan[self.current_action_index:]:
                    estimated_remaining += action.get('expected_duration', 5.0)

        status = {
            'state': self.state.name,
            'current_action': (
                self.current_plan[self.current_action_index]['action']
                if self.current_action_index < len(self.current_plan)
                else None
            ),
            'progress': self.current_action_index,
            'total_actions': len(self.current_plan),
            'elapsed_time': round(elapsed_time, 1),
            'estimated_remaining': round(estimated_remaining, 1)
        }

        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main(args=None):
    """Main entry point for the behavior coordinator node."""
    rclpy.init(args=args)

    try:
        node = BehaviorCoordinator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
