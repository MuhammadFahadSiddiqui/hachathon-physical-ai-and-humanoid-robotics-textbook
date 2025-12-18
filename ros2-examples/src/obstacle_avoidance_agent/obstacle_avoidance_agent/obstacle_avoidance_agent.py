#!/usr/bin/env python3
"""
Obstacle Avoidance Agent - Python Agents & Controllers Example

This node demonstrates the sensor-agent-actuator pipeline by implementing
a simple obstacle avoidance behavior using laser scan data.

Learning objectives:
- Implement an agent that bridges perception (sensors) to action (actuators)
- Use timer-based decision loops decoupled from sensor callbacks
- Process LaserScan messages to detect obstacles
- Publish Twist messages to command robot velocity
- Apply modular design with separate sensing, decision, and actuation methods
"""

import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Base class for ROS 2 nodes
from sensor_msgs.msg import LaserScan  # Laser scanner message type
from geometry_msgs.msg import Twist  # Velocity command message type


class ObstacleAvoidanceAgent(Node):
    """
    An agent that avoids obstacles using laser scan data.

    This agent subscribes to /scan (LaserScan), processes the data to detect
    obstacles, and publishes velocity commands to /cmd_vel (Twist) to navigate
    safely. It uses a timer-based decision loop running at 10 Hz.
    """

    def __init__(self):
        """
        Initialize the obstacle avoidance agent.

        This constructor:
        1. Calls the parent Node constructor
        2. Creates a subscription to /scan for laser scan data
        3. Creates a publisher to /cmd_vel for velocity commands
        4. Initializes internal state variables
        5. Creates a timer for periodic decision-making
        """
        # Call the Node constructor with the node name
        super().__init__('obstacle_avoidance_agent')

        # ===== 1. SUBSCRIPTIONS (Perception Layer) =====

        # Subscribe to laser scan data from /scan topic
        # LaserScan messages contain distance measurements from a laser rangefinder
        self.scan_subscription = self.create_subscription(
            LaserScan,           # Message type
            '/scan',             # Topic name
            self.scan_callback,  # Callback function
            10                   # QoS depth (buffer 10 messages)
        )

        # ===== 2. PUBLISHERS (Actuation Layer) =====

        # Publish velocity commands to /cmd_vel topic
        # Twist messages contain linear and angular velocity commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,      # Message type
            '/cmd_vel', # Topic name
            10          # QoS depth
        )

        # ===== 3. INTERNAL STATE (Agent Memory) =====

        # Store the latest laser scan message
        # This allows the decision loop to access sensor data asynchronously
        self.latest_scan = None

        # Minimum distance to obstacle (meters)
        # Updated by scan_callback, used by decision_callback
        self.min_distance = float('inf')

        # Obstacle detection threshold (meters)
        # If any laser reading is below this value, an obstacle is detected
        self.OBSTACLE_THRESHOLD = 1.0  # 1 meter

        # Velocity parameters
        self.FORWARD_SPEED = 0.2   # Linear velocity when path is clear (m/s)
        self.TURN_SPEED = 0.5      # Angular velocity when avoiding (rad/s)

        # ===== 4. TIMER (Decision Loop) =====

        # Create a timer that calls decision_callback every 0.1 seconds (10 Hz)
        # This decouples the decision rate from the sensor rate
        # Even if scans arrive at 30 Hz, decisions are made at 10 Hz
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.decision_callback)

        # Log that the agent has started
        self.get_logger().info('Obstacle avoidance agent started')
        self.get_logger().info(f'Subscribing to: /scan')
        self.get_logger().info(f'Publishing to: /cmd_vel')
        self.get_logger().info(f'Decision rate: {1/timer_period} Hz')
        self.get_logger().info(f'Obstacle threshold: {self.OBSTACLE_THRESHOLD} m')

    # ========== PERCEPTION LAYER ==========

    def scan_callback(self, msg):
        """
        Callback for laser scan messages.

        This method is called every time a LaserScan message arrives on /scan.
        It stores the latest scan and extracts the minimum distance to obstacles.

        Args:
            msg (sensor_msgs.msg.LaserScan): Incoming laser scan message
                - msg.ranges: List of distance measurements (meters)
                - msg.angle_min: Start angle of scan (radians)
                - msg.angle_max: End angle of scan (radians)
                - msg.angle_increment: Angular distance between measurements

        Note:
            This callback should complete quickly to avoid blocking other callbacks.
            Heavy processing should be done in the decision_callback.
        """
        # Store the latest scan for use in decision loop
        self.latest_scan = msg

        # Extract minimum distance from laser scan ranges
        # Filter out invalid readings (inf, nan, 0)
        valid_ranges = [r for r in msg.ranges if r > 0.0 and r < float('inf')]

        if valid_ranges:
            # Update minimum distance
            self.min_distance = min(valid_ranges)
        else:
            # No valid readings - assume no obstacles
            self.min_distance = float('inf')

    # ========== DECISION LAYER ==========

    def decision_callback(self):
        """
        Timer callback for making decisions.

        This method is called periodically (10 Hz) by the timer. It implements
        the agent's decision logic:
        1. Check if sensor data is available
        2. Determine if obstacle is detected
        3. Decide on appropriate action (move forward or avoid)
        4. Publish velocity command

        Decision Logic:
        - If min_distance < OBSTACLE_THRESHOLD: Stop and turn right
        - Otherwise: Move forward

        Note:
            This callback runs independently of sensor callbacks, allowing
            for consistent decision rates regardless of sensor frequency.
        """
        # Check if we have received any scan data yet
        if self.latest_scan is None:
            self.get_logger().warn('No scan data received yet, waiting...')
            # Publish stop command for safety
            self.publish_velocity(0.0, 0.0)
            return

        # === DECISION LOGIC ===

        # Check if obstacle is within threshold
        if self.min_distance < self.OBSTACLE_THRESHOLD:
            # AVOID: Obstacle detected - stop and turn right
            linear_velocity = 0.0          # Stop forward motion
            angular_velocity = -self.TURN_SPEED  # Turn right (negative = clockwise)

            self.get_logger().info(
                f'[AVOID] Obstacle at {self.min_distance:.2f}m - Turning right'
            )
        else:
            # FORWARD: Path is clear - move forward
            linear_velocity = self.FORWARD_SPEED  # Move forward
            angular_velocity = 0.0                # No turning

            self.get_logger().info(
                f'[FORWARD] Clear path ({self.min_distance:.2f}m) - Moving forward'
            )

        # Publish the velocity command
        self.publish_velocity(linear_velocity, angular_velocity)

    # ========== ACTUATION LAYER ==========

    def publish_velocity(self, linear_x, angular_z):
        """
        Publish a velocity command to /cmd_vel.

        This method creates a Twist message with the specified linear and
        angular velocities and publishes it to the /cmd_vel topic.

        Args:
            linear_x (float): Forward velocity in m/s
                - Positive: forward
                - Negative: backward
                - Zero: stop
            angular_z (float): Angular velocity in rad/s
                - Positive: turn left (counter-clockwise)
                - Negative: turn right (clockwise)
                - Zero: no turning

        Note:
            For differential drive robots, only linear.x and angular.z are used.
            Other components (linear.y, linear.z, angular.x, angular.y) are zero.
        """
        # Create a Twist message
        msg = Twist()

        # Set linear velocity (forward/backward)
        msg.linear.x = linear_x  # m/s
        msg.linear.y = 0.0       # No sideways motion (differential drive)
        msg.linear.z = 0.0       # No vertical motion (ground robot)

        # Set angular velocity (turning)
        msg.angular.x = 0.0      # No roll
        msg.angular.y = 0.0      # No pitch
        msg.angular.z = angular_z # rad/s

        # Publish the velocity command
        self.cmd_vel_publisher.publish(msg)


def main(args=None):
    """
    Main entry point for the obstacle_avoidance_agent node.

    This function:
    1. Initializes the ROS 2 Python client library
    2. Creates an instance of the ObstacleAvoidanceAgent
    3. Spins the node to keep it running
    4. Handles cleanup on shutdown

    Args:
        args: Command-line arguments (passed to rclpy.init)
    """
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create the agent node
    obstacle_avoidance_agent = ObstacleAvoidanceAgent()

    # Spin the node - this blocks and processes callbacks
    # The node will continue running until interrupted (Ctrl+C)
    try:
        rclpy.spin(obstacle_avoidance_agent)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        obstacle_avoidance_agent.get_logger().info('Keyboard interrupt, shutting down')

    # Cleanup: destroy the node and shutdown ROS 2
    obstacle_avoidance_agent.destroy_node()
    rclpy.shutdown()


# Standard Python idiom - run main() if this script is executed directly
if __name__ == '__main__':
    main()
