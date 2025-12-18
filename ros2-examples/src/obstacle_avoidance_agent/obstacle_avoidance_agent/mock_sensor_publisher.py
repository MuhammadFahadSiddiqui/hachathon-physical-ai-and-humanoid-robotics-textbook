#!/usr/bin/env python3
"""
Mock Sensor Publisher - Testing Tool for Obstacle Avoidance Agent

This node simulates a laser scanner by publishing mock LaserScan messages
to the /scan topic. It allows testing the obstacle avoidance agent without
needing a real robot or Gazebo simulation.

Learning objectives:
- Understand the LaserScan message structure
- Create mock sensor data for testing agents
- Use timers for periodic publishing
- Simulate different obstacle scenarios
"""

import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Base class for ROS 2 nodes
from sensor_msgs.msg import LaserScan  # Laser scanner message type
import math  # For generating realistic scan patterns


class MockSensorPublisher(Node):
    """
    A mock laser scanner that publishes simulated LaserScan messages.

    This publisher simulates different scenarios:
    - Clear path (all distances > 1.0m)
    - Obstacle ahead (distance < 1.0m in front)
    - Alternating between clear and blocked

    The mock data allows testing the obstacle avoidance agent's logic
    without hardware or simulation.
    """

    def __init__(self):
        """
        Initialize the mock sensor publisher.

        This constructor:
        1. Calls the parent Node constructor
        2. Creates a publisher for LaserScan messages
        3. Sets up laser scanner parameters
        4. Creates a timer for periodic publishing
        5. Initializes scenario state
        """
        # Call the Node constructor
        super().__init__('mock_sensor_publisher')

        # ===== PUBLISHER =====

        # Publish laser scan data to /scan topic
        self.scan_publisher = self.create_publisher(
            LaserScan,  # Message type
            '/scan',    # Topic name
            10          # QoS depth
        )

        # ===== LASER SCANNER PARAMETERS =====

        # Simulated laser scanner specifications
        # These parameters should match a typical 2D lidar (e.g., Hokuyo, RPLIDAR)
        self.angle_min = -math.pi / 2  # -90 degrees (right side)
        self.angle_max = math.pi / 2   # +90 degrees (left side)
        self.angle_increment = math.pi / 180  # 1 degree increments
        self.range_min = 0.1  # Minimum detectable distance (meters)
        self.range_max = 10.0 # Maximum detectable distance (meters)

        # Calculate number of laser beams
        self.num_readings = int(
            (self.angle_max - self.angle_min) / self.angle_increment
        ) + 1

        # ===== SCENARIO STATE =====

        # Scenario counter - alternates between clear and blocked
        self.scenario_counter = 0

        # Scenario duration (number of scans before switching)
        self.SCANS_PER_SCENARIO = 30  # 30 scans * 0.1s = 3 seconds per scenario

        # ===== TIMER =====

        # Publish laser scans at 10 Hz (typical for 2D lidars)
        timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(timer_period, self.publish_scan)

        # Log startup information
        self.get_logger().info('Mock sensor publisher started')
        self.get_logger().info(f'Publishing to: /scan at {1/timer_period} Hz')
        self.get_logger().info(f'Laser parameters:')
        self.get_logger().info(f'  - Field of view: {math.degrees(self.angle_max - self.angle_min):.0f} degrees')
        self.get_logger().info(f'  - Number of beams: {self.num_readings}')
        self.get_logger().info(f'  - Angular resolution: {math.degrees(self.angle_increment):.1f} degrees')

    def publish_scan(self):
        """
        Timer callback that publishes a mock LaserScan message.

        This method:
        1. Creates a LaserScan message
        2. Fills in header and laser parameters
        3. Generates mock distance readings based on current scenario
        4. Publishes the message to /scan

        Scenarios:
        - Scenario 0-29: Clear path (all distances = 5.0m)
        - Scenario 30-59: Obstacle ahead (center beams = 0.5m)
        """
        # Create LaserScan message
        msg = LaserScan()

        # ===== HEADER =====

        # Set timestamp to current time
        msg.header.stamp = self.get_clock().now().to_msg()

        # Set frame_id (coordinate frame for the laser scanner)
        # Typically 'base_scan' or 'laser' for mobile robots
        msg.header.frame_id = 'base_scan'

        # ===== LASER PARAMETERS =====

        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.angle_increment
        msg.time_increment = 0.0  # Time between measurements (0 for instantaneous)
        msg.scan_time = 0.1       # Time for complete scan (seconds)
        msg.range_min = self.range_min
        msg.range_max = self.range_max

        # ===== GENERATE MOCK DISTANCE READINGS =====

        # Determine current scenario
        scenario = (self.scenario_counter // self.SCANS_PER_SCENARIO) % 2

        if scenario == 0:
            # SCENARIO 0: Clear path - all distances are far
            ranges = [5.0] * self.num_readings
            self.get_logger().info('[CLEAR] All distances: 5.0m')

        else:
            # SCENARIO 1: Obstacle ahead - center beams detect obstacle
            ranges = []

            # Calculate center index
            center_index = self.num_readings // 2

            # Generate readings: obstacle in center 20 degrees, clear elsewhere
            obstacle_width = int(20 / math.degrees(self.angle_increment))  # 20 degrees

            for i in range(self.num_readings):
                # Check if this beam is in the center (obstacle region)
                if abs(i - center_index) < obstacle_width // 2:
                    # Obstacle detected - short distance
                    ranges.append(0.5)  # 0.5 meters
                else:
                    # Clear path - far distance
                    ranges.append(5.0)  # 5.0 meters

            # Find minimum distance for logging
            min_dist = min(ranges)
            self.get_logger().info(f'[OBSTACLE] Minimum distance: {min_dist:.2f}m')

        # Set the ranges in the message
        msg.ranges = ranges

        # Set intensities (optional, not used by obstacle avoidance agent)
        # Zero intensity means we don't have intensity data
        msg.intensities = [0.0] * self.num_readings

        # ===== PUBLISH MESSAGE =====

        self.scan_publisher.publish(msg)

        # Increment scenario counter
        self.scenario_counter += 1


def main(args=None):
    """
    Main entry point for the mock_sensor_publisher node.

    This function:
    1. Initializes ROS 2
    2. Creates the mock sensor publisher
    3. Spins the node
    4. Handles cleanup

    Args:
        args: Command-line arguments
    """
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create the mock sensor publisher node
    mock_sensor_publisher = MockSensorPublisher()

    # Spin the node
    try:
        rclpy.spin(mock_sensor_publisher)
    except KeyboardInterrupt:
        mock_sensor_publisher.get_logger().info('Keyboard interrupt, shutting down')

    # Cleanup
    mock_sensor_publisher.destroy_node()
    rclpy.shutdown()


# Run main if executed directly
if __name__ == '__main__':
    main()
