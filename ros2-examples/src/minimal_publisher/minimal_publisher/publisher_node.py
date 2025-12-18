#!/usr/bin/env python3
"""
Minimal Publisher Node - ROS 2 Fundamentals Example

This node demonstrates the basics of ROS 2 publishing by sending
String messages to the '/chatter' topic at a fixed rate.

Learning objectives:
- Initialize the ROS 2 Python client library (rclpy)
- Create a Node class instance
- Create a publisher with a specific message type
- Use timers for periodic publishing
- Properly shutdown ROS 2 resources
"""

import rclpy  # Import the ROS 2 Python client library
from rclpy.node import Node  # Import the Node class for creating ROS 2 nodes
from std_msgs.msg import String  # Import the String message type from std_msgs


class MinimalPublisher(Node):
    """
    A simple publisher node that sends String messages to '/chatter' topic.

    This class inherits from rclpy.node.Node, which provides the core
    functionality for ROS 2 nodes (publishers, subscribers, timers, etc.).
    """

    def __init__(self):
        """
        Initialize the publisher node.

        This constructor:
        1. Calls the parent Node constructor with a unique node name
        2. Creates a publisher for String messages on the '/chatter' topic
        3. Creates a timer that calls timer_callback() every 0.5 seconds
        4. Initializes a counter for message sequencing
        """
        # Call the Node constructor with the node name 'minimal_publisher'
        # This name must be unique within the ROS 2 graph
        super().__init__('minimal_publisher')

        # Create a publisher:
        # - Message type: String (from std_msgs)
        # - Topic name: 'chatter'
        # - Queue size: 10 (number of messages to buffer if subscribers are slow)
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # Create a timer that fires every 0.5 seconds (2 Hz publishing rate)
        # The timer will call self.timer_callback() on each tick
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize a counter to track the number of messages published
        # This will be incremented in the timer callback
        self.count = 0

        # Log that the node has started (appears in terminal and logs)
        self.get_logger().info('Minimal publisher node started, publishing at 2 Hz')

    def timer_callback(self):
        """
        Timer callback function - called every 0.5 seconds.

        This method:
        1. Creates a new String message
        2. Populates the message data field with a formatted string
        3. Publishes the message to all subscribers on '/chatter'
        4. Logs the published message to the console
        5. Increments the message counter
        """
        # Create a new String message instance
        msg = String()

        # Populate the 'data' field of the String message
        # The String message type has a single field: string data
        msg.data = f'Hello World: {self.count}'

        # Publish the message to all subscribers on the '/chatter' topic
        # This is asynchronous - the publisher does not wait for subscribers
        self.publisher_.publish(msg)

        # Log the published message to the console with INFO severity
        # This helps verify that the node is working correctly
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment the counter for the next message
        self.count += 1


def main(args=None):
    """
    Main entry point for the minimal_publisher node.

    This function:
    1. Initializes the ROS 2 Python client library (rclpy)
    2. Creates an instance of the MinimalPublisher node
    3. Spins the node to keep it running and processing callbacks
    4. Handles cleanup when the node is shut down (Ctrl+C)

    Args:
        args: Command-line arguments (passed to rclpy.init)
    """
    # Initialize the ROS 2 Python client library
    # This must be called before creating any ROS 2 nodes
    # It sets up communication with the ROS 2 middleware (DDS)
    rclpy.init(args=args)

    # Create an instance of our MinimalPublisher node
    # This calls __init__() which sets up the publisher and timer
    minimal_publisher = MinimalPublisher()

    # Spin the node - this blocks and processes callbacks (timers, subscriptions, etc.)
    # The node will continue running until interrupted (Ctrl+C) or an error occurs
    # In our case, this keeps the timer callback running at 2 Hz
    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        minimal_publisher.get_logger().info('Keyboard interrupt, shutting down')

    # Destroy the node explicitly (optional but good practice)
    # This releases resources like publishers, timers, and subscriptions
    minimal_publisher.destroy_node()

    # Shutdown the ROS 2 Python client library
    # This cleans up all ROS 2 resources and disconnects from the middleware
    rclpy.shutdown()


# Standard Python idiom to check if this script is being run directly
# (as opposed to being imported as a module)
if __name__ == '__main__':
    main()
