#!/usr/bin/env python3
"""
Minimal Subscriber Node - ROS 2 Fundamentals Example

This node demonstrates the basics of ROS 2 subscribing by receiving
String messages from the '/chatter' topic and logging them to the console.

Learning objectives:
- Create a subscription to a topic
- Implement a callback function to process incoming messages
- Understand Quality of Service (QoS) profiles
- Handle messages asynchronously
"""

import rclpy  # Import the ROS 2 Python client library
from rclpy.node import Node  # Import the Node class
from std_msgs.msg import String  # Import the String message type


class MinimalSubscriber(Node):
    """
    A simple subscriber node that receives String messages from '/chatter' topic.

    This class demonstrates the publish-subscribe pattern in ROS 2, where
    this node acts as the subscriber receiving messages sent by publishers.
    """

    def __init__(self):
        """
        Initialize the subscriber node.

        This constructor:
        1. Calls the parent Node constructor with a unique node name
        2. Creates a subscription to the '/chatter' topic
        3. Specifies a callback function to handle incoming messages
        4. Sets the QoS (Quality of Service) profile for message delivery
        """
        # Call the Node constructor with the node name 'minimal_subscriber'
        # This name must be unique within the ROS 2 graph
        super().__init__('minimal_subscriber')

        # Create a subscription:
        # - Message type: String (must match the publisher's message type)
        # - Topic name: 'chatter' (must match the topic the publisher uses)
        # - Callback function: self.listener_callback (called when messages arrive)
        # - Queue size: 10 (number of messages to buffer if callback is slow)
        #
        # The queue size (QoS depth) determines how many messages are buffered
        # if the callback function can't process messages fast enough.
        # If the queue fills up, older messages are discarded.
        self.subscription = self.create_subscription(
            String,                    # Message type
            'chatter',                 # Topic name
            self.listener_callback,    # Callback function
            10                         # QoS profile depth (queue size)
        )

        # Prevent unused variable warning (self.subscription is not directly used)
        # The subscription object must be stored as an instance variable
        # to prevent it from being garbage collected
        self.subscription  # prevent unused variable warning

        # Log that the node has started
        self.get_logger().info('Minimal subscriber node started, waiting for messages...')

    def listener_callback(self, msg):
        """
        Callback function - called whenever a message arrives on '/chatter'.

        This method is invoked asynchronously by the ROS 2 executor whenever
        a new message is published to the '/chatter' topic. The callback
        receives the message as a parameter.

        Args:
            msg (std_msgs.msg.String): The incoming message from the topic.
                                       For String messages, access data via msg.data

        Note:
            - This callback should complete quickly to avoid blocking other callbacks
            - If processing takes time, consider using a multi-threaded executor
            - The callback runs in the same thread as rclpy.spin() by default
        """
        # Access the 'data' field of the String message
        # String messages have a single field: string data
        message_content = msg.data

        # Log the received message to the console with INFO severity
        # This displays: [INFO] [timestamp] [minimal_subscriber]: I heard: "Hello World: 5"
        self.get_logger().info(f'I heard: "{message_content}"')


def main(args=None):
    """
    Main entry point for the minimal_subscriber node.

    This function:
    1. Initializes the ROS 2 Python client library
    2. Creates an instance of the MinimalSubscriber node
    3. Spins the node to keep it running and processing callbacks
    4. Handles cleanup when the node is shut down

    Args:
        args: Command-line arguments (passed to rclpy.init)
    """
    # Initialize the ROS 2 Python client library
    # This sets up communication with the ROS 2 middleware (DDS)
    rclpy.init(args=args)

    # Create an instance of our MinimalSubscriber node
    # This calls __init__() which sets up the subscription
    minimal_subscriber = MinimalSubscriber()

    # Spin the node - this blocks and waits for messages
    # When a message arrives on '/chatter', listener_callback() is called
    # The node continues running until interrupted (Ctrl+C) or an error occurs
    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        minimal_subscriber.get_logger().info('Keyboard interrupt, shutting down')

    # Destroy the node explicitly (optional but good practice)
    # This releases resources like subscriptions and timers
    minimal_subscriber.destroy_node()

    # Shutdown the ROS 2 Python client library
    # This cleans up all ROS 2 resources and disconnects from the middleware
    rclpy.shutdown()


# Standard Python idiom - run main() if this script is executed directly
if __name__ == '__main__':
    main()
