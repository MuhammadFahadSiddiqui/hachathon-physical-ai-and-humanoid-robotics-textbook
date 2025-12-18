#!/usr/bin/env python3
"""
Service Client Node - ROS 2 Fundamentals Example

This node demonstrates ROS 2 service clients by sending AddTwoInts
requests to the service server and receiving responses.

Learning objectives:
- Create a service client
- Wait for a service to become available
- Send asynchronous service requests
- Handle service responses using Futures
- Understand blocking vs. non-blocking service calls
"""

import sys  # For command-line arguments
import rclpy  # Import the ROS 2 Python client library
from rclpy.node import Node  # Import the Node class
from example_interfaces.srv import AddTwoInts  # Import the AddTwoInts service type


class MinimalClientAsync(Node):
    """
    A service client node that calls the '/add_two_ints' service.

    This node creates a client for the AddTwoInts service and sends
    requests with two integers, then receives and processes the response.
    """

    def __init__(self):
        """
        Initialize the service client node.

        This constructor:
        1. Calls the parent Node constructor with a unique node name
        2. Creates a service client for the AddTwoInts service
        3. Waits for the service server to become available
        """
        # Call the Node constructor with the node name 'minimal_client_async'
        super().__init__('minimal_client_async')

        # Create a service client:
        # - Service type: AddTwoInts (must match the server's service type)
        # - Service name: 'add_two_ints' (must match the server's service name)
        #
        # The client will automatically discover the service server via DDS
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service to become available
        # This is important because the client should not send requests
        # until the server is ready to handle them
        #
        # wait_for_service() blocks until the service is discovered
        # The timeout_sec parameter prevents infinite waiting
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        # Log that the service is now available
        self.get_logger().info('Service /add_two_ints is available')

    def send_request(self, a, b):
        """
        Send a service request to add two integers.

        This method creates an AddTwoInts request, populates it with
        the given integers, and sends it asynchronously to the server.

        Args:
            a (int): First integer to add
            b (int): Second integer to add

        Returns:
            Future: A Future object representing the pending response.
                    Call future.result() to get the response (blocks until ready)

        Note:
            - call_async() is non-blocking - it returns immediately
            - The response is not available until the server processes the request
            - Use rclpy.spin_until_future_complete() to wait for the response
        """
        # Create an AddTwoInts request object
        # The AddTwoInts.Request type has two fields: a and b (both int64)
        request = AddTwoInts.Request()

        # Populate the request fields with the given integers
        request.a = a
        request.b = b

        # Log the request being sent
        self.get_logger().info(f'Sending request: {a} + {b}')

        # Send the request asynchronously (non-blocking)
        # This returns a Future object immediately, even though the
        # response is not yet available
        #
        # The server will process the request and the response will be
        # delivered later (when the Future completes)
        future = self.client.call_async(request)

        # Return the Future object so the caller can wait for the response
        return future


def main(args=None):
    """
    Main entry point for the service_client node.

    This function:
    1. Parses command-line arguments (two integers)
    2. Initializes the ROS 2 Python client library
    3. Creates the service client node
    4. Sends a service request
    5. Waits for the response and displays the result
    6. Handles cleanup

    Args:
        args: Command-line arguments (passed to rclpy.init)

    Usage:
        ros2 run service_example service_client 3 5
        (This will request 3 + 5 = 8)
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Check if two integers were provided as command-line arguments
    # sys.argv[0] is the script name, sys.argv[1] and sys.argv[2] are the args
    if len(sys.argv) != 3:
        print('Usage: ros2 run service_example service_client <a> <b>')
        print('Example: ros2 run service_example service_client 3 5')
        rclpy.shutdown()
        return

    # Parse the command-line arguments as integers
    try:
        a = int(sys.argv[1])
        b = int(sys.argv[2])
    except ValueError:
        print('Error: Both arguments must be integers')
        rclpy.shutdown()
        return

    # Create an instance of our MinimalClientAsync node
    # This calls __init__() which sets up the service client
    # and waits for the service to become available
    minimal_client = MinimalClientAsync()

    # Send the service request with the two integers
    # This returns a Future object representing the pending response
    future = minimal_client.send_request(a, b)

    # Spin the node until the Future completes (blocking)
    # This processes callbacks and waits for the service response
    #
    # spin_until_future_complete() will:
    # 1. Process incoming messages and callbacks
    # 2. Check if the Future is complete
    # 3. Return when the Future has a result (or times out)
    rclpy.spin_until_future_complete(minimal_client, future)

    # Get the response from the completed Future
    # future.result() blocks if the Future is not yet complete
    # (but it should already be complete after spin_until_future_complete)
    try:
        response = future.result()
    except Exception as e:
        # Handle service call failures (e.g., server crashed, timeout)
        minimal_client.get_logger().error(f'Service call failed: {e}')
    else:
        # Success! Display the result
        # The AddTwoInts.Response type has a single field: sum (int64)
        minimal_client.get_logger().info(
            f'Result: {a} + {b} = {response.sum}'
        )

    # Destroy the node explicitly (optional but good practice)
    minimal_client.destroy_node()

    # Shutdown the ROS 2 Python client library
    rclpy.shutdown()


# Standard Python idiom - run main() if this script is executed directly
if __name__ == '__main__':
    main()
