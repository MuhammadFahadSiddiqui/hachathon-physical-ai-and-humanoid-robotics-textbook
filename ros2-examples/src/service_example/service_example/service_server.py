#!/usr/bin/env python3
"""
Service Server Node - ROS 2 Fundamentals Example

This node demonstrates ROS 2 services by implementing a server that
adds two integers. It receives AddTwoInts service requests and returns
the sum as a response.

Learning objectives:
- Create a service server
- Define a service callback to process requests
- Access request fields and populate response fields
- Understand synchronous request-response communication
"""

import rclpy  # Import the ROS 2 Python client library
from rclpy.node import Node  # Import the Node class
from example_interfaces.srv import AddTwoInts  # Import the AddTwoInts service type


class MinimalService(Node):
    """
    A service server node that adds two integers.

    This node creates a service named '/add_two_ints' that accepts
    AddTwoInts requests (two integers: a and b) and returns their sum.
    """

    def __init__(self):
        """
        Initialize the service server node.

        This constructor:
        1. Calls the parent Node constructor with a unique node name
        2. Creates a service server that handles AddTwoInts requests
        3. Registers a callback function to process incoming requests
        """
        # Call the Node constructor with the node name 'minimal_service'
        super().__init__('minimal_service')

        # Create a service server:
        # - Service type: AddTwoInts (from example_interfaces package)
        #   - Request fields: int64 a, int64 b
        #   - Response fields: int64 sum
        # - Service name: 'add_two_ints'
        # - Callback function: self.add_two_ints_callback
        #
        # The service is advertised on the ROS 2 graph and clients can
        # discover it automatically via DDS discovery
        self.srv = self.create_service(
            AddTwoInts,                      # Service type
            'add_two_ints',                  # Service name
            self.add_two_ints_callback       # Callback function
        )

        # Log that the service is ready to accept requests
        self.get_logger().info('Service server ready: /add_two_ints')

    def add_two_ints_callback(self, request, response):
        """
        Service callback function - processes AddTwoInts requests.

        This method is called whenever a client sends a request to the
        '/add_two_ints' service. It receives the request object, processes it,
        and populates the response object.

        Args:
            request (example_interfaces.srv.AddTwoInts_Request):
                The incoming request containing two integers:
                - request.a: First integer
                - request.b: Second integer

            response (example_interfaces.srv.AddTwoInts_Response):
                The response object to populate with the result:
                - response.sum: The sum of a and b

        Returns:
            example_interfaces.srv.AddTwoInts_Response:
                The populated response object (with response.sum set)

        Note:
            - This callback is synchronous - it blocks the client until complete
            - Keep processing time short to avoid blocking clients
            - For long operations, consider using actions instead of services
        """
        # Access the two integers from the request
        # The AddTwoInts service type defines:
        #   Request: int64 a, int64 b
        #   Response: int64 sum
        a = request.a
        b = request.b

        # Compute the sum
        result = a + b

        # Populate the response field 'sum'
        response.sum = result

        # Log the request and response for debugging
        # This helps verify that the service is processing requests correctly
        self.get_logger().info(
            f'Request: {a} + {b} = {result}'
        )

        # Return the populated response object
        # The ROS 2 middleware will send this response back to the client
        return response


def main(args=None):
    """
    Main entry point for the service_server node.

    This function:
    1. Initializes the ROS 2 Python client library
    2. Creates an instance of the MinimalService node
    3. Spins the node to keep it running and processing service requests
    4. Handles cleanup when the node is shut down

    Args:
        args: Command-line arguments (passed to rclpy.init)
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of our MinimalService node
    # This calls __init__() which sets up the service server
    minimal_service = MinimalService()

    # Spin the node - this blocks and waits for service requests
    # When a request arrives, add_two_ints_callback() is called
    # The node continues running until interrupted (Ctrl+C)
    try:
        rclpy.spin(minimal_service)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        minimal_service.get_logger().info('Keyboard interrupt, shutting down')

    # Destroy the node explicitly (optional but good practice)
    # This releases resources like services
    minimal_service.destroy_node()

    # Shutdown the ROS 2 Python client library
    rclpy.shutdown()


# Standard Python idiom - run main() if this script is executed directly
if __name__ == '__main__':
    main()
