"""Scissors Mechanism service."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64

from ros_vision.srv import ScissorsMechanismParams


class ScissorsMechanismSubscriber(Node):
    """Node who owns the actual position of the mechanism."""

    def __init__(self):
        """WIP."""
        super().__init__("scissors_subscriber")
        self.subscription = self.create_subscription(
            Int64, "scissors_position", self.listener_callback, 1
        )
        self.position = 0
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        """WIP."""
        self.position = msg.data


class ScissorsMechanism(Node):
    """Node for consumer and topics writer."""

    def __init__(self):
        """Init."""
        super().__init__("scissors_mechanism")
        self.srv = self.create_service(
            ScissorsMechanismParams, "shift_scissors_mechanism", self.procedure
        )
        self.position_publisher = self.create_publisher(Int64, "scissors_movement", 1)
        self.mechanism = ScissorsMechanismSubscriber()

    def procedure(self, request, response):
        """Movement for scissors."""
        position_required = request.position_required

        position_msg = Int64()
        position_msg.data = position_required
        self.position_publisher.publish(position_msg)

        while position_required != self.mechanism.position:
            rclpy.spin_once(self.mechanism)

        response.status_code = 2
        return response
