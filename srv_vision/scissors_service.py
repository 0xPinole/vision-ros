"""Scissors Mechanism service."""

# import rclpy
from rclpy.node import Node

# from ros_vision.srv import ScissorsMechanismParams


class ScissorsMechanism(Node):
    """Node for consumer and topics writer."""

    def __init__(self):
        """Init."""
        super().__init__("scissors_mechanism")
        self.srv = self.create_service()
