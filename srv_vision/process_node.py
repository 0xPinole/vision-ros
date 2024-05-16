from vision_interfaces.srv import Procedure
from vision_interfaces.msg import PositionalFrame

from std_msgs.msg import Int8

from geometry_msgs.msg import Pose2D

import numpy as np

class Procedure(Node):
    """Ros node subscriber to information and trigger of task.

    This node when needed, enables a process which capture each frame
    and getting a positional status, send to evaluation node the request
    to predict the model for the products on frame.
    """
    self.evaluate = False
    self.frame = None

    def __init__(self):
        """Set a service who enables or disable the connection.

        For the case is enable, the scissors subscriptions now the node will
        publish the camera frame on the evaluation topic with custom message.
        """
        super().__init__("procedure_node")
        self.service_setter = self.create_service(Procedure, "procedure_init", self.service_callback)
        self.evaluate_pub = self.create_publisher(PositionalFrame, "evaluation", 1000)
        self.scissors_sub = self.create_subscription(Pose2D, "scissor_movement_pub", self.scissors_callback, 1)
        self.camera_sub = self.create_subscription(Int8, "camera_pub", self.camera_callback, 1)

        self.positionalframe_msg = PositionalFrame()

    def scissors_callback(self, scissors_position):
        if self.evaluate:
            self.positionalframe_msg.frame = np.array(self.frame).flatten()
            self.positionalframe_msg.y = scissors_position.y

            self.evaluate_pub.publish(positionalframe_msg)

    def camera_callback(self, frame):
        self.frame = frame.frame

    def service_callback(self, request, response):
        self.evaluate = request.capture
        self.positionalframe_msg.aisle = request.aisle
        self.positionalframe_msg.shelf = request.shelf
        self.positionalframe_msg.x = request.x

        response.status_code = 1
        return response


def main():
    """
    This function initializes the ROS 2 node and keeps it spinning to handle callbacks.
    It ensures the node is properly shut down when the program exits.
    """
    rclpy.init()

    node_launcher = Procedure()

    rclpy.spin(node_launcher)

    node_launcher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    """
    Entry point for the script. If the script is run directly,
    the main() function will be executed.
    """
    main()
