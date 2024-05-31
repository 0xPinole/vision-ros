#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Procedure Node for ROS 2.

This node subscribes to camera frames and positional data, reshapes the frame,
and publishes it for evaluation. The node can be enabled or disabled via a
service call.

Example:
    To run the node with a specific log level.

        $ ros2 run srv_vision node_procedure --log-level DEBUG

Custom Messages:
    - ProcedureParams (Service):
        uint8 aisle
        uint8 shelf
        uint8 x
        bool capture
        ---
        uint8 status_code

    - PositionalFrame (Message):
        uint8[] frame
        uint8 aisle
        uint8 shelf
        uint8 x
        uint8 y

Links:
    https://github.com/ros-drivers/usb_cam/blob/ros2/scripts/show_image.py
"""
import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity

from vision_interfaces.srv import ProcedureParams
from vision_interfaces.msg import PositionalFrame

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D

import numpy as np
import argparse

class Procedure(Node):
    """Ros node subscriber to information and trigger of task.

    This node, when activated, captures each camera frame and retrieves
    positional information. It then sends a request to the evaluation node
    with the captured frame and positional data for product prediction.
    """
    evaluate = False
    frame = None
    prev_status = False
    mat = None
    side = False

    def __init__(self, log_level=LoggingSeverity.INFO):
        """Initializes the Procedure node and sets up subscribers and publishers.

        Args:
            log_level (LoggingSeverity): The logging level for the node.
        """
        super().__init__("procedure_node")
        self.get_logger().set_level(log_level)
        self.service_setter = self.create_service(ProcedureParams, "procedure_init", self.service_callback)
        self.evaluate_pub = self.create_publisher(PositionalFrame, "evaluation", 1000)
        self.scissors_sub = self.create_subscription(Pose2D, "scissors_movement_pub", self.scissors_callback, 1)
        self.camera_sub = self.create_subscription(Image, "image_raw", self.camera_callback, 1)

        self.positionalframe_msg = PositionalFrame()
        self.get_logger().info("[+] Procedure node initialized.")

    def scissors_callback(self, scissors_position):
        """Callback function for the scissor movement topic.

        Publishes positional data and captured frame when evaluation is enabled.

        Args:
            scissors_position (Pose2D): The position of the scissors.
        """
        if self.evaluate:
            frame = np.array(self.frame, dtype=np.uint8).flatten().tolist()
            if self.side:
                frame = cv2.flip(frame, 0)
            self.positionalframe_msg.frame = frame
            self.positionalframe_msg.y = int(scissors_position.y)

            self.evaluate_pub.publish(self.positionalframe_msg)
            self.get_logger().info("[+] Published positional frame with y: {} and frame lon: {}".format(scissors_position.y, len(self.positionalframe_msg.frame)))


    def camera_callback(self, msg):
        """Callback function for the camera image topic.

        Processes the incoming image data and stores it in a frame.

        Args:
            msg (Image): The incoming image message.
        """
        sz = (msg.height, msg.width)
        self.get_logger().debug("[?] Received image with the following properties: "
            "encoding={}, width={}, height={}, step={}, data_size={}".format(
            msg.encoding, msg.width, msg.height, msg.step, len(msg.data)))

        dirty = (self.mat is None or msg.width != self.mat.shape[1] or
                 msg.height != self.mat.shape[0] or len(self.mat.shape) < 2 or
                 self.mat.shape[2] != 3)

        if dirty:
            self.mat = np.zeros([msg.height, msg.width, 3], dtype=np.uint8)
            self.get_logger().info("[!] Initialized new matrix for image processing.")

        self.mat[:, :, 2] = np.array(msg.data[0::3]).reshape(sz)
        self.mat[:, :, 1] = np.array(msg.data[1::3]).reshape(sz)
        self.mat[:, :, 0] = np.array(msg.data[2::3]).reshape(sz)

        if self.mat is not None:
            self.frame = self.mat
            self.get_logger().debug("[~] Updated frame with new image data.")

    def service_callback(self, request, response):
        """Callback function for the service to enable/disable data capture.

        Args:
            request (ProcedureParams.Request): The service request.
            response (ProcedureParams.Response): The service response.

        Returns:
            ProcedureParams.Response: The response indicating status.
        """
        self.evaluate = request.capture
        self.positionalframe_msg.aisle = request.aisle
        self.positionalframe_msg.shelf = request.shelf
        self.positionalframe_msg.x = request.x
        self.side = request.side

        response.status_code = 1
        self.get_logger().info("[+] Service request processed: capture={}, aisle={}, shelf={}, x={}".format(
            request.capture, request.aisle, request.shelf, request.x))
        return response


def main():
    """Main function to initialize and spin the ROS node.

    Parses arguments for logging level and starts the node.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--log-level', type=str, default='INFO', help='Logging level (DEBUG, INFO, WARN, ERROR)')
    args = parser.parse_args()

    log_level = LoggingSeverity.INFO
    if args.log_level.upper() == 'DEBUG':
        log_level = LoggingSeverity.DEBUG
    elif args.log_level.upper() == 'ERROR':
        log_level = LoggingSeverity.ERROR

    rclpy.init()

    node_launcher = Procedure(log_level)

    try:
        rclpy.spin(node_launcher)
    except KeyboardInterrupt:
        pass
    finally:
        node_launcher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    """Entry point for the script. Executes main function
    if the script is run directly."""
    main()
