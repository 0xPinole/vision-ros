#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Vision Service Node for ROS 2.

This node manages a service that handles requested movements for the procedure. It provides
the routine for moving the scissors mechanism and evaluating products from each side of the
aisle. The node is responsible for coordinating the movement of the scissors, capturing
video/images for evaluation, and updating the aisle and rack positions.

Example:
    To run the node with a specific log level.

        $ ros2 run srv_vision node_service --log-level DEBUG

Definitions:
    - left:
        rotated at 0 degrees
    - right:
        rotated at 180 degrees

Custom Messages:
    - VisionParams (Service):
        uint8 aruco_id
        ---
        uint8 status_code

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
"""

import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity

from vision_interfaces.srv import VisionParams
from vision_interfaces.srv import ProcedureParams
from vision_interfaces.srv import StateParams

from std_msgs.msg import Empty
from geometry_msgs.msg import Pose2D

from srv_vision.storage_manager import Shelves

from time import sleep
import argparse

class VisionService(Node):
    aruco_id = -1

    def __init__(self, log_level=LoggingSeverity.INFO):
        """Initialize the VisionService node.

        Args:
            log_level (LoggingSeverity): The logging severity level.
        """
        super().__init__("vision_service_node")
        self.get_logger().set_level(log_level)
        self.vision_service = self.create_service(VisionParams, "vision_service", self.service_callback)

        self.scissors_pub = self.create_publisher(Pose2D, "scissors_movement", 1)
        self.communication_pub = self.create_publisher(Empty, "analize_captures", 1)

        self.process_srv = self.create_client(ProcedureParams, "procedure_init")
        while not self.process_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"[!] service procedure not available, waiting until available.")

        self.state_srv = self.create_client(StateParams, "service_state")
        while not self.state_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"[!] service state not available, waiting until available.")

        self.shelves = Shelves()
        self.scissors_msg = Pose2D()
        self.process_msg = ProcedureParams.Request()
        self.empty_msg = Empty()
        self.state_msg = StateParams.Request()

        self.get_logger().info("[+] Vision node initialized.")


    async def service_callback(self, request, response):
        """Callback function for the vision service.

        Sets the required movement for the scissors mechanism and triggers the evaluation
        node to capture video/images for evaluation.

        Args:
            request (VisionParams): The request message containing the ARUCO ID.
            response (VisionParams.Response): The response message indicating the status code.
        """
        shelf_map = self.shelves.get_shelf(request.aruco_id)
        self.process_msg.x = (request.aruco_id * 160) + 80 + (request.aruco_id * 5)
        self.get_logger().info(f"[+] Received service request: {request.aruco_id}.")
        self.scissors_msg.theta = 0.0
        self.scissors_msg.x = 0.0
        self.scissors_msg.y = 0.0

        if shelf_map.get("state") == "start":
            self.state_msg.state = True
            self.state_srv.call_async(self.state_msg)

        for side in ["left", "right"]:
            if shelf_map.get(side) is not None:
                self.get_logger().debug(f"[>] Evaluation {side} side.")
                self.scissors_msg.theta = 9.0 if side == "right" else 180.0
                self.scissors_pub.publish(self.scissors_msg)
                self.get_logger().debug(f"[>] Publish theta: {self.scissors_msg.theta}.")

                sleep(4)

                self.process_msg.capture = True
                self.process_msg.aisle = shelf_map[side].get("aisle")
                self.process_msg.shelf = shelf_map[side].get("shelf")
                self.process_srv.call_async(self.process_msg)
                self.get_logger().debug(f"[>] Call async capture start.")

                self.scissors_msg.y = int(self.scissors_msg.y == 0.0) * 50.0

                # Snipped code to stop and stabilize taking steps
                # if self.scissors_msg.y == 0.0:
                #     route = range(10, 51, 10)
                # else:
                #     route = range(50, 1, -10)
                #
                # for step in route:
                #     self.scissors_msg.y = step # from 10 to 50 cm or viceversa
                #     self.scissors_pub.publish(self.scissors_msg)
                #     sleep(2) # sleep 2 sec to stabilize

                self.scissors_pub.publish(self.scissors_msg)
                self.get_logger().debug(f"[>] Publish y: {self.scissors_msg.y}.")
                sleep(2)

                self.process_msg.capture = False
                self.process_srv.call_async(self.process_msg)
                self.get_logger().debug(f"[>] Call async capture finish.")

        self.scissors_msg.theta = 0.0
        self.scissors_msg.y = 0.0
        self.scissors_pub.publish(self.scissors_msg)
        self.get_logger().info(f"[>] Scissors mechanism reset to y: 0.0.")

        self.communication_pub.publish(self.empty_msg)
        self.get_logger().info(f"[>] Scissors mechanism reset to y: 0.0.")
        sleep(10)

        if shelf_map.get("state") == "finish":
            self.state_msg.state = False
            self.state_srv.call_async(self.state_msg)

        response.status_code = 1
        self.get_logger().info(f"[~] Vision service response code: {response.status_code}")
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

    node_launcher = VisionService(log_level)

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
