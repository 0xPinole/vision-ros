#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Data Node Manager for Ros 2.

This node was created in order to manage each writing request on database,
once any of the subscribes receives the execution reformat the information
as dictionary and using the storage manager save it into json file.

Example:
    To run the node with a specific log level.
        $ ros2 run srv_vision node_data --log-level DEBUG

Custom Messages:
    - Detection (Message):
        uint16 x_1
        uint16 y_1
        uint16 x_2
        uint16 y_2
        uint8 aisle
        uint8 shelf
        float16 conf
        string code_name
"""

import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity

from std_msgs.msg import String
from vision_interfaces.msg import Detection

from srv_vision.storage_manager import EvaluationsDB

import json
import argparse

class Data(Node):
    def __init__(self, log_level=LoggingSeverity.INFO):
        """Initialize the data manager node.

        Args:
            log_level (LoggingSeverity): The logging severity level.
        """
        super().__init__("data_node")
        self.get_logger().set_level(log_level)

        self.detection_sub = self.create_subscription(Detection, "save_detection", self.detection_callback, 1000)
        self.fetch_sub = self.create_subscription(String, "save_fetch", self.fetch_callback, 1000)

        self.database = EvaluationsDB()
        self.get_logger().info("[+] Data node initialized.")

    def fetch_callback(self, fetch):
        """
        Callback function for evaluating positional frames.

        Args:
            fetch (string): Formated as code_name:fetch_json_file.
        """
        code_name, data = str(fetch.data).split(":", 1)
        self.database.save_fetch(code_name, data)
        self.get_logger().info(f"[+] Fetch received {code_name}.")
        self.get_logger().debug(f"[~] Fetch data: {data}.")

    def detection_callback(self, detection):
        """
        Callback function for evaluating positional frames.

        Args:
            detection (Detection): Object detection data from positional frame.
        """
        self.get_logger().debug(f"[~] Debug::Callback detection.")
        self.get_logger().debug(f"[~] Debug::detection = {detection}.")
        data = {
            "x_1": detection.x_1,
            "y_1": detection.y_1,
            "x_2": detection.x_2,
            "y_2": detection.y_2,
            "aisle": detection.aisle,
            "shelf": detection.shelf,
            "conf": detection.conf
        }
        self.get_logger().debug(f"[~] Debug::data = {data}.")
        data = json.dumps(data)
        self.get_logger().debug(f"[~] Debug::data = {data}.")
        self.database.save_capture(detection.code_name, data)
        self.get_logger().info(f"[+] Detection received {detection.code_name}.")
        self.get_logger().debug(f"[~] Detection data: {data}.")

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

    node_launcher = Data(log_level)

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
