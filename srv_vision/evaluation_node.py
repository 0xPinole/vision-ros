#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS 2 Node for evaluating positional frames against a YOLO model.

This node subscribes to positional frames, evaluates them using a YOLO model,
and publishes the detection results.

Example:
    To run the node with a specific log level.
        $ ros2 run srv_vision node_evaluation --log-level DEBUG

Custom Messages:
    - Detection (Message):
        uint16 x_1
        uint16 y_1
        uint16 x_2
        uint16 y_2
        uint8 aisle
        uint8 shelf
        string code_name

    - PositionalFrame (Message):
        uint8[] frame
        uint8 aisle
        uint8 shelf
        uint8 x
        uint8 y

Glossary:
    - YOLO: You Only Look Once, a state-of-the-art object detection model.
"""

import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity

from vision_interfaces.msg import PositionalFrame
from vision_interfaces.msg import Detection

import time
import argparse
import numpy as np
from PIL import Image
from ultralytics import YOLO


class Evaluation(Node):
    frame_width_px = 640
    frame_height_px = 480
    frame_width_cm = 160
    frame_height_cm = 60
    #frame_width_cm = 60
    #frame_height_cm = 40
    counter = 0

    def __init__(self, log_level=LoggingSeverity.INFO):
        """Initialize the evaluation node.

        Args:
            log_level (LoggingSeverity): The logging severity level.
        """
        super().__init__("evaluation_node")
        self.get_logger().set_level(log_level)

        self.data_pub = self.create_publisher(Detection, "save_detection", 1000)
        self.evaluate_sub = self.create_subscription(PositionalFrame, "evaluation", self.evaluation_callback, 1000)
        self.get_logger().debug("[>] Topic connection done.")

        self.model = YOLO("/home/pinole/ros2_ws/src/srv_vision/srv_vision/models/ultralytics_yolov8_model.pt")
        self.get_logger().info("[+] Evaluation node initialized.")

    async def evaluation_callback(self, positionframe_msg):
        """
        Callback function for evaluating positional frames.

        Args:
            positionframe_msg (PositionalFrame): The positional frame message to evaluate.
        """
        self.get_logger().debug("[~] Callback evaluation.")
        frame_base_x = int((positionframe_msg.x - (self.frame_width_cm/2)) * (self.frame_width_px / self.frame_width_cm))
        frame_base_y = int((positionframe_msg.y - (self.frame_height_cm/2)) * (self.frame_height_px / self.frame_height_cm))

        frame = np.array(positionframe_msg.frame).reshape(self.frame_height_px, self.frame_width_px, 3)
        self.get_logger().debug("[~] Frame reshaped for YOLO evaluation.")
        start_time = time.time()
        result = self.model.predict(frame, conf=0.79)
        result = result[0]
        result = result.boxes.cpu()
        res_xyxy = result.xyxy.numpy()
        res_cls = result.cls.numpy()
        res_conf = result.conf.numpy()

        self.get_logger().info(f"[~] YOLO evaluation completed x: {frame_base_x}, y: {frame_base_y}.")
        for index, cls_det in enumerate(res_cls):
            detection = Detection()
            detection.code_name = self.model.names[cls_det]
            detection.aisle = positionframe_msg.aisle
            detection.shelf = positionframe_msg.shelf
            detection.conf = float(res_conf[index])

            x_1, y_1, x_2, y_2 = res_xyxy[index]
            self.get_logger().debug(f"X detected: {int(frame_base_x + x_1)}")
            detection.x_1 = int(frame_base_x + x_1)
            detection.y_1 = int(frame_base_y + y_1)
            detection.x_2 = int(frame_base_x + x_2)
            detection.y_2 = int(frame_base_y + y_2)
            self.data_pub.publish(detection)
            self.get_logger().debug(f"Published detection: {detection}")

        self.get_logger().debug(f"[%] Evaluation taken time: {time.time() - start_time}")

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

    node_launcher = Evaluation(log_level)

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
