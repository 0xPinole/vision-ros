"""File created to manage the AI model."""

import rclpy
from rclpy.node import Node

from vision_interfaces.msg import PositionalFrame, Detection

import numpy as np
from ultralytics import YOLO


class Evaluation(Node):
    """
    Ros node who subscribes positional frames and evaluate against a yolo model,
    it locate the product axis shifted by the scissors mechanism position,
    then crosses the class name with the uuid4 from product primary key
    and finally saves the lecture at logger database.
    """
    frame_width_px = 480
    frame_height_px = 640
    frame_width_cm = 60
    frame_height_cm = 40

    def __init__(self):
        """WIP."""
        super().__init__("evaluation_node")

        self.data_pub = self.create_publisher(Detection, "save_detection", 1000)
        self.evaluate_sub = self.create_subscription(PositionalFrame, "evaluation", self.evaluation_callback, 1000)

        self.model = YOLO("/home/pinole/ros2_ws/src/srv_vision/srv_vision/models/ultralytics_yolov8_model.pt")

    async def evaluation_callback(self, positionframe_msg):
        frame_base_x = int((positionframe_msg.x - (self.frame_width_cm/2)) * (self.frame_width_px / self.frame_width_cm))
        frame_base_y = int((positionframe_msg.y - (self.frame_height_cm/2)) * (self.frame_height_px / self.frame_height_cm))

        frame = np.array(positionframe_msg.frame).reshape(480, 640)
        result = self.model.predict(frame) # benchmarking with parameters
        result = result[0]
        result = result.boxes.cpu()
        res_xyxy = result.xyxy.numpy()
        res_cls = result.cls.numpy()
        for index, cls_det in enumerate(res_cls):
            detection = Detection()
            detection.code_name = self.model.names[cls_det]
            detection.aisle = positionframe_msg.aisle
            detection.shelf = positionframe_msg.shelf

            x_1, y_1, x_2, y_2 = res_xyxy[index]
            detection.x_1 = frame_base_x + x_1
            detection.y_1 = frame_base_y + y_1
            detection.x_2 = frame_base_x + x_2
            detection.y_2 = frame_base_y + y_2
            self.data_pub.publish(detection)

def main():
    """
    This function initializes the ROS 2 node and keeps it spinning to handle callbacks.
    It ensures the node is properly shut down when the program exits.
    """
    rclpy.init()

    node_launcher = Evaluation()

    rclpy.spin(node_launcher)

    node_launcher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    """
    Entry point for the script. If the script is run directly,
    the main() function will be executed.
    """
    main()
