"""Node who can control buffer of writing log files."""

import json

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from vision_interfaces.msg import Detection

from srv_vision.storage_manager import EvaluationsDB


class Data(Node):
    """
    This node was created in order to manage each writing request on database,
    once any of the subscribes receives the execution reformat the information
    as dictionary and using the storage manager saves the json file.
    """

    def __init__(self):
        super().__init__("data_node")
        self.detection_sub = self.create_subscription(Detection, "save_detection", self.detection_callback, 1000)
        self.fetch_sub = self.create_subscription(String, "save_fetch", self.fetch_callback, 1000)

        self.database = EvaluationsDB()

    def fetch_callback(self, fetch):
        code_name, data = fetch.split(":", 1)
        self.database.save_fetch(code_name, data)

    def detection_callback(self, detection):
        data = {
            "x_1": detection.x_1,
            "y_1": detection.y_1,
            "x_2": detection.x_2,
            "y_2": detection.y_2,
            "aisle": detection.aisle,
            "shelf": detection.shelf
        }
        data = json.dumps(data)
        self.database.save_capture(detection.code_name, data)


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
