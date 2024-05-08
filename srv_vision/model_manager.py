"""File created to manage the AI model."""

import numpy as np

import rclpy
from rclpy.node import Node

from ultralytics import YOLO
from sklearn.cluster import MeanShift
from srv_vision.camera_manager import Camera

from std_msgs import Int8MultiArray, Empty


class Evaluation(Node):
    """WIP."""

    def __init__(self):
        """WIP."""
        super().__init__("model_evaluation_node")
        self.model = YOLO(
            "/home/pinole/ros2_ws/src/srv_vision/srv_vision/models/ultralytics_yolov8_model.pt"
        )
        self.__subscription = self.create_subscription(
            Int8MultiArray, "model_evaluate", self.__listener_callback, 1
        )

        self.__server_post_pub = self.create_publisher(
            Empty, "server_post", 1
        )

        self.__data_saver_pub = self.create_publisher(
            String, "data_saver", 10
        )

        self.__subscription
        self.__empty_msg = Empty()
        self.__data_saver_msgs = String()

        self.mean_shift = MeanShift(bandwidth=None, bin_seeding=True)

    def __listener_callback(self, msg):
        self.location = msg.data
        with Camera() as cap:
            frame = cap.get_frame()

        frame_evaluation = self.evaluate(self.frame)



    def evaluate(self, frame) -> list[list[int]]:
        """WIP."""
        result = self.model.predict(frame)
        result = result[0]
        reduce = self.simplify(result.boxes.xywh.numpy(), result.boxes.cls.numpy())
        print(reduce)
        return reduce

    def simplify(
        self, boxes: list[list[float]], cls_prods: list[float]
    ) -> list[list[str]]:
        """Search over boxes to relate."""
        # x_lines = boxes[:,1] + boxes[:,2]
        # x_lines = np.reshape(x_lines, (-1, 1))
        # self.mean_shift.fit(cluster_array)
        # cluster_centers = ms.cluster_centers_
        # labels = self.mean_shift.labels_
        #
        labels_unique = np.unique(cls_prods)
        # n_clusters_ = len(labels_unique)
        # reduced_array = []
        # cols = []
        #
        # for label in labels_unique:
        #
        #   if section.get(label) is None:
        #       section[label] = []
        #
        #   cluster_boxes[label].append([boxes[index], cls[index]])

        # reduced_array = []
        reduced_array = {}

        for label in labels_unique:
            reduced_array[label] = len(cls_prods[cls_prods == label])

        return reduced_array
