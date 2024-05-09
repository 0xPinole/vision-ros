"""File created to manage the AI model."""

import numpy as np

import rclpy
from rclpy.node import Node
import json

import time

from ultralytics import YOLO
from sklearn.cluster import MeanShift
from srv_vision.camera_manager import Camera

from std_msgs.msg import Int8MultiArray, Empty
from vision_interfaces.srv import DataParams


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

        self.__server_post_pub = self.create_publisher(Empty, "server_post", 1)
        self.__data_saver_client = self.create_client(DataParams, 'data_saver')
        while not self.__data_saver_client.wait_for_service(timeout_sec=1.0):
            pass

        self.__subscription
        self.__empty_msg = Empty()

        self.mean_shift = MeanShift(bandwidth=None, bin_seeding=True)

        self.__data_saver_request = DataParams.Request()
        self.camera = Camera()
        self.waiting = True
        self.result = 200

    async def data_saver_service_request(self, data_json: str):
        self.__data_saver_request.saver_json = data_json
        print("callback created")
        self.future = self.__data_saver_client.call_async(self.__data_saver_request)
        print("future declare")
        self.waiting = True
        self.result = await self.future
        self.waiting = False
        #rclpy.spin_until_future_complete(self, self.future)
        print(self.result)
        print("spin end")
        return self.result

    async def __listener_callback(self, msg):
        print("init listener")
        location = msg.data
        location = location.tolist()

        frame = self.camera.get_frame()

        frame_evaluation = self.evaluate(frame)

        data_saver_req = {
            "type_request": "capture",
            "location": location,
            "totals": frame_evaluation
        }

        data_saver_req = json.dumps(data_saver_req)

        self.__data_saver_request.saver_json = data_saver_req

        self.__data_saver_client.call_async(self.__data_saver_request)

        time.sleep(2)
        self.__server_post_pub.publish(self.__empty_msg)

    def evaluate(self, frame) -> list[list[int]]:
        """WIP."""
        result = self.model.predict(frame)
        result = result[0]
        bxs = result.boxes.cpu()
        coords_bx = bxs.xywh.numpy()
        classes_bx = bxs.cls.numpy()
        reduce = self.simplify(coords_bx, classes_bx)
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
            label_val = label.item()
            reduced_array[label_val] = len(cls_prods[cls_prods == label])

        return reduced_array


def main():
    """Init ros2 and node of vision service."""
    rclpy.init()
    node = Evaluation()

    rclpy.spin(node)
    service.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    """Run main fun."""
    main()
