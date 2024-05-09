"""This main node creates a service who publish requested movements over the full procedure on ."""

import json
import time

import rclpy

from vision_interfaces.srv import VisionParams
from rclpy.node import Node
from std_msgs.msg import Bool, Int8MultiArray, Int32

from srv_vision.storage_manager import Shelves


class VisionService(Node):
    """Node and service in charge of provide a procedure."""

    def __init__(self):
        """Init vision main service."""
        super().__init__("vision_service_node")
        self.vision_service = self.create_service(
            VisionParams, "vision_service", self.__service_callback
        )  # Replace with aligned interface

        self.__shift_scissors_pub = self.create_publisher(Int32, "shift_scissors", 1)

        self.__rotate_camera_pub = self.create_publisher(Int32, "rotate_camera", 1)

        self.__model_evaluation_pub = self.create_publisher(
            Int8MultiArray, "model_evaluate", 10
        )

        self.__shelves = Shelves()

        self.__shift_scissors_msgs = Int32()
        self.__rotate_camera_msgs = Int32()
        self.__model_evaluation_msgs = Int8MultiArray()

        self.__aruco_id = -1
        self.servo_side = True

    def srv_publish_topic(self, topic_name: str, topic_value: any):
        """Simplification of publisher for casting and waiting default methods."""
        if topic_name == "shift_scissors":
            self.__rotate_camera_msgs.data = topic_value
            self.__shift_scissors_pub.publish(self.__rotate_camera_msgs)
            time.sleep(5)
        elif topic_name == "rotate_camera":
            topic_value = 0 if topic_value == "right" else 180
            self.__rotate_camera_msgs.data = topic_value
            self.servo_side = topic_value == 0
            self.__rotate_camera_pub.publish(self.__rotate_camera_msgs)
            time.sleep(3)
        elif topic_name == "model_evaluate":
            self.__model_evaluation_msgs.data = list(topic_value.values())
            self.__model_evaluation_pub.publish(self.__model_evaluation_msgs)

    def __service_callback(self, request, response):
        """Main loop to run all needed procesess."""
        self.__aruco_id = request.aruco_id
        self.__shelves_map = self.__shelves.search_by_aruco_id(self.__aruco_id)
        setpoints = (
            self.__shelves_map["left"]["setpoints"]
            + self.__shelves_map["right"]["setpoints"]
        )

        location = {
            "aisle": -1,
            "shelf": -1,
            "aruco_id": self.__aruco_id,
            "setpoint_index": -1,
        }

        for i, _setpoint in enumerate(sorted(list(dict.fromkeys(setpoints)))):
            location["setpoint_index"] = i
            self.srv_publish_topic("shift_scissors", _setpoint)
            for side_key in ["right", "left"] if self.servo_side else ["left", "right"]:
                if _setpoint not in self.__shelves_map[side_key]["setpoints"]:
                    continue
                location.update(
                    {
                        "aisle": self.__shelves_map[side_key]["aisle"],
                        "shelf": self.__shelves_map[side_key]["shelf"],
                    }
                )

                self.srv_publish_topic("rotate_camera", side_key)
                self.srv_publish_topic("model_evaluate", location)

        self.srv_publish_topic("shift_scissors", 0)
        self.srv_publish_topic("rotate_camera", 0)

        response.status_code = 1
        return response

def main():
    """Init ros2 and node of vision service."""
    rclpy.init()
    service = VisionService()

    rclpy.spin(service)
    service.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    """Run main fun."""
    main()
