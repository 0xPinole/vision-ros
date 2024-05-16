"""Node who can control buffer of writing log files."""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from srv_vision.storage_manager import Logs


class Data(Node):
    """This node was created in order to take control of all trying of writing on databases, once each of the subscribes receives the correspondance execution, it writes on the file and then saves the data, the """

    def __init__(self):
        """WIP."""
        super().__init__("data_saver_service_node")
        self.data_service = self.create_service(
            DataParams, "data_saver", self.__service_callback
        )

    def __service_callback(self, request, response):
        """Callback of service for each request."""

        data = request.saver_json
        if data == "test":
            response.status_code = 500
            return response

        data = json.loads(data)

        if data["type_request"] == "capture":
            logger = Logs()
            logger.insert_log(data["location"], data["totals"])
            logger.save_logger()
            response.status_code = 200
        elif data["type_request"] == "fetches":
            logger = Logs()
            logger.insert_fetch(data["key"], data["fetch"])
            logger.save_logger()
            response.status_code = 200
        else:
            response.status_code = 400
        print(response.status_code)
        return response

def main():
    """Init ros2 and node of vision service."""
    rclpy.init()
    node = DataSaver()

    rclpy.spin(node)
    service.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    """Run main fun."""
    main()
