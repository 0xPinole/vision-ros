"""Node who can control buffer of writing log files."""

import json

from rclpy.node import Node
from std_msgs.msg import String

from srv_vision.storage_manager import Logs

from vision_interfaces.srv import DataParams


class DataSaver(Node):
    """WIP."""

    def __init__(self):
        """WIP."""
        super().__init__("data_saver_service_node")
        self.data_service = self.create_service(
            DataParams, "data_saver", self.__service_callback
        )

    def __service_callback(self, request, response):
        """Callback of service for each request."""

        data = request.saver_json
        data = json.loads(data)

        if data["type_request"] == "capture":
            logger = Logs()
            logger.insert_log(data["location"], data["totals"])
            logger.save_logger()
            response.status_code = 200
        elif data["type_request"] == "capture":
            logger = Logs()
            logger.insert_fetch(data["key"], data["fetch"])
            logger.save_logger()
            response.status_code = 200
        else:
            response.status_code = 400

        return response
