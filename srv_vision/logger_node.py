"""Node who can control buffer of writing log files."""

import json

from rclpy.node import Node
from std_msgs.msg import String

from srv_vision.storage_manager import Logs


class LoggerFiles(Node, Logs):
    """WIP."""

    def __init__(self):
        """WIP."""
        super().__init__("logs_manager_node")
        self.subscription = self.create_subscription(
            String, "write_file", self.listener_callback, 10
        )

        self.subscription
        self.logger = Logs()

    def listener_callback(self, msg):
        """WIP."""
        type_request, callback_request = msg.data.split(":", 1)
        data = json.loads(callback_request)

        if type_request == "capture":
            self.logger.insert_log(data["location"], data["totals"])
        elif type_request == "fetch":
            self.logger.insert_fetch(data["key"], data["fetch"])

        self.logger.save_logger()

        self.logger = Logs()
