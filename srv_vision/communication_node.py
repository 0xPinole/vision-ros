"""WIP."""

import json

from rclpy.node import Node
from std_msgs.msg import String

from srv_vision.aws_manager import Client
from srv_vision.storage_manager import Logs, Products


class DataManager(Node):
    """WIP."""

    def __init__(self):
        """WIP."""
        super().__init__("communication_node")
        self.subscription = self.create_subscription(
            String, "aws_communication", self.listener_callback, 1
        )

        self.logger_writer = self.create_publisher(String, "write_file", 10)

        # self.products = Products()
        # self.logs = Logs()
        self.aws_client = Client()  # Future feature expected to remove this line

        self.subscription  # Prevent unused variable warning.
        self.products = Products()

    def listener_callback(self, msg):
        """Callback for the subscriber."""
        if msg.data == "POST":
            logger = Logs()
            captures = logger.get_captures()
            fetches = logger.get_fetches()
            fetches_msg = String()

            for capture in captures:
                for class_prod, stock in capture["totals"].items():
                    position = capture["location"].split(",")
                    if fetches.get(class_prod) is None:
                        side = position[0]
                        aisle = position[1] if "R" in side else position[1] + 1
                        shelf = position[2]
                        jpost = self.post_product(class_prod, stock, aisle, shelf)
                        fetches_msg.data = json.dumps(
                            {"key": class_prod, "fetch": jpost}
                        )
                        self.logger_writer.publish(fetches_msg)

    def filter_valid_fetches(self, fetches) -> list[dict[str, any]]:
        """Check if at the previous fetches of the same product, has the same data."""
        if fetches is []:
            return []
        return []

    def generate_capture_stamp(self, capture_stamp: list[list[str]]):
        """Algorithm to group captures into one array of detection."""
        # captures = self.logs.get_captures()
        # For comparation with errors
        # capture_stamp = self.logs.get_capture_stamp()
        # Your logic here ...
        # Result data on self
        # self.grouped = []
        # self.logs.insert_capture_stamp(self.grouped)
        pass

    def search_available_products(self):
        """Search over the self.grouped for products that are separate and ready to send."""
        # If from self.group are some available group to send on server, then do
        # reduce of has been sended before
        # self.logs.get_fetches() {vision_hash: {stock, location, ...} }
        # for i in fetches2send
        # self.logs.insert_fetch(vision_hash, {...})
        pass

    def post_product(self, vision_hash: str, stock: str, aisle: str, rack: str):
        """WIP."""
        product_info = self.products.get_product_by_hash_vision(vision_hash)
        if product_info is not None:
            json_send = {
                "PLU": product_info["plu"],
                "Product": product_info["name"],
                "Type": product_info["variety"],
                "Aisle": aisle,
                "Rack": rack,
                "AmountS": stock,
                "AmountF": "0",
                "Supply": False,
            }

            self.aws_client.post(json_send)

        return json_send
