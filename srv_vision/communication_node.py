"""WIP."""

from aws_manager import Client
from rclpy.node import Node
from std_msgs.msg import String
from storage_manager import Logs, Products


class DataManager(Node):
    """WIP."""

    def __init__(self):
        """WIP."""
        super().__init__("communication_node")
        self.subscription = self.create_subscription(
            String, "topic", self.listener_callback, 10
        )

        # self.products = Products()
        # self.logs = Logs()
        self.aws_client = Client()  # Future feature expected to remove this line

        self.subscription  # Prevent unused variable warning.

    def listener_callback(self, msg):
        """Callback for the subscriber."""
        if msg.data == "POST":
            with Logs() as logger:
                captures = logger.get_captures()
                capture_stamp = self.generate_capture_stamp(captures)
                logger.set_capture_stamp(capture_stamp)
                fetches = logger.get_fetches()

            with Products() as products:
                fetches = self.search_available_products(capture_stamp)
                fetches = self.filter_valid_fetches(fetches)

                for fetch in fetches:
                    fetch["product_info"] = products.get_product_by_hash_vision(
                        fetch["vision_hash"]
                    )

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

    def post_product(self, vision_hash: str, stock: str):
        """WIP."""
        product_info = self.products.get_product_by_hash_vision(vision_hash)
        if product_info is not None:
            json_send = {
                "PLU": product_info["plu"],
                "Product": product_info["name"],
                "Type": product_info["variety"],
                "Aisle": "0",
                "Rack": "0",
                "AmountS": stock,
                "AmountF": "0",
                "Supply": False,
            }

            self.aws_client.post(json_send)

        # True On success, else False
        return True
