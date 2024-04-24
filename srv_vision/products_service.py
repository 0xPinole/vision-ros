"""WIP."""

from aws_server import Client
from storage_manager import Logs, Products


class DataManager:
    """WIP."""

    def __init__(self):
        """WIP."""
        self.products = Products()
        self.logs = Logs()
        self.aws_client = Client()
        self.grouped = []

    def generate_capture_stamp(self):
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
