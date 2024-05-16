"""WIP."""

from rclpy.node import Node
from std_msgs.msg import Empty
from std_msgs.msg import String

import base64
import requests
import json

from srv_vision.storage_manager import EvaluationsDB, Products
from srv_vision.sort import Sort


class Analize(Node):
    endpoint_s1 = "aHR0cHM6Ly9zM3pmcGQ5bmsxLmV4ZWN1dGUtYXBpLnVzLWVhc3Qt"
    endpoint_s2 = "Mi5hbWF6b25hd3MuY29tL2RlZmF1bHQvaW52ZW50b3J5bGFtYmRh"

    headers = {"Content-Type": "application/json"}

    def __init__(self):
        super().__init__("analize_node")
        self.analize_sub = self.create_subscription(Empty, "analize_captures", self.analize_callback, 1)
        self.save_pub = self.create_publisher(String, "save_fetch", 100)

        self.products = Products()
        self.fetch_msg = String()

    def analize_callback(self, _empty):
        evaluations = EvaluationsDB() # Any write is not permit
        captures = evaluations.get_captures()
        fetches = evaluations.get_fetches()

        for code_name, detections in list(captures.items()):
            total_detections = len(detections)
            if fetches.get(code_name) is None:
                continue

            if fetches[code_name][total_tokens] >= total_detections:
                continue

            if total_detections == 0:
                continue

            deep_sort = Sort()
            stock = int(total_detections > 0)
            for detection in detections:
                location = [detection["x_1"], detection["y_1"], detection["x_2"], detection["y_2"], 1.0]
                response = deep_sort.update(location)
                if len(response) > 0:
                    stock = int(max(np.max(response[:][3]), stock))

            previous_fetches = evaluations.get_previous_fetches(code_name, 10)
            expected_stock = 0
            for fetch in previous_fetches:
                expected_stock = max(fetch["stock"], expected_stock)

            fetch = json.dumps({"stock": stock, "tokens": total_tokens})
            self.fetch_msg.data = str(code_name) + ":" + fetch

            self.save_pub.publish(self.fetch_msg)
            self.post_product(code_name, stock, expected_stock, detections[0]["aisle"], detections[0]["shelf"])


    def post_product(self, code_name: str, stock: int, expected_stock: int, aisle: int, rack: int):
        product_info = self.products.get_product_information(code_name)

        json_send = json.dumps({
            "PLU": product_info["plu"],
            "Product": product_info["name"],
            "Type": product_info["variety"],
            "Aisle": aisle,
            "Rack": rack,
            "AmountS": stock,
            "AmountF": expected_stock,
            "Supply": expected_stock > stock,
        })

        try:
            response = requests.post(
                base64.b64decode(self.endpoint_s1 + self.endpoint_s2).decode("utf-8"),
                json=sender,
                headers=self.headers,
            )
            response.raise_for_status()
        except requests.exceptions.RequestException as e:
            if hasattr(e, "response"):
                return f"Error: code {e.response.status_code} received"
            return "Error: Unreachable server"
        return f"Ok: code {response.status_code} received"
