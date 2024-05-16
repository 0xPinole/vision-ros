"""WIP."""

import datetime
import json

from uuid import uuid4

class Shelves:
    """Class owner of storage of shelves toml file."""

    def __init__(self):
        """On initialization of the class, it reads all data on the database,
        their methods, will permit to acces only the requested data, there is
        no permited to write data on this database file."""
        self.load_shelves()

    def load_shelves(self):
        """"Read and local save the full shelves file."""
        with open("/home/pinole/ros2_ws/src/srv_vision/srv_vision/db/shelves_trace.json", "r") as fg:
            self.shelves = json.load(fg)

    def search_by_aruco_id(self, aruco_id: int) -> list[list[int]]:
        """Search by the aruco_id to get the distances to go down."""
        shelf = self.shelves["aruco_id"][str(aruco_id)]
        return shelf


class Products:
    """WIP"""

    def __init__(self):
        """WIP."""
        self.load_products()

    def load_product_list(self):
        """Load products from json database."""
        with open("/home/pinole/ros2_ws/src/srv_vision/srv_vision/db/products.json", "r") as fg:
            self.products = json.load(fg)

    def get_product_information(self, query: str) -> dict[str, any]:
        """WIP."""
        uuid = self.products["code_name"].get(query)
        if uuid is None:
            return None
        product = self.products["product"].get(uuid)
        return product


class Logs:
    """Class owner of save each camera detecter iteration."""

    def __init__(self):
        """WIP."""
        self._load_logger()

    def _load_logger(self):
        """Load products from json database."""
        with open(
            "/home/pinole/ros2_ws/src/srv_vision/srv_vision/db/logger.json", "r"
        ) as fg:
            self.logs = json.load(fg)

    def _dump_logger(self, logs_json: dict[str, any]):
        """Overwrite json database."""
        with open(
            "/home/pinole/ros2_ws/src/srv_vision/srv_vision/db/logger.json", "w"
        ) as fg:
            json.dump(logs_json, fg)

    def save_logger(self):
        """Save the actual logger."""
        self._dump_logger(self.logs)

    def insert_log(self, location: str, detections: list[list[str]]):
        """Insert at last if time diff is less than 3 hours, else insert new."""
        last_timestamp = datetime.datetime.strptime(
            self.logs["detections"][-1]["timestamp"], "%Y-%m-%d %H:%M:%S"
        )
        now = datetime.datetime.now()

        if (now - last_timestamp).total_seconds() > 10800:  # 3 hours
            now_str = now.strftime("%Y-%m-%d %H:%M:%S")
            self.logs["detections"].append(
                {
                    "timestamp": now_str,
                    "captures": [],
                    "capture_stamp": None,
                    "fetches": {},
                }
            )

        self.logs["detections"][-1]["captures"].append(
            {"location": location, "detections": detections}
        )

    def set_capture_stamp(self, detections: list[list[list[str]]]):
        """Insert the final record of accomodations of objects."""
        self.logs["detections"][-1]["capture_stamp"] = detections

    def get_captures(self) -> list[dict[str, any]]:
        """Return the list of detections images."""
        last_detections = self.logs["detections"][-1]["captures"]
        return last_detections

    def get_capture_stamp(self) -> list[list[list[str]]]:
        """WIP."""
        capture_stamp = self.logs["detections"][-1]["capture_stamp"]
        return capture_stamp

    def insert_fetch(self, fetch_key: str, fetch_data: dict[str, any]):
        """WIP."""
        self.logs["detections"][-1]["fetches"][fetch_key] = fetch_data

    def get_fetches(self) -> dict[str, dict[str, any]]:
        """WIP."""
        fetches = self.logs["detections"][-1]["fetches"]
        return fetches
