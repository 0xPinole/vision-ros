"""WIP."""

import datetime
import json

class Shelves:

    def __init__(self):
        """On initialization of the class, it reads all data on the database,
        their methods, will permit to acces only the requested data, there is
        no permited to write data on this database file."""
        self.load_shelves()

    def load_shelves(self):
        with open("/home/pinole/ros2_ws/src/srv_vision/srv_vision/db/shelves_trace.json", "r") as fg:
            self.shelves = json.load(fg)

    def get_shelf(self, aruco_id: int):
        shelf = self.shelves["aruco_id"][str(aruco_id)]
        return shelf


class Products:
    def __init__(self):
        self.load_products()

    def load_products(self):
        with open("/home/pinole/ros2_ws/src/srv_vision/srv_vision/db/products.json", "r") as fg:
            self.products = json.load(fg)

    def get_product_information(self, query: str) -> dict[str, any]:
        uuid = self.products["code_name"].get(query)
        if uuid is None:
            uuid = self.products["code_name"].get("default")
        product = self.products["product"].get(uuid)
        return product

    def get_code_name(self):
        return list(self.products["code_name"].keys())


class EvaluationsDB:
    def __init__(self):
        self.read_file()

    def read_file(self):
        with open("/home/pinole/ros2_ws/src/srv_vision/srv_vision/db/evaluations.json", "r") as fg:
            self.logs = json.load(fg)

    def write_file(self):
        with open("/home/pinole/ros2_ws/src/srv_vision/srv_vision/db/evaluations.json", "w") as fg:
            json.dump(self.logs, fg)

    def save_capture(self, code_name: str, data: dict[str, any]):
        self.read_file()
        last_timestamp = self.logs["execution"][-1]["timestamp"]
        last_timestamp = datetime.datetime.strptime(last_timestamp, "%Y-%m-%d %H:%M:%S")
        now_timestamp = datetime.datetime.now()

        if (now_timestamp - last_timestamp).total_seconds() > 10800:  # 3 hours
            now_str = now_timestamp.strftime("%Y-%m-%d %H:%M:%S")
            self.logs["execution"].append({
                    "timestamp": now_str,
                    "capture": {},
                    "fetch": {},
            })

        if self.logs["execution"][-1]["capture"].get(code_name) is None:
            self.logs["execution"][-1]["capture"][code_name] = []

        self.logs["execution"][-1]["capture"][code_name].append(data)

        self.write_file()

    def save_fetch(self, code_name: str, data: dict[str, any]):
        self.read_file()
        self.logs["execution"][-1]["fetch"][code_name] = data
        self.write_file()

    def get_captures(self):
        predictions = self.logs["execution"][-1]["capture"]
        return predictions

    def get_previous_fetches(self, code_name: str, lookback: int = 2):
        response = []
        top_lookback = len(self.logs["execution"])
        lookback = min(top_lookback, lookback)

        for index in range(-1, -lookback -1, -1):
            release = self.logs["execution"][index]["fetch"].get(code_name)
            response.append(release)

        return response

    def get_fetches(self):
        fetches = self.logs["execution"][-1]["fetch"]
        return fetches
