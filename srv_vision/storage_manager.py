"""WIP."""

import datetime
import json

# import tomllib
from uuid import uuid4

try:
    import tomllib
except ModuleNotFoundError:
    import pip._vendor.tomli as tomllib


class Shelves:
    """Class owner of storage of shelves toml file."""

    def __init__(self):
        """WIP."""
        self._load_shelves()

    def __enter__(self):
        """WIP."""
        pass

    def __exit__(self, exception_type, exception_val, trace):
        """WIP."""
        return True

    def _load_shelves(self):
        """Read and get the toml file of shelves."""
        with open("db/shelves_trace.toml", "rb") as fg:
            self.shelves = tomllib.load(fg)

    def _search_index(
        self, array: list[dict[str, any]], match_key: str, match_value: any
    ) -> int:
        """Get index for seacrh over list of dictionaries."""
        if array is None:
            return None
        for index, value in enumerate(array):
            if value.get(match_key) == match_value:
                return index
        return None

    def get_shelves(self) -> dict[str, any]:
        """Get complete schemer."""
        return self.shelves

    def search_shelves(self, aisle: int, shelf: int, section: int) -> list[int]:
        """Get distances of specific schemer location."""
        id_aisle = self._search_index(self.shelves.get("schemer"), "aisle_index", aisle)
        if id_aisle is None:
            return None

        db_aisle = self.shelves["schemer"][id_aisle]
        if db_aisle.get("enable") is False:
            return None

        id_shelves = self._search_index(db_aisle["shelves"], "shelf_index", shelf)
        if id_shelves is None:
            return None

        db_shelves = db_aisle["schemer"][id_shelves]
        if db_shelves.get("enable") is False:
            return None

        id_section = self._search_index(
            db_shelves["sections"], "section_index", section
        )
        if id_section is None:
            return None

        db_section = db_shelves["sections"][id_section]
        if db_section.get("enable") is False:
            return None
        distances = db_section.get("distances")
        return distances

    def search_by_aruco_id(self, aruco_id: int) -> list[list[int]]:
        """Search by the aruco_id to get the distances to go down."""
        return [2, 1, 1]
        # Return [[aisle, shelf, section], ...], should work with left and right
        # by len(return) === 1 or 2


class Products:
    """Class owner of storage products on json file."""

    def __init__(self):
        """WIP."""
        self._load_product_list()

    def __enter__(self):
        """WIP."""
        pass

    def __exit__(self, exception_type, exception_val, trace):
        """WIP."""
        self._dump_product_list(self.products)
        return True

    def _load_product_list(self):
        """Load products from json database."""
        with open("db/products.json", "r") as fg:
            self.products = json.load(fg)

    def _dump_product_list(self, products_json: dict[str, any]):
        """Overwrite json database."""
        with open("db/products.json", "w") as fg:
            json.dump(products_json, fg)

    def get_product_by_uuid(self, query: str) -> dict[str, any]:
        """Return the product information."""
        product = self.products["products"].get(query)
        return product

    def get_product_by_hash_vision(self, query: str) -> dict[str, any]:
        """WIP."""
        uuid = self.products["vision_hash"].get(query)
        if uuid is None:
            return None
        product = self.get_product_by_uuid(uuid)
        return product

    def insert_product(self, product_information: dict[str, any]) -> str:
        """Return uuid of insertion."""
        while True:
            uuid = uuid4()
            if self.products["products"].get(uuid) is None:
                self.producs["products"][uuid] = product_information
                return uuid

    def search_product_name(self, name: str) -> list[list[str]]:
        """Search product by has in name."""
        response = []
        for uuid, product in self.products["products"].items():
            if name in product["name"]:
                response.append(
                    [uuid, product["name"], product["variety"], product["plu"]]
                )
        return response


class Logs:
    """Class owner of save each camera detecter iteration."""

    def __init__(self):
        """WIP."""
        self._load_logger()

    def __enter__(self):
        """WIP."""
        pass

    def __exit__(self, exception_type, exception_val, trace):
        """WIP."""
        self._dump_logger(self.logs)
        return True

    def _load_logger(self):
        """Load products from json database."""
        with open("db/logger.json", "r") as fg:
            self.logs = json.load(fg)

    def _dump_logger(self, logs_json: dict[str, any]):
        """Overwrite json database."""
        with open("db/logger.json", "w") as fg:
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
