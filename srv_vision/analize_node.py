#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Data Node Manager for Ros 2.

This node was created in order to manage each writing request on database,
once any of the subscribes receives the execution reformat the information
as dictionary and using the storage manager save it into json file.

Example:
    To run the node with a specific log level.
        $ ros2 run srv_vision node_analize --log-level DEBUG
"""

import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity

from vision_interfaces.srv import StateParams

from std_msgs.msg import Empty
from std_msgs.msg import String

from srv_vision.sort import Sort
from srv_vision.storage_manager import EvaluationsDB
from srv_vision.storage_manager import Products

import json
import base64
import argparse
import requests
import numpy as np
from time import sleep

class Analize(Node):
    # endpoint_s1 = "aHR0cHM6Ly9zM3pmcGQ5bmsxLmV4ZWN1dGUtYXBpLnVzLWVhc3Qt"
    # endpoint_s2 = "Mi5hbWF6b25hd3MuY29tL2RlZmF1bHQvaW52ZW50b3J5bGFtYmRh"

    endpoint_s1 = "aHR0cDovL2xvY2FsaG9zdDo5MDAw" # localhost
    endpoint_s2 = "" # localhost

    post_buffer = []
    has_ended = False
    has_started = False

    headers = {"Content-Type": "application/json"}

    def __init__(self, log_level=LoggingSeverity.INFO):
        """Initialize the analizer and publisher node.

        Args:
            log_level (LoggingSeverity): The logging severity level.
        """
        super().__init__("analize_node")
        self.get_logger().set_level(log_level)

        self.analize_sub = self.create_subscription(Empty, "analize_captures", self.analize_callback, 1)
        self.save_pub = self.create_publisher(String, "save_fetch", 100)
        self.state_srv = self.create_service(StateParams, "service_state", self.state_callback)

        self.products = Products()
        self.fetch_msg = String()

        self.fetched_data = dict.fromkeys(self.products.get_code_name(), False)

        self.get_logger().info("[+] Analize node initialized.")

    async def state_callback(self, request, response):
        self.get_logger().info(f"Service Status Received{request.state}")
        if request.state:
            # Represent a start of the process
            if not self.has_ended or not self.has_started:
                self.get_logger().error("Corruption at service callback")
            if self.has_ended:
                self.get_logger().info("Process restarted")
            self.has_started = True
            self.has_ended = False

            self.fetched_data = dict.fromkeys(self.products.get_code_name(), False)
            self.post_buffer = []
        else:
            # Represent the end of the process
            self.has_ended = True
            self.get_logger().info("[!] Process finished")
            sleep(20)
            self.get_logger().info("[>] Start to fetch not found products")
            for code_name, published in self.fetched_data.items():
                if not published:
                    self.post_product(code_name, 0, 0, 0)
        response.status_code = 1
        return response

    def analize_callback(self, _empty):
        evaluations = EvaluationsDB()
        captures = evaluations.get_captures()
        fetches = evaluations.get_fetches()

        for code_name, detections in list(captures.items()):
            lon_detections = len(detections)

            if fetches.get(code_name) is not None:
                fetches[code_name] = json.loads(fetches.get(code_name))
                self.get_logger().info(f" Tokens: {fetches[code_name]['tokens']}")
                if int(fetches[code_name]["tokens"]) >= lon_detections:
                    continue

            if lon_detections == 0:
                continue

            deep_sort = Sort()
            stock = int(lon_detections > 0)

            total_detections = {}
            position_detections = {}

            for detection in detections:
                detection = json.loads(detection)
                location = np.array([[detection["x_1"], detection["y_1"], detection["x_2"], detection["y_2"], detection["conf"]]])
                response = deep_sort.update(location)
                self.get_logger().debug(f"[~] {code_name} with responses: \n{response}")
                if len(response) > 0:
                    stock = int(max(np.max(response[:, 4]), stock))
                    """
                    track_id = response[0, 4]
                    if total_detections.get(track_id) is None:
                        total_detections[track_id] = 0
                        position_detections[track_id] = []

                    total_detections[track_id] += 1
                    position_detections[track_id].append([response[0], response[1], response[2], response[3]])
                    """

            self.get_logger().info(f"total_detections: {total_detections}")
            self.get_logger().info(f"position_detections: {position_detections}")

            self.get_logger().info(f"[+] Product {code_name} with stock: {stock}")
            # stock = lon_detections

            fetch = json.dumps({"stock": stock, "tokens": lon_detections})
            self.fetch_msg.data = str(code_name) + ":" + fetch
            self.get_logger().info(f"Fetch data: {self.fetch_msg.data}")

            self.save_pub.publish(self.fetch_msg)
            detection_0 = json.loads(detections[0])
            self.post_product(code_name, stock, detection_0["aisle"], detection_0["shelf"])

    def post_product(self, code_name: str, stock: int, aisle: int, rack: int):
        evaluations = EvaluationsDB()
        previous_fetches = evaluations.get_previous_fetches(code_name, 10)
        expected_stock = 0

        self.get_logger().debug(f"[r] Previous fetches: {previous_fetches}")
        for fetch in previous_fetches:
            if fetch is not None:
                self.get_logger().debug(f"[#] Readed {type(fetch)}: {fetch}")
                if type(fetch) == type(""):
                    fetch = json.loads(fetch)
                expected_stock = max(fetch.get("stock") or 0, expected_stock)

        product_info = self.products.get_product_information(code_name)
        json_send = json.dumps({
            "PLU": product_info["plu"],
            "Product": product_info["name"],
            "Type": product_info["variety"],
            "Aisle": str(aisle),
            "Rack": str(rack),
            "AmountS": str(stock),
            "AmountF": str(expected_stock),
            "Supply": expected_stock > stock,
        })

        self.get_logger().info(f"json data: {json_send}")
        res = self.post_catcher(json_send, code_name)
        if "Ok" in res:
            self.fetched_data[code_name] = True
            self.get_logger().info(f"[+] Sending previous fetches with error.")
            for i, fetch_err in enumerate(self.post_buffer):
                if fetch_err["code_name"] != "":
                    res = self.post_catcher(fetch_err["fetch"], fetch_err["code_name"])
                    if "Ok" in res:
                        self.post_buffer[i]["code_name"] = ""
                        self.fetched_data[fetch_err["code_name"]] = True
        else:
            self.post_buffer.append({"fetch": json_send, "code_name": code_name})

    def post_catcher(self, json_send, code_name):
        try:
            response = requests.post(
                base64.b64decode(self.endpoint_s1 + self.endpoint_s2).decode("utf-8"),
                json=json_send,
                headers=self.headers,
            )
            response.raise_for_status()
            self.fetched_data[code_name] = True
            self.get_logger().info(f"[>] Succesfully request post, http code: {response.status_code}")
        except requests.exceptions.RequestException as e:
            if hasattr(e, "response"):
                if hasattr(e.response, "status_code"):
                    self.get_logger().error(f"[!] HTTP Error {e.response.status_code} received")
                    return f"Error: code {e.response.status_code} received"
            self.get_logger().error(f"[!] HTTP Error Unreachable server")
            return "Error: Unreachable server"
        return f"Ok: code {response.status_code} received"

def main():
    """Main function to initialize and spin the ROS node.

    Parses arguments for logging level and starts the node.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--log-level', type=str, default='INFO', help='Logging level (DEBUG, INFO, WARN, ERROR)')
    args = parser.parse_args()

    log_level = LoggingSeverity.INFO
    if args.log_level.upper() == 'DEBUG':
        log_level = LoggingSeverity.DEBUG
    elif args.log_level.upper() == 'ERROR':
        log_level = LoggingSeverity.ERROR

    rclpy.init()

    node_launcher = Analize(log_level)

    try:
        rclpy.spin(node_launcher)
    except KeyboardInterrupt:
        pass
    finally:
        node_launcher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    """Entry point for the script. Executes main function
    if the script is run directly."""
    main()
