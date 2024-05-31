"""Composition of all nodes."""

import rclpy
from rclpy.executors import SingleThreadedExecutor

from srv_vision.analize_node import Analize
from srv_vision.data_node import Data
from srv_vision.evaluation_node import Evaluation
from srv_vision.process_node import Procedure
from srv_vision.vision_service import VisionService


def main(args=None):
    """
    This function initializes all the ROS 2 nodes and keeps it spinning to handle callbacks.
    It ensures the node is properly shutdown when the program exits.
    """
    rclpy.init(args=args)
    try:
        analize_node = Analize()
        data_node = Data()
        evaluation_node = Evaluation()
        procedure_node = Procedure()
        service_node = VisionService()

        executor = SingleThreadedExecutor()
        executor.add_node(analize_node)
        executor.add_node(data_node)
        executor.add_node(evaluation_node)
        executor.add_node(procedure_node)
        executor.add_node(service_node)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            analize_node.destroy_node()
            data_node.destroy_node()
            evaluation_node.destroy_node()
            procedure_node.destroy_node()
            service_node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    """
    Entry point for the script. If the script is run directly,
    the main() function will be executed.
    """
    main()
