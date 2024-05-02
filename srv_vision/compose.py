"""Composition of all nodes."""

import rclpy
from rclpy.executors import SingleThreadedExecutor

from srv_vision.communication_node import DataManager
from srv_vision.vision_service import VisionService


def main(args=None):
    """Main run for all nodes & services."""
    rclpy.init(args=args)
    try:
        data_manager = DataManager()
        vision_service = VisionService()

        executor = SingleThreadedExecutor()
        executor.add_node(vision_service)
        executor.add_node(data_manager)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            data_manager.destroy_node()
            vision_service.destroy_node()

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    """Main."""
    main()
