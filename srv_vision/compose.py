"""Composition of all nodes."""

import rclpy
from rclpy.executors import SingleThreadedExecutor

#from srv_vision.communication_node import DataManager
#from srv_vision.vision_service import VisionService
from srv_vision.data_saver_node import DataSaver
from srv_vision.model_manager import Evaluation


def main(args=None):
    """Main run for all nodes & services."""
    rclpy.init(args=args)
    try:
        data_service = DataSaver()
        evaluation_node = Evaluation()

        executor = SingleThreadedExecutor()
        executor.add_node(data_service)
        executor.add_node(evaluation_node)

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
