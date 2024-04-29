"""Code aware of run vision from ros2 as a service."""

import rclpy
from camera_manager import Camera
from model_manager import Evaluation
from rclpy.node import Node
from std_msgs.msg import String
from storage_manager import Logs, Shelves

from ros_vision.srv import ScissorsMechanismParams, VisionParams


class VisionService(Node):
    """Node and service in charge of provide a procedure."""

    def __init__(self):
        """Init vision main service."""
        super().__init__("vision_service")
        self.srv = self.create_service(VisionParams, "start_procedure", self.procedure)

        self.communication_publisher = self.create_publisher(
            String, "aws_communication", 1
        )

        self.scissor_client = self.create_client(
            ScissorsMechanismParams, "scissors_mechanism"
        )
        while not self.scissor_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Error: Timeout response ScissorsMechanism")

        self.shelves = Shelves()  # Prefered to be a with Shelves() as :
        self.camera = Camera()
        self.vision_model = Evaluation()
        self.logger = Logs()

        self.scissors_request = ScissorsMechanismParams.Request()

    def procedure(self, request, response):
        """Main loop to run all needed procesess."""
        aruco_id = request.ArUco_id
        self.position = self.shelves.search_by_aruco_id(aruco_id)
        self.distances = self.shelves.search_shelves(*self.position)
        for index, distance in enumerate(self.distances):
            response = self.scissors_send_request(distance)
            frame = self.camera.get_frame()
            reduced_boxes = self.vision_model.evaluate(frame)
            self.logger.insert_log(
                f"{','.join(map(str, self.position))},{index}", reduced_boxes
            )
            # Logic to rotate 180 degrees

        self.logger.save_logger()
        self.camera.release()

        communication_msg = String()
        communication_msg.data = "POST"
        self.communication_publisher.publish(communication_msg)
        self.scissors_send_request(0)
        response.status_code = 2

        return response

    def scissors_send_request(self, position):
        """Service client request runner."""
        self.scissors_request.position_required = position

        self.future = self.scissor_client.call_async(self.scissors_request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    """Init ros2 and node of vision service."""
    rclpy.init()
    service = VisionService()

    rclpy.spin(service)
    service.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    """Run main fun."""
    main()
