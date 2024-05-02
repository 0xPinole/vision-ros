"""Code aware of run vision from ros2 as a service."""

import json
from time import sleep

import rclpy
from camera_manager import Camera

# from ros_vision.srv import ScissorsMechanismParams, VisionParams
from example_interfaces.srv import AddTwoInts
from model_manager import Evaluation
from rclpy.node import Node
from std_msgs.msg import String
from storage_manager import Shelves


class VisionService(Node):
    """Node and service in charge of provide a procedure."""

    def __init__(self):
        """Init vision main service."""
        super().__init__("vision_service")
        self.srv = self.create_service(AddTwoInts, "start_procedure", self.procedure)

        self.communication_publisher = self.create_publisher(
            String, "aws_communication", 1
        )

        self.scissors_position_publisher = self.create_publisher(
            String, "scissors_movement", 1
        )

        self.scissors_microservo_publisher = self.create_publisher(
            String, "scissors_servo", 1
        )

        self.logger_publisher = self.create_publisher(String, "write_file", 10)

        # self.scissor_client = self.create_client(
        #    ScissorsMechanismParams, "shift_scissors_mechanism"
        # )

        # while not self.scissor_client.wait_for_service(timeout_sec=1.0):
        #    self.get_logger().info("Error: Timeout response ScissorsMechanism")

        self.shelves = Shelves()  # Prefered to be a with Shelves() as :
        self.camera = Camera()
        self.vision_model = Evaluation()
        # self.logger = Logs()

        # self.scissors_request = ScissorsMechanismParams.Request()

    def procedure(self, request, response):
        """Main loop to run all needed procesess."""
        aruco_id = request.a  # request.ArUco_id
        self.position = self.shelves.search_by_aruco_id(aruco_id)
        aisle, shelf, section = self.position
        self.distances = self.shelves.search_shelves(aisle, shelf, section)

        scissors_movement_msg = String()
        scissors_servo_msg = String()
        logger_msg = String()

        for index, distance in enumerate(self.distances):
            scissors_movement_msg.data = str(distance)
            self.scissors_position_publisher.publish(scissors_movement_msg)
            sleep(5)

            frame = self.camera.get_frame()
            reduced_boxes = self.vision_model.evaluate(frame)

            location = f"R,{aisle},{shelf},{section},{index}"
            logger_dict_save = {"location": location, "totals": reduced_boxes}

            logger_msg.data = json.dumps(logger_dict_save)
            self.logger_publisher.publish(logger_msg)

            scissors_servo_msg.data = "180"
            self.scissors_microservo_publisher.publish(scissors_servo_msg)
            sleep(2)

            frame = self.camera.get_frame()
            reduced_boxes = self.vision_model.evaluate(frame)

            location = f"L,{aisle},{shelf},{section},{index}"
            logger_dict_save = {"location": location, "totals": reduced_boxes}

            logger_msg.data = json.dumps(logger_dict_save)
            self.logger_publisher.publish(logger_msg)

            scissors_servo_msg.data = "0"
            self.scissors_microservo_publisher.publish(scissors_servo_msg)
            sleep(2)

        self.logger.save_logger()
        self.camera.release()

        scissors_movement_msg.data = "0"
        self.scissors_position_publisher.publish(scissors_movement_msg)

        communication_msg = String()
        communication_msg.data = "POST"
        self.communication_publisher.publish(communication_msg)

        response.sum = 1

        return response

        # def scissors_send_request(self, position):
        # """Service client request runner."""
        # self.scissors_request.position_required = position

        # self.future = self.scissor_client.call_async(self.scissors_request)
        # rclpy.spin_until_future_complete(self, self.future)
        # return


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
