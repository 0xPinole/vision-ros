"""This main node creates a service who publish requested movements over the full procedure on ."""

import rclpy
from rclpy.node import Node

from time import sleep

from vision_interfaces.srv import VisionParams
from vision_interfaces.srv import Procedure

from geometry_msgs.msg import Empty
from geometry_msgs.msg import Pose2D

from srv_vision.storage_manager import Shelves


class VisionService(Node):
    """Node and service in charge of provide a procedure."""
    aruco_id = -1

    def __init__(self):
        """Init vision main service.
        This service will provide the appropriate rutine for move the
        scissors mechanism and evaluate the product from each side of
        the aisle. This node uses the aruco code as primary key on db
        and extract the correspondance aisle and shelf.

        This node has the responsability to hold the response until the
        scissors procedure has completed the rutine, it uses a time sleep
        from the tracked move requesting.
        """
        super().__init__("vision_service_node")
        self.vision_service = self.create_service(VisionParams, "vision_service", self.service_callback)

        self.scissors_pub = self.create_publisher(Pose2D, "scissors_movement", 1)
        self.communication_pub = self.create_publisher(Empty, "analize_captures", 1)

        self.process_srv = self.create_client(Procedure, "procedure_init")
        while not self.process_srv.wait_for_service(timeout_sec=1.0):
            pass

        self.shelves_data = Shelves()
        self.scissors_msg = Pose2D()
        self.process_msg = Procedure.request()
        self.empty_msg = Empty()


    async def service_callback(self, request, response):
        """Main loop to run all needed procesess."""
        shelf_map = self.shelves.get_shelf(request.aruco_id)
        self.scissors_msg.theta = 0
        self.scissors_msg.x = 1000

        for side in ["left", "right"]:
            if shelf_map.get(side) is not None:
                self.scissors_msg.theta = int(side == "right") * 180
                self.scissors_pub.publish(self.scissors_msg)

                self.process_msg.capture = True
                self.process_msg.aisle = shelf_map[side].get("aisle")
                self.process_msg.shelf = shelf_map[side].get("shelf")
                self.process_srv.call_async(self.process_msg)

                if self.scissors_msg.y == 0:
                    route = range(10, 51, 10)
                else:
                    route = range(50, 1, -10)

                for step in route:
                    self.scissors_msg.y = step # from 10 to 50 cm or viceversa
                    self.scissors_pub.publish(self.scissors_msg)
                    sleep(5) # sleep 5 sec to stabilize

                self.process_msg.capture = False
                self.process_srv.call_async(self.process_msg)

        self.scissors_msg.theta = 0
        self.scissors_msg.y = 0
        self.scissors_pub.publish(self.scissors_msg)

        self.communication_pub.publish(self.empty_msg)
        sleep(10)

        response.status_code = 1
        return response

def main():
    """
    This function initializes the ROS 2 node and keeps it spinning to handle callbacks.
    It ensures the node is properly shut down when the program exits.
    """
    rclpy.init()

    node_launcher = VisionService()

    rclpy.spin(node_launcher)

    node_launcher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    """
    Entry point for the script. If the script is run directly,
    the main() function will be executed.
    """
    main()
