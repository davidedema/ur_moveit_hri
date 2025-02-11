#!/usr/bin/env python3

import os
import socket
import time
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from bt_interfaces.srv import Gripper
from std_srvs.srv import Trigger


class GripperController(Node):
    def __init__(self):
        super().__init__("gripper_controller_execution")

        # IP and port for the UR robot
        self.HOST = "192.168.1.35"
        self.PORT = 30002
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self.HOST, self.PORT))

        # Path to your scripts
        self.path = os.path.join(
            get_package_share_directory("main_simulation"), "scripts"
        )

        # Create service server called gripper_controller_cmd of type std_srvs/srv/string
        self.sub = self.create_service(Gripper, "/robot1/gripper", self.callback)

    def callback(self, request, response):
        print(f"received: {request.command}")
        script = ""
        if request.command.lower().startswith('o'):
            script = os.path.join(self.path, "open3.script")
            response.status = "Opening gripper 3"
        elif request.command.lower().startswith('c'):
            script = os.path.join(self.path, "close.script")
            response.status = "Closing gripper"
        else:
            response.status = f"Invalid argument {request.command}"
            response.success = False
            self.get_logger().info(response.status)
            return response

        try:
            with open(script, "rb") as f:
                l = f.read(2024)
                while l:
                    self.s.send(l)
                    l = f.read(2024)
                    
            time.sleep(1.0)
                    
            # Call service /io_and_status_controller/resend_robot_program sending std_srvs/srv/Trigger
            client = self.create_client(Trigger, "/robot1/io_and_status_controller/resend_robot_program")
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Service not available, waiting again...")
            request = Trigger.Request()
            future = client.call_async(request)
            
            
        except FileNotFoundError:
            self.get_logger().error(f"Script file {script} not found.")
            response.status = F"Error: script {script} not found"
            response.success = False
        except Exception as e:
            self.get_logger().error(f"Error while sending the script: {e}")
            response.status = f"Error while sending the script {e}"
            response.success = False
            
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    gripper_controller = GripperController()

    rclpy.spin(gripper_controller)

    # Clean up and shutdown
    gripper_controller.s.close()
    gripper_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
