#!/usr/bin/env python3

import os
import socket
import time
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class GripperController(Node):
    def __init__(self):
        super().__init__("gripper_controller_execution")

        # IP and port for the UR robot
        self.HOST = "192.168.0.100"
        self.PORT = 30002
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self.HOST, self.PORT))

        # Path to your scripts
        self.path = os.path.join(
            get_package_share_directory("main_simulation"), "scripts"
        )

        # Subscriber to the gripper command topic
        self.subscription = self.create_subscription(
            String, "gripper_controller_cmd", self.callback, 10
        )

    def callback(self, data):
        print(f"received: {data.data}")
        script = ""
        if data.data == "open1":
            script = os.path.join(self.path, "open1.script")
        elif data.data == "open2":
            script = os.path.join(self.path, "open2.script")
        elif data.data == "open3":
            script = os.path.join(self.path, "open3.script")
        elif data.data == "close":
            script = os.path.join(self.path, "close.script")
        else:
            self.get_logger().info("Invalid argument!")
            return

        try:
            with open(script, "rb") as f:
                l = f.read(2024)
                while l:
                    self.s.send(l)
                    l = f.read(2024)
                    
            time.sleep(1.0)
                    
            # Call service /io_and_status_controller/resend_robot_program sending std_srvs/srv/Trigger
            client = self.create_client(Trigger, "/io_and_status_controller/resend_robot_program")
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Service not available, waiting again...")
            request = Trigger.Request()
            future = client.call_async(request)
            
            
        except FileNotFoundError:
            self.get_logger().error(f"Script file {script} not found.")
        except Exception as e:
            self.get_logger().error(f"Error while sending the script: {e}")


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
