#!/usr/bin/env python3
"""
Small helper script to start the tool communication interface for ROS2.
"""

# -- BEGIN LICENSE BLOCK ----------------------------------------------
# Copyright 2019 FZI Forschungszentrum Informatik
# Created on behalf of Universal Robots A/S
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# -- END LICENSE BLOCK ------------------------------------------------

import subprocess
import rclpy
from rclpy.node import Node


class ToolCommunicationNode(Node):
    def __init__(self):
        super().__init__('ur_tool_communication')

        # Declare and read parameters
        self.declare_parameter('robot_ip', '192.168.1.34')  # Default IP for illustration
        self.declare_parameter('tcp_port', 54321)  # Default port
        self.declare_parameter('device_name', '/home/davide/Desktop/dual_arm_ws/install/robotiq_2f85/share/robotiq_2f85/ttyURO') #'/home/davide/Desktop/ttyUR0')  # Default device name

        robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        tcp_port = str(self.get_parameter('tcp_port').get_parameter_value().integer_value)
        local_device = self.get_parameter('device_name').get_parameter_value().string_value

        self.get_logger().info(f"Remote device will be available at '{local_device}'")

        # Configure the socat command
        cfg_params = ["pty", f"link={local_device}", "raw", "ignoreeof", "waitslave"]
        cmd = ["socat", ",".join(cfg_params), ":".join(["tcp", robot_ip, tcp_port])]

        self.get_logger().info(f"Starting socat with the following command:\n{' '.join(cmd)}")
        try:
            subprocess.call(cmd)
        except Exception as e:
            self.get_logger().error(f"Failed to start socat: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ToolCommunicationNode()
    rclpy.spin(node)

    # Shutdown ROS2 properly
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
