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
import os
import time
# import threading

import serial
import binascii

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from bt_interfaces.srv import Gripper
from std_srvs.srv import Trigger

class Robotiq2f85(Node):
    def __init__(self):
        super().__init__('Robotiq2f85')
        
        self.declare_parameter('robot_name', 'robot2')
        
        self.interface_path = os.path.join(get_package_share_directory('robotiq_2f85'), 'ttyURO')

        print("interface: ", self.interface_path)

        # Declare and read parameters
        self.declare_parameter('robot_ip', '192.168.1.34')  # Default IP for illustration
        self.declare_parameter('tcp_port', 54321)  # Default port
        self.declare_parameter('device_name', self.interface_path)  # Default device name

        robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        tcp_port = str(self.get_parameter('tcp_port').get_parameter_value().integer_value)
        local_device = self.get_parameter('device_name').get_parameter_value().string_value

        self.get_logger().info(f"Remote device will be available at '{local_device}'")

        # Configure the socat command
        cfg_params = ["pty", f"link={local_device}", "raw", "ignoreeof", "waitslave"]
        cmd = ["socat", ",".join(cfg_params), ":".join(["tcp", robot_ip, tcp_port])]

        self.get_logger().info(f"Starting socat with the following command:\n{' '.join(cmd)}")
        try:
            # threading.Thread(target=subprocess.call, args=(cmd,)).start()
            subprocess.Popen(cmd)
            time.sleep(1)
        except Exception as e:
            self.get_logger().error(f"Failed to start socat: {e}")
        
        print("Connecting to gripper")
        
        self.ser = serial.Serial(
            port=self.interface_path,  # Ensure the correct port name (add '/dev/' for Linux)
            baudrate=115200,
            timeout=1,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        
        self.init_communication()
        
        # self.srv = self.create_service(Gripper, 'robotiq2f85_service', self.gripper_callback)
        self.srv = self.create_service(Gripper, '/robot2/gripper', self.gripper_callback)
        self.rise_srv = self.create_client(Trigger, self.get_parameter('robot_name').get_parameter_value().string_value + '/rise')
        while not self.rise_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Rise service not available, waiting again...')
    
    def gripper_callback(self, request, response):
        if request.command.lower().startswith('o'):
            print("Opening gripper")
            self.open_gripper()
            response.success = True
            response.status = "Gripper opened, moving up"
            print("Gripper opened, moving up...")
            rise_request = Trigger.Request()
            rise_future = self.rise_srv.call_async(rise_request)
            time.sleep(7)
            print("Moved up")            
            return response
            
        elif request.command.lower().startswith('c'):
            print("Closing gripper")
            self.close_gripper()
            response.success = True
            response.status = "Gripper closed"
            print("Gripper closed, moving up...")
            rise_request = Trigger.Request()
            rise_future = self.rise_srv.call_async(rise_request)
            time.sleep(7)
            print("Moved up")            
            return response
            
        elif request.command.lower().startswith('a'):
            print("Initializing communication")
            self.init_communication()
            print("Communication initialized")
            response.success = True
            response.status = "Gripper connected"
            return response
            
        elif request.command.lower().startswith('s'):
            print("Getting gripper status")
            status = self.get_status()
            response.success = True
            response.status = status if not None else "Gripper status retrieved, but empty"
            return response
            
        else:
            print("Invalid command received")
            response.success = False
            response.status = "Invalid command"
            return response
                
    
        
    def init_communication(self) -> None:
        self.ser.write(b"\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30")
        data_raw = self.ser.readline()
        data = binascii.hexlify(data_raw).decode()  # Convert bytes to string
        print("Enabled gripper:", data)
        time.sleep(0.01)

    def get_status(self) -> None:
        self.ser.write(b"\x09\x03\x07\xD0\x00\x01\x85\xCF")
        data_raw = self.ser.readline()
        data = binascii.hexlify(data_raw).decode()
        print("Gripper status:", data)
        time.sleep(1)
        
    def close_gripper(self) -> str:
        self.ser.write(b"\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF\xFF\x42\x29")
        data_raw = self.ser.readline()
        data = binascii.hexlify(data_raw).decode()
        print("Gripper closed:", data)
        return data
    
    def open_gripper(self) -> None:
        self.ser.write(b"\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19")
        data_raw = self.ser.readline()
        data = binascii.hexlify(data_raw).decode()
        print("Gripper open:", data)
        

def main(args=None):
    rclpy.init(args=args)
    node = Robotiq2f85()
    rclpy.spin(node)

    # Shutdown ROS2 properly
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
