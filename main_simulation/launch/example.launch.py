#!/usr/bin/env python3
#
# Author: Placido Falqueto placido.falqueto [at] unitn.it
# Maintainer: Enrico Saccon  enrico.saccon [at] unitn.it

import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='main_simulation',
            executable='poc_motion',
            # name='poc_motion',
            namespace='robot1', 
            output='screen',
        ),
        
    ])