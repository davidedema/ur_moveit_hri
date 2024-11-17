# Copyright (c) 2021 PickNik, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#
# Author: Denis Stogl

import os

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution


def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    ur5_type = LaunchConfiguration("ur5_type")
    ur3_type = LaunchConfiguration("ur3_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    # General arguments
    description_package = LaunchConfiguration("description_package")
    description_file_ur3 = LaunchConfiguration("description_file_ur3")
    description_file_ur5 = LaunchConfiguration("description_file_ur5")
    _publish_robot_description_semantic = LaunchConfiguration("publish_robot_description_semantic")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_joint_limits_file = LaunchConfiguration("moveit_joint_limits_file")
    ur5_moveit_config_file = LaunchConfiguration("ur5_moveit_config_file")
    ur3_moveit_config_file = LaunchConfiguration("ur3_moveit_config_file")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    prefix_ur5 = LaunchConfiguration("ur5_prefix")
    prefix_ur3 = LaunchConfiguration("ur3_prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_servo = LaunchConfiguration("launch_servo")

    ur3_joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur3_type, "joint_limits.yaml"]
    )
    ur3_kinematics_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur3_type, "default_kinematics.yaml"]
    )
    ur3_physical_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur3_type, "physical_parameters.yaml"]
    )
    ur3_visual_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur3_type, "visual_parameters.yaml"]
    )
    
    ur5_joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur5_type, "joint_limits.yaml"]
    )
    ur5_kinematics_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur5_type, "default_kinematics.yaml"]
    )
    ur5_physical_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur5_type, "physical_parameters.yaml"]
    )
    ur5_visual_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur5_type, "visual_parameters.yaml"]
    )

    ur5_robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file_ur5]),
            " ",
            "robot_ip:=192.168.0.100",
            " ",
            "joint_limit_params:=",
            ur5_joint_limit_params,
            " ",
            "kinematics_params:=",
            ur5_kinematics_params,
            " ",
            "physical_params:=",
            ur5_physical_params,
            " ",
            "visual_params:=",
            ur5_visual_params,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            "ur5",
            " ",
            "ur_type:=",
            ur5_type,
            " ",
            "script_filename:=ros_control.urscript",
            " ",
            "input_recipe_filename:=rtde_input_recipe.txt",
            " ",
            "output_recipe_filename:=rtde_output_recipe.txt",
            " ",
            "tf_prefix:=",
            prefix_ur5,
            " ",
        ]
    )
    
    ur5_robot_description = {"robot_description": ur5_robot_description_content}
    
    
    ur3_robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file_ur3]),
            " ",
            "robot_ip:=192.168.0.100",
            " ",
            "joint_limit_params:=",
            ur3_joint_limit_params,
            " ",
            "kinematics_params:=",
            ur3_kinematics_params,
            " ",
            "physical_params:=",
            ur3_physical_params,
            " ",
            "visual_params:=",
            ur3_visual_params,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            "ur3",
            " ",
            "ur_type:=",
            ur3_type,
            " ",
            "script_filename:=ros_control.urscript",
            " ",
            "input_recipe_filename:=rtde_input_recipe.txt",
            " ",
            "output_recipe_filename:=rtde_output_recipe.txt",
            " ",
            "tf_prefix:=",
            prefix_ur3,
            " ",
        ]
    )
    
    ur3_robot_description = {"robot_description": ur3_robot_description_content}
    

    # MoveIt Configuration
    ur5_robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "srdf", ur5_moveit_config_file]
            ),
            " ",
            "name:=",
            # Also ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
            "ur5",
            " ",
            "prefix:=",
            prefix_ur5,
            " ",
        ]
    )
    ur5_robot_description_semantic = {"robot_description_semantic": ur5_robot_description_semantic_content}
    
    ur3_robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "srdf", ur3_moveit_config_file]
            ),
            " ",
            "name:=",
            # Also ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
            "ur3",
            " ",
            "prefix:=",
            prefix_ur3,
            " ",
        ]
    )
    ur3_robot_description_semantic = {"robot_description_semantic": ur3_robot_description_semantic_content}

    publish_robot_description_semantic = {
        "publish_robot_description_semantic": _publish_robot_description_semantic
    }

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "kinematics.yaml"]
    )

    robot_description_planning = {
        "robot_description_planning": load_yaml(
            str(moveit_config_package.perform(context)),
            os.path.join("config", str(moveit_joint_limits_file.perform(context))),
        )
    }

    # Planning Configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("main_simulation", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Configuration
    ur5_controllers_yaml = load_yaml("main_simulation", "config/controllers_ur5.yaml")
    ur3_controllers_yaml = load_yaml("main_simulation", "config/controllers_ur3.yaml")
    # the scaled_joint_trajectory_controller does not work on fake hardware
    ur5_change_controllers = context.perform_substitution(use_fake_hardware)
    if ur5_change_controllers == "true":
        ur5_controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
        ur5_controllers_yaml["joint_trajectory_controller"]["default"] = True

    ur5_moveit_controllers = {
        "moveit_simple_controller_manager": ur5_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    
    ur3_change_controllers = context.perform_substitution(use_fake_hardware)
    if ur3_change_controllers == "true":
        ur3_controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
        ur3_controllers_yaml["joint_trajectory_controller"]["default"] = True

    ur3_moveit_controllers = {
        "moveit_simple_controller_manager": ur3_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    # Start the actual move_group node/action server
    ur5_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace="robot1",
        output="screen",
        parameters=[
            ur5_robot_description,
            ur5_robot_description_semantic,
            publish_robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            ur5_moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
            warehouse_ros_config,
        ],
    )
    
    ur3_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace="robot2",
        output="screen",
        parameters=[
            ur3_robot_description,
            ur3_robot_description_semantic,
            publish_robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            ur3_moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
            warehouse_ros_config,
        ],
    )

    # rviz with moveit configuration
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "rviz", "view_robot_moveit.rviz"]
    )
    
    collisions_node = Node(package="main_simulation",
             executable="collision_loader_node",
             name="collision_loader_node",
             output="screen"
             )
        
    world_1_node = Node(package="tf2_ros",
             executable="static_transform_publisher",
             name="static_transform_publisher",
             output="screen",
             arguments=["0.0", "0.0", "1.79",
                        "3.14", "0.0", "3.14",
                        "world", "world1"])
    
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            ur5_robot_description,
            ur5_robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            robot_description_planning,
            warehouse_ros_config,
        ],
    )

    # Servo node for realtime control
    ur5_servo_yaml = load_yaml("main_simulation", "config/ur5_servo.yaml")
    ur5_servo_params = {"moveit_servo": ur5_servo_yaml}
    ur5_servo_node = Node(
        package="moveit_servo",
        condition=IfCondition(launch_servo),
        namespace="robot1",
        executable="servo_node_main",
        parameters=[
            ur5_servo_params,
            ur5_robot_description,
            ur5_robot_description_semantic,
        ],
        output="screen",
    )
    # servo_yaml = load_yaml("main_simulation", "config/ur_servo.yaml")
    # servo_params = {"moveit_servo": servo_yaml}
    # servo_node = Node(
    #     package="moveit_servo",
    #     condition=IfCondition(launch_servo),
    #     executable="servo_node_main",
    #     parameters=[
    #         servo_params,
    #         robot_description,
    #         robot_description_semantic,
    #     ],
    #     output="screen",
    # )

    # nodes_to_start = [ur5_move_group_node, ur3_move_group_node, rviz_node, servo_node, collisions_node, world_1_node]
    nodes_to_start = [ur5_move_group_node, ur3_move_group_node, ur5_servo_node, rviz_node, world_1_node]

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur5_type",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur3_type",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Indicate whether robot is running with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="main_simulation",
            description="Description package with robot URDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file_ur5",
            default_value="ur5.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file_ur3",
            default_value="ur3.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_robot_description_semantic",
            default_value="True",
            description="Whether to publish the SRDF description on topic /robot_description_semantic.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="main_simulation",
            description="MoveIt config package with robot SRDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom moveit config.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur5_moveit_config_file",
            default_value="ur5.srdf.xacro",
            description="MoveIt SRDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur3_moveit_config_file",
            default_value="ur3.srdf.xacro",
            description="MoveIt SRDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_joint_limits_file",
            default_value="joint_limits.yaml",
            description="MoveIt joint limits that augment or override the values from the URDF robot_description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
            description="Path where the warehouse database should be stored",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Make MoveIt to use simulation time. This is needed for the trajectory planing in simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur5_prefix",
            default_value='""',
            description="Prefix of the joint names, useful for "
            "multi-robot setup. If changed than also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur3_prefix",
            default_value='""',
            description="Prefix of the joint names, useful for "
            "multi-robot setup. If changed than also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_servo", default_value="true", description="Launch Servo?")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
