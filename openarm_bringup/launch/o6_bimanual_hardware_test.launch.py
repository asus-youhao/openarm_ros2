#!/usr/bin/env python3
# Copyright 2025 Enactic, Inc.
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

"""
O6 Hands Bimanual hardware test launch file.

Simplified launch file specifically for testing both O6 hands simultaneously.

Usage:
    # Test both hands with real hardware (can0=right, can1=left) using joint trajectory controller
    ros2 launch openarm_bringup o6_bimanual_hardware_test.launch.py

    # Test with different CAN interfaces
    ros2 launch openarm_bringup o6_bimanual_hardware_test.launch.py right_can:=can0 left_can:=can1

    # Test with joint trajectory controller (action server, default)
    ros2 launch openarm_bringup o6_bimanual_hardware_test.launch.py robot_controller:=joint_trajectory_controller

    # Test with forward position controller (topic-based)
    ros2 launch openarm_bringup o6_bimanual_hardware_test.launch.py robot_controller:=forward_position_controller

    # Test with fake hardware
    ros2 launch openarm_bringup o6_bimanual_hardware_test.launch.py use_fake_hardware:=true

    # Launch with RViz
    ros2 launch openarm_bringup o6_bimanual_hardware_test.launch.py use_rviz:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    """Setup function to evaluate launch configurations."""
    
    # Get launch arguments
    right_can = LaunchConfiguration("right_can")
    left_can = LaunchConfiguration("left_can")
    robot_controller = LaunchConfiguration("robot_controller").perform(context)
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    move_to_home = LaunchConfiguration("move_to_home")
    init_speed = LaunchConfiguration("init_speed")
    init_torque = LaunchConfiguration("init_torque")
    use_rviz = LaunchConfiguration("use_rviz")

    # Get URDF via xacro
    openarm_package = "openarm"
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(openarm_package), "urdf", "o6_bimanual_standalone.urdf.xacro"]
            ),
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "move_to_home:=",
            move_to_home,
            " ",
            "init_speed:=",
            init_speed,
            " ",
            "init_torque:=",
            init_torque,
        ]
    )
    
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Controller configuration - choose based on robot_controller
    if robot_controller == "joint_trajectory_controller":
        controller_config_file = "o6_bimanual_action_controllers.yaml"
        
    elif robot_controller == "forward_position_controller":
        controller_config_file = "o6_bimanual_forward_controllers.yaml"
    else:
        raise ValueError(f"Unknown robot_controller: {robot_controller}")
    if robot_controller == "forward_position_controller":
    elif robot_controller == "joint_trajectory_controller":
        left_hand_controller = "left_hand_forward_position_controller"
        right_hand_controller = "right_hand_forward_position_controller"
        left_hand_controller = "left_hand_controller"
        right_hand_controller = "right_hand_controller"
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("openarm_bringup"),
            "config",
            "v10_controllers",
            controller_config_file,
        ]
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Controller Manager
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Right Hand Controller
    right_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[right_hand_controller, "--controller-manager", "/controller_manager"],
    )
    
    # Left Hand Controller
    left_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[left_hand_controller, "--controller-manager", "/controller_manager"],
    )

    # Delay controller spawning after joint state broadcaster
    delay_right_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[right_controller_spawner],
        )
    )
    
    delay_left_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=right_controller_spawner,
            on_exit=[left_controller_spawner],
        )
    )

    # RViz (optional)
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("openarm_bringup"), "rviz", "bimanual.rviz"]
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(use_rviz),
    )

    nodes_to_start = [
        robot_state_publisher_node,
        control_node,
        joint_state_broadcaster_spawner,
        delay_right_controller,
        delay_left_controller,
        rviz_node,
    ]

    return nodes_to_start


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_can",
            default_value="can0",
            description="CAN interface for right O6 Hand (default: can2).",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_can",
            default_value="can1",
            description="CAN interface for left O6 Hand (default: can3).",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="joint_trajectory_controller",
            choices=["forward_position_controller", "joint_trajectory_controller"],
            description="Robot controller to start.",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "move_to_home",
            default_value="false",
            description="Move O6 Hands to home position on activation.",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "init_speed",
            default_value="150",
            description="Initial speed for O6 Hand joints (0-250, default: 150).",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "init_torque",
            default_value="150",
            description="Initial torque for O6 Hand joints (0-250, default: 150).",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Launch RViz for visualization.",
        )
    )

    # Get launch arguments
    right_can = LaunchConfiguration("right_can")
    left_can = LaunchConfiguration("left_can")
    robot_controller = LaunchConfiguration("robot_controller")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    move_to_home = LaunchConfiguration("move_to_home")
    init_speed = LaunchConfiguration("init_speed")
    init_torque = LaunchConfiguration("init_torque")
    use_rviz = LaunchConfiguration("use_rviz")

    # OpaqueFunction to setup launch with evaluated configurations
    opaque_function = OpaqueFunction(function=launch_setup)

    return LaunchDescription(declared_arguments + [opaque_function])

