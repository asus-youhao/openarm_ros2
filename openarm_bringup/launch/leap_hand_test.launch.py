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
LEAP Hand standalone test launch file.

This launch file starts only the LEAP Hand hardware interface and controller
for testing without the full bimanual arm system.

Usage:
    # Test with real hardware
    ros2 launch openarm_bringup leap_hand_test.launch.py serial_port:=/dev/ttyUSB1

    # Test with fake hardware
    ros2 launch openarm_bringup leap_hand_test.launch.py use_fake_hardware:=true

    # Move to home position on startup (hardware initialization)
    ros2 launch openarm_bringup leap_hand_test.launch.py serial_port:=/dev/ttyUSB1 move_to_home:=true

    # Send initial command after launch (software level)
    ros2 launch openarm_bringup leap_hand_test.launch.py serial_port:=/dev/ttyUSB1 send_initial_command:=true initial_pose:=home

    # Send grasp command on startup
    ros2 launch openarm_bringup leap_hand_test.launch.py serial_port:=/dev/ttyUSB1 send_initial_command:=true initial_pose:=grasp

    # Launch with RViz visualization
    ros2 launch openarm_bringup leap_hand_test.launch.py serial_port:=/dev/ttyUSB1 use_rviz:=true
"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyUSB0",
            description="Serial port for LEAP Hand (e.g., /dev/ttyUSB0, /dev/ttyUSB1)",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "baudrate",
            default_value="4000000",
            description="Baudrate for Dynamixel communication",
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
            description="Move LEAP Hand to home position (0.0 rad) on activation.",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Launch RViz for visualization.",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_rate",
            default_value="100",
            description="Controller manager update rate in Hz.",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "send_initial_command",
            default_value="false",
            description="Send initial position command after controller startup (home, grasp, or none).",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_pose",
            default_value="home",
            description="Initial pose to send: 'home' (all zeros) or 'grasp' (closed hand).",
        )
    )

    # Initialize Arguments
    serial_port = LaunchConfiguration("serial_port")
    baudrate = LaunchConfiguration("baudrate")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    move_to_home = LaunchConfiguration("move_to_home")
    use_rviz = LaunchConfiguration("use_rviz")
    controller_rate = LaunchConfiguration("controller_rate")
    send_initial_command = LaunchConfiguration("send_initial_command")
    initial_pose = LaunchConfiguration("initial_pose")

    # Get URDF via xacro
    description_package = "openarm"
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", "leap_hand_standalone.urdf.xacro"]
            ),
            " ",
            "serial_port:=",
            serial_port,
            " ",
            "baudrate:=",
            baudrate,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "move_to_home:=",
            move_to_home,
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}

    # Controller configuration
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("openarm_bringup"),
            "config",
            "v10_controllers",
            "leap_hand_test_controllers.yaml",
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

    # LEAP Hand Controller
    leap_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["leap_hand_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay controller spawner after joint state broadcaster
    delay_leap_hand_controller_spawner_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[leap_hand_controller_spawner],
        )
    )

    # RViz (optional)
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("openarm_bringup"), "rviz", "leap_hand_test.rviz"]
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(use_rviz),
    )

    # Send initial command to controller (delayed to ensure controller is ready)
    # Home position: all joints at 0.0 rad
    send_home_command = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            '/leap_hand_controller/commands',
            'std_msgs/msg/Float64MultiArray',
            '{data: [0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0]}'
        ],
        output='screen',
    )
    
    # Grasp position: closed hand
    send_grasp_command = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            '/leap_hand_controller/commands',
            'std_msgs/msg/Float64MultiArray',
            '{data: [0.0,0.525,1.11,0.86, 0.0,0.525,1.11,0.86, 0.0,0.525,1.11,0.86, 1.46,0.56,0.71,1.36]}'
        ],
        output='screen',
    )
    
    # Delay initial command by 3 seconds to ensure controller is ready
    delayed_home_command = TimerAction(
        period=3.0,
        actions=[send_home_command],
        condition=IfCondition(send_initial_command),
    )
    
    delayed_grasp_command = TimerAction(
        period=3.0,
        actions=[send_grasp_command],
        condition=IfCondition(send_initial_command),
    )

    nodes = [
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_leap_hand_controller_spawner_after_joint_state_broadcaster,
        rviz_node,
        delayed_home_command,
    ]

    return LaunchDescription(declared_arguments + nodes)
