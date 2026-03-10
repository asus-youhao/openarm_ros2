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
O6 Hand (LinkerHand) standalone hardware test launch file.

This launch file starts the O6 Hand hardware interface and controller(s)
for testing without the full bimanual arm system. Supports single hand or bimanual testing.

Usage:
    # Test single right hand with real hardware (CAN2)
    ros2 launch openarm_bringup o6_hand_hardware_test.launch.py can_interface:=can2 hand_type:=right

    # Test single left hand with real hardware (CAN3)
    ros2 launch openarm_bringup o6_hand_hardware_test.launch.py can_interface:=can3 hand_type:=left

    # Test both hands (bimanual mode)
    ros2 launch openarm_bringup o6_hand_hardware_test.launch.py bimanual:=true

    # Test with fake hardware
    ros2 launch openarm_bringup o6_hand_hardware_test.launch.py use_fake_hardware:=true

    # Use joint trajectory controller (action server, default)
    ros2 launch openarm_bringup o6_hand_hardware_test.launch.py robot_controller:=joint_trajectory_controller

    # Use forward position controller (topic-based)
    ros2 launch openarm_bringup o6_hand_hardware_test.launch.py robot_controller:=forward_position_controller

    # Bimanual with joint trajectory controller
    ros2 launch openarm_bringup o6_hand_hardware_test.launch.py bimanual:=true robot_controller:=joint_trajectory_controller

    # Move to home position on startup (hardware initialization)
    ros2 launch openarm_bringup o6_hand_hardware_test.launch.py can_interface:=can2 move_to_home:=true

    # Send initial command after launch
    ros2 launch openarm_bringup o6_hand_hardware_test.launch.py can_interface:=can2 send_initial_command:=true initial_pose:=home

    # Launch with RViz visualization
    ros2 launch openarm_bringup o6_hand_hardware_test.launch.py use_rviz:=true
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    """Setup function to evaluate launch configurations."""
    
    # Get launch arguments
    bimanual = LaunchConfiguration("bimanual").perform(context)
    robot_controller = LaunchConfiguration("robot_controller").perform(context)
    can_interface = LaunchConfiguration("can_interface")
    hand_type = LaunchConfiguration("hand_type")
    hand_prefix = LaunchConfiguration("hand_prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    move_to_home = LaunchConfiguration("move_to_home")
    init_speed = LaunchConfiguration("init_speed")
    init_torque = LaunchConfiguration("init_torque")
    use_rviz = LaunchConfiguration("use_rviz")
    controller_rate = LaunchConfiguration("controller_rate")
    send_initial_command = LaunchConfiguration("send_initial_command")
    initial_pose = LaunchConfiguration("initial_pose")
    
    nodes_to_start = []
    
    # Determine URDF file based on bimanual mode
    openarm_package = "openarm"
    if bimanual == "true":
        # Bimanual URDF with both hands
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
    else:
        # Single hand URDF
        robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare(openarm_package), "urdf", "o6_hand_standalone.urdf.xacro"]
                ),
                " ",
                "can_interface:=",
                can_interface,
                " ",
                "hand_type:=",
                hand_type,
                " ",
                "hand_prefix:=",
                hand_prefix,
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

    # Controller configuration - choose based on robot_controller and hand type
    hand_type_str = LaunchConfiguration("hand_type").perform(context)
    
    if bimanual == "true":
        if robot_controller == "joint_trajectory_controller":
            controller_config_file = "o6_bimanual_action_controllers.yaml"
        elif robot_controller == "forward_position_controller":
            controller_config_file = "o6_bimanual_forward_controllers.yaml"
        else:
            raise ValueError(f"Unknown robot_controller: {robot_controller}")
    else:
        # Single hand - check if left or right
        if hand_type_str == "left":
            if robot_controller == "joint_trajectory_controller":
                controller_config_file = "o6_left_hand_action_controllers.yaml"
            elif robot_controller == "forward_position_controller":
                controller_config_file = "o6_left_hand_forward_controllers.yaml"
            else:
                raise ValueError(f"Unknown robot_controller: {robot_controller}")
        else:  # right hand (default)
            if robot_controller == "joint_trajectory_controller":
                controller_config_file = "o6_hand_action_controllers.yaml"
            elif robot_controller == "forward_position_controller":
                controller_config_file = "o6_hand_forward_controllers.yaml"
            else:
                raise ValueError(f"Unknown robot_controller: {robot_controller}")
    
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
    nodes_to_start.append(robot_state_publisher_node)

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
    nodes_to_start.append(control_node)

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    nodes_to_start.append(joint_state_broadcaster_spawner)

    if robot_controller == "joint_trajectory_controller":
        # Use joint trajectory controller mode (FollowJointTrajectory)
        if bimanual == "true":
            # Spawn both hand controllers
            right_controller_spawner = Node(
                package="controller_manager",
                executable="spawner",
                arguments=["right_hand_controller", "--controller-manager", "/controller_manager"],
            )
            
            left_controller_spawner = Node(
                package="controller_manager",
                executable="spawner",
                arguments=["left_hand_controller", "--controller-manager", "/controller_manager"],
            )
            
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
            nodes_to_start.extend([delay_right_controller, delay_left_controller])
        else:
            # Single hand controller
            hand_controller_spawner = Node(
                package="controller_manager",
                executable="spawner",
                arguments=["o6_hand_controller", "--controller-manager", "/controller_manager"],
            )
            delay_hand_controller = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[hand_controller_spawner],
                )
            )
            nodes_to_start.append(delay_hand_controller)
    elif robot_controller == "forward_position_controller":
        # Use forward position controller mode
        if bimanual == "true":
            right_controller_spawner = Node(
                package="controller_manager",
                executable="spawner",
                arguments=["right_hand_controller", "--controller-manager", "/controller_manager"],
            )
            
            left_controller_spawner = Node(
                package="controller_manager",
                executable="spawner",
                arguments=["left_hand_controller", "--controller-manager", "/controller_manager"],
            )
            
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
            nodes_to_start.extend([delay_right_controller, delay_left_controller])
        else:
            o6_hand_controller_spawner = Node(
                package="controller_manager",
                executable="spawner",
                arguments=["o6_hand_controller", "--controller-manager", "/controller_manager"],
            )
            delay_o6_hand_controller_spawner = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[o6_hand_controller_spawner],
                )
            )
            nodes_to_start.append(delay_o6_hand_controller_spawner)
    else:
        raise ValueError(f"Unknown robot_controller: {robot_controller}")

    # RViz (optional)
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("openarm_bringup"), "rviz", "o6_hand_test.rviz"]
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(use_rviz),
    )
    nodes_to_start.append(rviz_node)

    # Initial commands (only for forward position controller mode)
    if robot_controller == "forward_position_controller":
        if bimanual == "true":
            # Send commands to both hands
            send_right_home = ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once',
                    '/right_hand_controller/commands',
                    'std_msgs/msg/Float64MultiArray',
                    '{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'
                ],
                output='screen',
            )
            
            send_left_home = ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once',
                    '/left_hand_controller/commands',
                    'std_msgs/msg/Float64MultiArray',
                    '{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'
                ],
                output='screen',
            )
            
            delayed_home_command = TimerAction(
                period=3.0,
                actions=[send_right_home, send_left_home],
                condition=IfCondition(send_initial_command),
            )
            nodes_to_start.append(delayed_home_command)
        else:
            send_home_command = ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once',
                    '/o6_hand_controller/commands',
                    'std_msgs/msg/Float64MultiArray',
                    '{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'
                ],
                output='screen',
            )
            
            delayed_home_command = TimerAction(
                period=3.0,
                actions=[send_home_command],
                condition=IfCondition(send_initial_command),
            )
            nodes_to_start.append(delayed_home_command)

    return nodes_to_start

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "bimanual",
            default_value="false",
            description="Launch both hands (right and left) for bimanual testing.",
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
            "can_interface",
            default_value="can0",
            description="CAN interface for O6 Hand (can2 for right, can3 for left). Only used in single hand mode.",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "hand_type",
            default_value="right",
            description="Hand type: 'right' or 'left'. Only used in single hand mode.",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "hand_prefix",
            default_value="R_",
            description="Joint name prefix (R_ for right, L_ for left). Only used in single hand mode.",
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
            description="Move O6 Hand to home position on activation.",
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
            description="Send initial position command after controller startup.",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_pose",
            default_value="home",
            description="Initial pose to send: 'home' (all zeros) or 'grasp' (closed hand).",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
