# Copyright 2026 Enactic, Inc.
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
Launch file for OpenArm V10 Bimanual with O6 Hand end-effectors.

This launch file starts:
- Robot state publisher with V10 bimanual + O6 left & right hands URDF
- ROS2 Control hardware interface (v10_simple_hardware)
- Joint trajectory controllers for both arms
- O6 hand controllers for both hands

Example usage:
  # Bimanual with O6 hands (can0=right arm, can1=left arm, can2=right O6, can3=left O6)
  ros2 launch openarm_bringup openarm_o6_bimanual.launch.py

  # With custom CAN interfaces
  ros2 launch openarm_bringup openarm_o6_bimanual.launch.py \
    right_can_interface:=can0 left_can_interface:=can1 \
    right_o6_can_interface:=can2 left_o6_can_interface:=can3

  # With forward position controller mode
  ros2 launch openarm_bringup openarm_o6_bimanual.launch.py \
    robot_controller:=forward_position_controller

  # With RViz visualization
  ros2 launch openarm_bringup openarm_o6_bimanual.launch.py launch_rviz:=true
"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext, conditions
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def namespace_from_context(context, arm_prefix):
    """Extract namespace from arm_prefix."""
    arm_prefix_str = context.perform_substitution(arm_prefix)
    if arm_prefix_str:
        return arm_prefix_str.strip('/')
    return None


def generate_robot_description(context: LaunchContext, description_package, description_file,
                               arm_type, use_fake_hardware, right_can_interface, left_can_interface,
                               right_o6_can_interface, left_o6_can_interface):
    """Generate robot description with O6 hands using xacro processing."""

    # Substitute launch configuration values
    description_package_str = context.perform_substitution(description_package)
    description_file_str = context.perform_substitution(description_file)
    arm_type_str = context.perform_substitution(arm_type)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    right_can_interface_str = context.perform_substitution(right_can_interface)
    left_can_interface_str = context.perform_substitution(left_can_interface)
    right_o6_can_interface_str = context.perform_substitution(right_o6_can_interface)
    left_o6_can_interface_str = context.perform_substitution(left_o6_can_interface)

    # Build xacro file path
    xacro_path = os.path.join(
        get_package_share_directory(description_package_str),
        "urdf", "robot", description_file_str
    )

    # Process xacro with bimanual O6 hand configuration
    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            "arm_type": arm_type_str,
            "bimanual": "true",
            "hand": "true",  # Enable hand/gripper
            "use_fake_hardware": use_fake_hardware_str,
            "ros2_control": "true",
            "right_can_interface": right_can_interface_str,
            "left_can_interface": left_can_interface_str,
            "ee_type": "o6",  # Bimanual with O6 hands (use 'o6' not 'o6_hand')
            "right_o6_can_interface": right_o6_can_interface_str,
            "left_o6_can_interface": left_o6_can_interface_str,
        }
    ).toprettyxml(indent="  ")

    return robot_description


def robot_nodes_spawner(context: LaunchContext, description_package, description_file,
                        arm_type, use_fake_hardware, right_can_interface, left_can_interface,
                        right_o6_can_interface, left_o6_can_interface, arm_prefix):
    """Spawn robot state publisher and control nodes with O6 hands."""

    namespace = namespace_from_context(context, arm_prefix)

    # Generate robot description
    robot_description = generate_robot_description(
        context, description_package, description_file, arm_type, use_fake_hardware,
        right_can_interface, left_can_interface, right_o6_can_interface, left_o6_can_interface
    )

    # Build controllers file path (fixed path)
    controllers_file_path = os.path.join(
        get_package_share_directory("openarm_bringup"),
        "config", "v10_controllers",
        "openarm_v10_o6_bimanual_controllers.yaml"
    )
    robot_description_param = {"robot_description": robot_description}

    # Robot state publisher node
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        namespace=namespace,
        parameters=[robot_description_param],
    )

    # Control node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        namespace=namespace,
        parameters=[robot_description_param, controllers_file_path],
    )

    return [robot_state_pub_node, control_node]


def controller_spawner(context: LaunchContext, robot_controller, arm_prefix):
    """Spawn arm controllers based on robot_controller argument."""
    namespace = namespace_from_context(context, arm_prefix)
    controller_manager_ref = f"/{namespace}/controller_manager" if namespace else "/controller_manager"

    robot_controller_str = context.perform_substitution(robot_controller)

    # Select controller names based on mode
    if robot_controller_str == "forward_position_controller":
        robot_controller_left = "left_forward_position_controller"
        robot_controller_right = "right_forward_position_controller"
    elif robot_controller_str == "joint_trajectory_controller":
        robot_controller_left = "left_joint_trajectory_controller"
        robot_controller_right = "right_joint_trajectory_controller"
    else:
        raise ValueError(f"Unknown robot_controller: {robot_controller_str}")

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[robot_controller_left, robot_controller_right, "-c", controller_manager_ref],
    )

    return [robot_controller_spawner]


def o6_hand_controller_spawner(context: LaunchContext, robot_controller, arm_prefix):
    """Spawn O6 hand controllers for both hands."""
    namespace = namespace_from_context(context, arm_prefix)
    controller_manager_ref = f"/{namespace}/controller_manager" if namespace else "/controller_manager"

    robot_controller_str = context.perform_substitution(robot_controller)

    # Select O6 hand controller names based on mode
    if robot_controller_str == "forward_position_controller":
        left_hand_controller = "left_o6_hand_forward_position_controller"
        right_hand_controller = "right_o6_hand_forward_position_controller"
    elif robot_controller_str == "joint_trajectory_controller":
        left_hand_controller = "left_o6_hand_controller"
        right_hand_controller = "right_o6_hand_controller"
    else:
        raise ValueError(f"Unknown robot_controller: {robot_controller_str}")

    o6_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[left_hand_controller, right_hand_controller, "-c", controller_manager_ref],
    )

    return [o6_controller_spawner]


def generate_launch_description():
    """Generate launch description for OpenArm V10 Bimanual with O6 hands configuration."""

    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="openarm_description",
            description="Description package with robot URDF/xacro files. Use external package at /home/asus/openArm_leapHand_urdf/src/openarm_description",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="v10.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        ),
        DeclareLaunchArgument(
            "arm_type",
            default_value="v10",
            description="Type of arm (e.g., v10).",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use fake hardware instead of real hardware.",
        ),
        DeclareLaunchArgument(
            "robot_controller",
            default_value="joint_trajectory_controller",
            choices=["forward_position_controller", "joint_trajectory_controller"],
            description="Robot controller to start (affects both arm and O6 hand controllers).",
        ),
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="openarm_bringup",
            description="Package with the controller's configuration in config folder.",
        ),
        DeclareLaunchArgument(
            "arm_prefix",
            default_value="",
            description="Prefix for the arm for topic namespacing.",
        ),
        DeclareLaunchArgument(
            "right_can_interface",
            default_value="can2",
            description="CAN interface for the right arm (e.g., can0).",
        ),
        DeclareLaunchArgument(
            "left_can_interface",
            default_value="can3",
            description="CAN interface for the left arm (e.g., can1).",
        ),
        DeclareLaunchArgument(
            "right_o6_can_interface",
            default_value="can0",
            description="CAN interface for right O6 hand (default: can2).",
        ),
        DeclareLaunchArgument(
            "left_o6_can_interface",
            default_value="can1",
            description="CAN interface for left O6 hand (default: can3).",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Start RViz automatically with this launch file.",
        ),
    ]

    # Initialize launch configurations
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    arm_prefix = LaunchConfiguration("arm_prefix")
    launch_rviz = LaunchConfiguration("launch_rviz")

    # Spawn robot nodes (state publisher + control node)
    robot_nodes = OpaqueFunction(
        function=robot_nodes_spawner,
        args=[
            LaunchConfiguration("description_package"),
            LaunchConfiguration("description_file"),
            LaunchConfiguration("arm_type"),
            LaunchConfiguration("use_fake_hardware"),
            LaunchConfiguration("right_can_interface"),
            LaunchConfiguration("left_can_interface"),
            LaunchConfiguration("right_o6_can_interface"),
            LaunchConfiguration("left_o6_can_interface"),
            arm_prefix,
        ],
    )

    # Spawn joint state broadcaster
    joint_state_broadcaster_spawner = OpaqueFunction(
        function=lambda context: [Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace_from_context(context, arm_prefix),
            arguments=["joint_state_broadcaster",
                       "--controller-manager",
                       f"/{namespace_from_context(context, arm_prefix)}/controller_manager" if namespace_from_context(context, arm_prefix) else "/controller_manager"],
        )]
    )

    # Spawn arm trajectory controllers
    arm_controller_spawner_func = OpaqueFunction(
        function=controller_spawner,
        args=[LaunchConfiguration("robot_controller"), arm_prefix]
    )

    # Spawn O6 hand controllers
    o6_hand_controller_spawner_func = OpaqueFunction(
        function=o6_hand_controller_spawner,
        args=[LaunchConfiguration("robot_controller"), arm_prefix]
    )

    # Optional RViz node
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(LaunchConfiguration("description_package")), "rviz", "bimanual.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=conditions.IfCondition(launch_rviz),
    )

    # Timing and sequencing
    LAUNCH_DELAY_SECONDS = 2.0
    
    delayed_joint_state_broadcaster = TimerAction(
        period=LAUNCH_DELAY_SECONDS,
        actions=[joint_state_broadcaster_spawner],
    )

    delayed_arm_controller = TimerAction(
        period=LAUNCH_DELAY_SECONDS,
        actions=[arm_controller_spawner_func],
    )

    delayed_o6_hand_controller = TimerAction(
        period=LAUNCH_DELAY_SECONDS,
        actions=[o6_hand_controller_spawner_func],
    )

    nodes = [
        *declared_arguments,
        robot_nodes,
        delayed_joint_state_broadcaster,
        delayed_arm_controller,
        delayed_o6_hand_controller,
        rviz_node,
    ]

    return LaunchDescription(nodes)
