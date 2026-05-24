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
Launch file for OpenArm V10 Bimanual with O6 Hands — LPF Command Filter variant.

Identical to openarm_o6_bimanual.launch.py except the hardware plugin is
OpenArm_v10LPF_HW instead of OpenArm_v10HW.  This adds a Low-Pass Filter on
the arm (and O6 hand) POSITION COMMANDS coming from the JointTrajectoryController,
preventing jerk from discrete trajectory waypoints from reaching the motors.

State-side LPF (100 Hz) is inherited from the base class and remains active.

Extra launch argument:
  cmd_filter_cutoff_hz  — command LPF cutoff frequency in Hz (default: 10.0)
                          Typical values:
                            10 Hz  — strong smoothing (VLA / neural-network policy)
                            20 Hz  — moderate smoothing (teleoperation)
                            50 Hz  — nearly transparent (standard trajectory tracking)

Example usage:
  # Same as openarm_o6_bimanual.launch.py but with LPF
  ros2 launch openarm_bringup openarm_o6_bimanual_lpf.launch.py \\
    right_can_interface:=can2 left_can_interface:=can3 \\
    right_o6_can_interface:=can0 left_o6_can_interface:=can1 \\
    robot_controller:=joint_trajectory_controller

  # Adjust command filter strength
  ros2 launch openarm_bringup openarm_o6_bimanual_lpf.launch.py \\
    right_can_interface:=can2 left_can_interface:=can3 \\
    right_o6_can_interface:=can0 left_o6_can_interface:=can1 \\
    robot_controller:=joint_trajectory_controller \\
    cmd_filter_cutoff_hz:=20.0

  # Disable LPF (pass through — equivalent to base plugin)
  ros2 launch openarm_bringup openarm_o6_bimanual_lpf.launch.py \\
    cmd_filter_cutoff_hz:=500.0
"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext, conditions
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# ---------------------------------------------------------------------------
# LPF plugin name (replaces the hardcoded name from the xacro)
# ---------------------------------------------------------------------------
_ORIGINAL_PLUGIN = "openarm_hardware/OpenArm_v10HW"
_LPF_PLUGIN      = "openarm_hardware/OpenArm_v10LPF_HW"


def namespace_from_context(context, arm_prefix):
    """Extract namespace string from arm_prefix launch configuration."""
    arm_prefix_str = context.perform_substitution(arm_prefix)
    if arm_prefix_str:
        return arm_prefix_str.strip("/")
    return None


def _inject_lpf_plugin(xml_str: str, cmd_filter_cutoff_hz: str) -> str:
    """
    Post-process the xacro-generated URDF string:
      1. Replace every occurrence of OpenArm_v10HW with OpenArm_v10LPF_HW.
      2. Inject  <param name="cmd_filter_cutoff_hz">…</param>
         immediately after each replaced <plugin> tag so the hardware
         interface receives the tunable cutoff frequency.

    The O6HandHardware plugin tag is left untouched.
    """
    lpf_plugin_tag = f"<plugin>{_LPF_PLUGIN}</plugin>"
    inject_param    = (
        f'<param name="cmd_filter_cutoff_hz">{cmd_filter_cutoff_hz}</param>'
    )
    # Replace plugin name and inject param on the following line
    xml_str = xml_str.replace(
        f"<plugin>{_ORIGINAL_PLUGIN}</plugin>",
        f"{lpf_plugin_tag}\n            {inject_param}",
    )
    return xml_str


def generate_robot_description(
    context: LaunchContext,
    description_package,
    description_file,
    arm_type,
    use_fake_hardware,
    right_can_interface,
    left_can_interface,
    right_o6_can_interface,
    left_o6_can_interface,
    cmd_filter_cutoff_hz,
):
    """Generate URDF with LPF hardware plugin injected."""

    description_package_str    = context.perform_substitution(description_package)
    description_file_str       = context.perform_substitution(description_file)
    arm_type_str               = context.perform_substitution(arm_type)
    use_fake_hardware_str      = context.perform_substitution(use_fake_hardware)
    right_can_interface_str    = context.perform_substitution(right_can_interface)
    left_can_interface_str     = context.perform_substitution(left_can_interface)
    right_o6_can_interface_str = context.perform_substitution(right_o6_can_interface)
    left_o6_can_interface_str  = context.perform_substitution(left_o6_can_interface)
    cmd_filter_cutoff_hz_str   = context.perform_substitution(cmd_filter_cutoff_hz)

    xacro_path = os.path.join(
        get_package_share_directory(description_package_str),
        "urdf", "robot", description_file_str,
    )

    # Generate the standard bimanual + O6 URDF (plugin still says OpenArm_v10HW)
    robot_description_xml = xacro.process_file(
        xacro_path,
        mappings={
            "arm_type":               arm_type_str,
            "bimanual":               "true",
            "hand":                   "true",
            "use_fake_hardware":      use_fake_hardware_str,
            "ros2_control":           "true",
            "right_can_interface":    right_can_interface_str,
            "left_can_interface":     left_can_interface_str,
            "ee_type":                "o6",
            "right_o6_can_interface": right_o6_can_interface_str,
            "left_o6_can_interface":  left_o6_can_interface_str,
        },
    ).toprettyxml(indent="  ")

    # --- Patch: swap plugin name and inject cmd_filter_cutoff_hz param ---
    # Only applied when NOT using fake hardware (fake hardware keeps GenericSystem)
    if use_fake_hardware_str.lower() != "true":
        robot_description_xml = _inject_lpf_plugin(
            robot_description_xml, cmd_filter_cutoff_hz_str
        )

    return robot_description_xml


def robot_nodes_spawner(
    context: LaunchContext,
    description_package,
    description_file,
    arm_type,
    use_fake_hardware,
    right_can_interface,
    left_can_interface,
    right_o6_can_interface,
    left_o6_can_interface,
    arm_prefix,
    cmd_filter_cutoff_hz,
):
    """Spawn robot state publisher and ros2_control node."""

    namespace = namespace_from_context(context, arm_prefix)

    robot_description = generate_robot_description(
        context,
        description_package, description_file,
        arm_type, use_fake_hardware,
        right_can_interface, left_can_interface,
        right_o6_can_interface, left_o6_can_interface,
        cmd_filter_cutoff_hz,
    )

    controllers_file_path = os.path.join(
        get_package_share_directory("openarm_bringup"),
        "config", "v10_controllers",
        "openarm_v10_o6_bimanual_controllers.yaml",
    )
    robot_description_param = {"robot_description": robot_description}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        namespace=namespace,
        parameters=[robot_description_param],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        namespace=namespace,
        parameters=[robot_description_param, controllers_file_path],
    )

    return [robot_state_pub_node, control_node]


def controller_spawner(context: LaunchContext, robot_controller, arm_prefix):
    """Spawn arm trajectory / forward controllers for both arms."""

    namespace = namespace_from_context(context, arm_prefix)
    controller_manager_ref = (
        f"/{namespace}/controller_manager" if namespace else "/controller_manager"
    )

    robot_controller_str = context.perform_substitution(robot_controller)

    if robot_controller_str == "forward_position_controller":
        left_ctrl  = "left_forward_position_controller"
        right_ctrl = "right_forward_position_controller"
    elif robot_controller_str == "joint_trajectory_controller":
        left_ctrl  = "left_joint_trajectory_controller"
        right_ctrl = "right_joint_trajectory_controller"
    else:
        raise ValueError(f"Unknown robot_controller: {robot_controller_str}")

    return [Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[left_ctrl, right_ctrl, "-c", controller_manager_ref],
    )]


def o6_hand_controller_spawner(context: LaunchContext, robot_controller, arm_prefix):
    """Spawn O6 hand controllers for both hands."""

    namespace = namespace_from_context(context, arm_prefix)
    controller_manager_ref = (
        f"/{namespace}/controller_manager" if namespace else "/controller_manager"
    )

    robot_controller_str = context.perform_substitution(robot_controller)

    if robot_controller_str == "forward_position_controller":
        left_hand_ctrl  = "left_hand_forward_position_controller"
        right_hand_ctrl = "right_hand_forward_position_controller"
    elif robot_controller_str == "joint_trajectory_controller":
        left_hand_ctrl  = "left_hand_controller"
        right_hand_ctrl = "right_hand_controller"
    else:
        raise ValueError(f"Unknown robot_controller: {robot_controller_str}")

    return [Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[left_hand_ctrl, right_hand_ctrl, "-c", controller_manager_ref],
    )]


def generate_launch_description():
    """Generate launch description — bimanual O6, LPF command filter."""

    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="openarm_description",
            description="Package containing the robot URDF/xacro files.",
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
            description="Controller type for both arms and O6 hands.",
        ),
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="openarm_bringup",
            description="Package with controller configuration in config folder.",
        ),
        DeclareLaunchArgument(
            "arm_prefix",
            default_value="",
            description="Prefix for topic namespacing.",
        ),
        DeclareLaunchArgument(
            "right_can_interface",
            default_value="can2",
            description="CAN interface for the right arm.",
        ),
        DeclareLaunchArgument(
            "left_can_interface",
            default_value="can3",
            description="CAN interface for the left arm.",
        ),
        DeclareLaunchArgument(
            "right_o6_can_interface",
            default_value="can0",
            description="CAN interface for the right O6 hand.",
        ),
        DeclareLaunchArgument(
            "left_o6_can_interface",
            default_value="can1",
            description="CAN interface for the left O6 hand.",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Start RViz automatically with this launch file.",
        ),
        # ---- LPF-specific argument ----
        DeclareLaunchArgument(
            "cmd_filter_cutoff_hz",
            default_value="10.0",
            description=(
                "Command LPF cutoff frequency (Hz).  "
                "Lower = smoother but more phase-lag.  "
                "Recommended: 10 Hz (VLA/NN policy), 20 Hz (teleoperation), "
                "50 Hz (standard trajectory tracking)."
            ),
        ),
    ]

    arm_prefix    = LaunchConfiguration("arm_prefix")
    launch_rviz   = LaunchConfiguration("launch_rviz")

    # Robot nodes: state publisher + ros2_control node
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
            LaunchConfiguration("cmd_filter_cutoff_hz"),
        ],
    )

    # Joint state broadcaster
    joint_state_broadcaster_spawner = OpaqueFunction(
        function=lambda context: [Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace_from_context(context, arm_prefix),
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                (
                    f"/{namespace_from_context(context, arm_prefix)}/controller_manager"
                    if namespace_from_context(context, arm_prefix)
                    else "/controller_manager"
                ),
            ],
        )]
    )

    # Arm controllers
    arm_controller_spawner_func = OpaqueFunction(
        function=controller_spawner,
        args=[LaunchConfiguration("robot_controller"), arm_prefix],
    )

    # O6 hand controllers
    o6_hand_controller_spawner_func = OpaqueFunction(
        function=o6_hand_controller_spawner,
        args=[LaunchConfiguration("robot_controller"), arm_prefix],
    )

    # Optional RViz
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

    # Delay controller spawners until ros2_control_node is ready
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
