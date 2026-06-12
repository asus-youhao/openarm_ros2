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
MoveIt Move Group Only Launch File - For O6 Hands

This launch file ONLY starts the move_group node for motion planning.
It assumes hardware and controllers are already running.

IMPORTANT: This version uses NO-GRIPPER configuration for O6 Hands.
- Uses openarm_bimanual_no_gripper.srdf (no gripper groups)
- Uses moveit_controllers_no_gripper.yaml (no gripper controllers)

Before running this, make sure hardware is started:
  ros2 launch openarm_bringup openarm_o6_bimanual.launch.py

Usage:
  ros2 launch openarm_bimanual_moveit_config move_group_only.launch.py
"""

import os
import yaml
import subprocess
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_file(package_name, file_path):
    """Load a file and return its content."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    print(f"Loading file: {absolute_file_path}")
    try:
        with open(absolute_file_path, 'r') as file:
            content = file.read()
            # Verify it's the NO-GRIPPER version
            if 'gripper' in file_path.lower() and 'no_gripper' in file_path.lower():
                has_gripper = 'left_gripper' in content or 'right_gripper' in content
                print(f"   File loaded, has_gripper_groups: {has_gripper}")
            return content
    except Exception as e:
        print(f"   Failed to load: {e}")
        return None


def load_yaml(package_name, file_path):
    """Load a YAML file and return its content."""
    import yaml
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        return None


def get_robot_description_from_param(timeout_sec=3.0):
    """Fetch robot_description from robot_state_publisher parameter."""
    print("Fetching robot_description from /robot_state_publisher parameter...")
    
    try:
        # Use ros2 param get to fetch robot_description
        cmd = ['ros2', 'param', 'get', '/robot_state_publisher', 'robot_description']
        
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout_sec
        )
        
        if result.returncode == 0:
            # Parse output: "String value is: <urdf content>"
            output = result.stdout.strip()
            
            # Find "String value is: " and extract everything after it
            prefix = "String value is: "
            if prefix in output:
                robot_description = output[output.find(prefix) + len(prefix):].strip()
                
                if len(robot_description) > 1000:  # Sanity check (URDF should be large)
                    print(f"   Received robot_description ({len(robot_description)} characters)")
                    return robot_description
                else:
                    print(f"   robot_description too short ({len(robot_description)} chars), likely invalid")
                    return None
            else:
                print(f"   Unexpected output format from ros2 param get")
                return None
        
        print(f"   Failed to get robot_description (returncode: {result.returncode})")
        if result.stderr:
            stderr_preview = result.stderr[:300].replace('\n', ' ')
            print(f"   Error: {stderr_preview}")
        print(f"   Make sure hardware launch (openarm_o6_bimanual.launch.py) is running!")
        return None
            
    except subprocess.TimeoutExpired:
        print(f"   Timeout: No robot_description received after {timeout_sec}s")
        print(f"   Make sure hardware launch (openarm_o6_bimanual.launch.py) is running!")
        return None
    except Exception as e:
        print(f"   Error fetching robot_description: {e}")
        return None


def generate_launch_description():
    # Launch arguments
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )
    
    launch_rviz = LaunchConfiguration('launch_rviz')
    
    print("=" * 60)
    print("MoveIt Move Group Launch (NO-GRIPPER for O6 Hands)")
    print("Planning Pipelines: OMPL (RRTConnect) + Pilz (PTP/LIN)")
    print("=" * 60)
    
    # Get robot_description from robot_state_publisher parameter
    robot_description = get_robot_description_from_param(timeout_sec=3.0)
    
    if not robot_description:
        print("FATAL: Could not get robot_description!")
        print("   Make sure hardware is running:")
        print("   ros2 launch openarm_bringup openarm_o6_bimanual.launch.py")
        raise RuntimeError("robot_description not available")
    
    # Load SRDF (NO GRIPPER version for O6 Hands)
    robot_description_semantic = load_file(
        'openarm_bimanual_moveit_config',
        'config/openarm_bimanual_no_gripper.srdf'
    )
    
    if robot_description_semantic and ('left_gripper' in robot_description_semantic or 'right_gripper' in robot_description_semantic):
        print("WARNING: SRDF contains gripper groups!")
    else:
        print("SRDF loaded successfully (no gripper groups)")
    
    # Load kinematics
    kinematics_yaml = load_yaml(
        'openarm_bimanual_moveit_config',
        'config/kinematics.yaml'
    )
    
    # Load joint limits (NO GRIPPER version)
    joint_limits_yaml = load_yaml(
        'openarm_bimanual_moveit_config',
        # 'config/joint_limits_no_gripper.yaml'
        'config/joint_limits.yaml'
    )

    # Load Pilz Cartesian limits (max translation/rotation velocity & acceleration)
    # Required by pilz_industrial_motion_planner for PTP and LIN motion types.
    pilz_cartesian_limits = load_yaml(
        'openarm_bimanual_moveit_config',
        'config/pilz_cartesian_limits.yaml'
    ) or {}
    
    # Load MoveIt controllers (NO GRIPPER version)
    moveit_controllers = load_yaml(
        'openarm_bimanual_moveit_config',
        'config/moveit_controllers_no_gripper.yaml'
    )
    
    if moveit_controllers:
        controller_names = moveit_controllers.get('moveit_simple_controller_manager', {}).get('controller_names', [])
        print(f"Controller names: {controller_names}")
        has_gripper_ctrl = any('gripper' in name for name in controller_names)
        if has_gripper_ctrl:
            print("WARNING: Controllers contain gripper!")
        else:
            print("Controllers loaded successfully (no gripper)")
    
    print("=" * 60)
    
    # ---------------------------------------------------------------------------
    # Pilz Industrial Motion Planner
    # PTP: deterministic joint-space point-to-point,  ~1-5 ms planning
    # LIN: straight-line Cartesian path,              ~1-5 ms planning
    # WARNING: No random-sampling obstacle avoidance — only use in clear workspace.
    # Use --planner pilz_ptp or pilz_lin in csv_waypoint_runner.py.
    # ---------------------------------------------------------------------------
    pilz_planning_yaml = {
        'planning_plugin': 'pilz_industrial_motion_planner/CommandPlanner',
        'request_adapters': '',
        'default_planner_config': 'PTP',
    }

    # OMPL Planning configuration
    # NOTE: request_adapters MUST be inside ompl_planning_yaml (i.e., under the 'ompl' namespace).
    # Putting them under 'move_group' (as a separate dict) is WRONG — MoveIt2 reads adapters from
    # /move_group/ompl/request_adapters, not /move_group/move_group/request_adapters.
    # Missing AddTimeOptimalParameterization causes all trajectory timestamps to be 0.0.
    ompl_planning_yaml = {
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'request_adapters': ('default_planner_request_adapters/AddTimeOptimalParameterization '
                             'default_planner_request_adapters/ResolveConstraintFrames '
                             'default_planner_request_adapters/FixWorkspaceBounds '
                             'default_planner_request_adapters/FixStartStateBounds '
                             'default_planner_request_adapters/FixStartStatePathConstraints'),
        'start_state_max_bounds_error': 0.1,
        'planner_configs': {
            'RRTConnect': {
                'type': 'geometric::RRTConnect',
                'range': 0.0,  # 0.0 = unlimited
            },
            'RRT': {
                'type': 'geometric::RRT',
                'range': 0.0,
                'goal_bias': 0.05,
            },
            'TRRT': {
                'type': 'geometric::TRRT',
                'range': 0.0,
                'goal_bias': 0.05,
            },
        },
        'left_arm': {
            'planner_configs': ['RRTConnect', 'RRT', 'TRRT'],
            'projection_evaluator': 'joints(openarm_left_joint1,openarm_left_joint2)',
            'longest_valid_segment_fraction': 0.01,
        },
        'right_arm': {
            'planner_configs': ['RRTConnect', 'RRT', 'TRRT'],
            'projection_evaluator': 'joints(openarm_right_joint1,openarm_right_joint2)',
            'longest_valid_segment_fraction': 0.01,
        },
    }
    
    # MoveIt parameters
    moveit_params = {
        # robot_description fetched from /robot_description topic (published by hardware)
        'robot_description': robot_description,
        'robot_description_semantic': robot_description_semantic,
        'robot_description_kinematics': kinematics_yaml,
        # Merge Pilz cartesian limits into robot_description_planning so Pilz
        # PTP/LIN can read max_trans_vel / max_rot_vel from the same param namespace.
        'robot_description_planning': {**(joint_limits_yaml or {}), **pilz_cartesian_limits},
        'planning_pipelines': ['ompl', 'pilz_industrial_motion_planner'],
        'ompl': ompl_planning_yaml,
        'pilz_industrial_motion_planner': pilz_planning_yaml,
        'moveit_controller_manager': moveit_controllers.get('moveit_controller_manager', ''),
        'moveit_simple_controller_manager': moveit_controllers.get('moveit_simple_controller_manager', {}),
        'use_sim_time': False,
        'publish_robot_description_semantic': True,
    }
    
    # MoveIt move_group node  
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_params],
    )
    
    # RViz node (optional)
    rviz_config_file = os.path.join(
        get_package_share_directory('openarm_bimanual_moveit_config'),
        'config',
        'moveit.rviz'
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_params,
            {"use_sim_time": False},
        ],
        condition=IfCondition(launch_rviz)
    )

    return LaunchDescription([
        launch_rviz_arg,
        run_move_group_node,
        rviz_node
    ])

