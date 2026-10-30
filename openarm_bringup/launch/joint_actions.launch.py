"""
/joint_actions aggregator: standalone launch, also included by the O6 bringup.

Starts scripts/data_collection/joint_actions_aggregator.py, which merges the arm
and O6 hand FPC commands into a fixed 26-joint /joint_actions. The script is not a
ROS executable (scripts/ is outside any package), so openarm_bringup installs it
into share/openarm_bringup/scripts/data_collection (CMakeLists.txt) and it is run
with the current interpreter. That works with and without --symlink-install.

openarm_o6_bimanual.launch.py includes this file when with_joint_actions:=true
(the default); only start it by hand after a bringup with with_joint_actions:=false,
/joint_actions must have exactly one publisher. See docs/JOINT_ACTIONS.md.

Usage (after the bringup is running):
  ros2 launch openarm_bringup joint_actions.launch.py
  ros2 launch openarm_bringup joint_actions.launch.py hand_config:=none
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

SCRIPT_REL = os.path.join('scripts', 'data_collection', 'joint_actions_aggregator.py')

# Launch argument -> script flag. Empty values are not passed (script default wins).
ARG_TO_FLAG = {
    'arm_config': '--arm_config',
    'hand_config': '--hand_config',
    'publish_rate': '--publish_rate',
    'stale_timeout': '--stale_timeout',
}


def _launch_setup(context, *args, **kwargs):
    script = (LaunchConfiguration('aggregator_script').perform(context)
              or os.path.join(get_package_share_directory('openarm_bringup'), SCRIPT_REL))
    if not os.path.isfile(script):
        raise FileNotFoundError(
            f'joint_actions_aggregator script not found: {script}. '
            'Rebuild openarm_bringup or pass aggregator_script:=/abs/path')

    cmd = [sys.executable, script]
    for arg, flag in ARG_TO_FLAG.items():
        value = LaunchConfiguration(arg).perform(context)
        if value:
            cmd += [flag, value]

    return [ExecuteProcess(cmd=cmd, name='joint_actions_aggregator', output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('aggregator_script', default_value='',
                              description='Path override; empty = installed copy.'),
        DeclareLaunchArgument('arm_config', default_value='both',
                              choices=['both', 'left', 'right'],
                              description='Arms subscribed. With both, single-arm teleop '
                                          'works too: only arms that sent a command gate '
                                          'publishing; the other is filled from '
                                          '/joint_states.'),
        DeclareLaunchArgument('hand_config', default_value='o6_both',
                              choices=['none', 'o6_left', 'o6_right', 'o6_both'],
                              description='Hands subscribed; the vector is always 26 joints.'),
        DeclareLaunchArgument('publish_rate', default_value='50.0',
                              description='Seeds the arm staleness window only.'),
        DeclareLaunchArgument('stale_timeout', default_value='0.0',
                              description='Fixed staleness window (s); 0 = auto.'),
        OpaqueFunction(function=_launch_setup),
    ])
