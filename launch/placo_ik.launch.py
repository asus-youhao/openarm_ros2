"""
placo_ik.launch.py — run the online profiler the `ros2 launch` way, WITHOUT
touching the script's own code.

It wraps `ik_node/placo_ik_online_profiler_ws_mesh.py` in an ExecuteProcess and
maps launch arguments onto the script's existing argparse flags. The script is
unchanged — this file only builds its command line.

Usage:
    ros2 launch placo_ik/launch/placo_ik.launch.py                 # arm:=both
    ros2 launch placo_ik/launch/placo_ik.launch.py arm:=right
    ros2 launch placo_ik/launch/placo_ik.launch.py arm:=right dry_run:=true home:=false
    ros2 launch placo_ik/launch/placo_ik.launch.py -s              # list all arguments

Inside the deploy container (deps baked in, OPENARM_URDF preset):
    ./deploy/shell.sh ros2 launch /work/openarm_ros2/placo_ik/launch/placo_ik.launch.py arm:=right
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

# Resolve the target script relative to THIS file, so it works regardless of
# where the repo is checked out / mounted (host path or container /work/...).
_HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.abspath(os.path.join(_HERE, "..", "ik_node",
                                      "placo_ik_online_profiler_ws_mesh.py"))

# launch-arg name  ->  script flag.  These take a value (--flag <value>).
_VALUE_FLAGS = {
    "arm": "--arm",
    "config": "--config",
    "rate": "--rate",
    "horizon": "--horizon",
    "max_iter": "--max-iter",
    "ws_mesh": "--ws-mesh",
    "csv": "--csv",
    "plot": "--plot",
    "calib_yaw": "--calib-yaw",
    "calib_rpy": "--calib-rpy",
    "wrist_vel_cap": "--wrist-vel-cap",
    "lpf_alpha": "--lpf-alpha",
    "ori_lpf_alpha": "--ori-lpf-alpha",
}

# launch-arg name (true/false)  ->  script store_true flag.
_BOOL_TRUE_FLAGS = {
    "dry_run": "--dry-run",
    "rebuild": "--rebuild",
    "verbose": "--verbose",
    "keyboard": "--keyboard",
    "use_traj": "--use-traj",
    "no_rot_tracking": "--no-rot-tracking",
    "success_gate": "--success-gate",
}


def _setup(context, *_args, **_kwargs):
    def val(name):
        return LaunchConfiguration(name).perform(context).strip()

    cmd = ["python3", SCRIPT]

    for arg_name, flag in _VALUE_FLAGS.items():
        v = val(arg_name)
        if v != "":                       # empty string => leave script default
            cmd += [flag, v]

    for arg_name, flag in _BOOL_TRUE_FLAGS.items():
        if val(arg_name).lower() == "true":
            cmd += [flag]

    # `home` is on by default in the script (--home-first). Only pass the opt-out.
    if val("home").lower() == "false":
        cmd += ["--no-home-first"]

    return [ExecuteProcess(cmd=cmd, output="screen", emulate_tty=True)]


def generate_launch_description():
    decls = [
        # value args (empty default => fall through to the script's own default)
        DeclareLaunchArgument("arm", default_value="both",
                              description="right | left | both"),
        DeclareLaunchArgument("config", default_value="",
                              description="bimanual YAML (required for arm:=both with per-arm cfg)"),
        DeclareLaunchArgument("rate", default_value="50.0",
                              description="control-loop Hz (also solver dt)"),
        DeclareLaunchArgument("horizon", default_value="",
                              description="JointTrajectory duration ms (default auto)"),
        DeclareLaunchArgument("max_iter", default_value="",
                              description="solver iteration cap"),
        DeclareLaunchArgument("ws_mesh", default_value="",
                              description="WorkspaceMesh .npz path (default auto-detect)"),
        DeclareLaunchArgument("csv", default_value="", description="output CSV path"),
        DeclareLaunchArgument("plot", default_value="", description="output plot PNG path"),
        DeclareLaunchArgument("calib_yaw", default_value="",
                              description="tracker->arm yaw offset deg"),
        DeclareLaunchArgument("calib_rpy", default_value="",
                              description="'roll,pitch,yaw' deg (overrides calib_yaw)"),
        DeclareLaunchArgument("wrist_vel_cap", default_value="",
                              description="wrist joint5-7 velocity cap rad/s"),
        DeclareLaunchArgument("lpf_alpha", default_value="",
                              description="output LPF alpha (single or 7-value csv)"),
        DeclareLaunchArgument("ori_lpf_alpha", default_value="",
                              description="orientation SLERP EMA (1.0 = off)"),
        # boolean args
        DeclareLaunchArgument("home", default_value="true",
                              description="home before start (false => --no-home-first)"),
        DeclareLaunchArgument("dry_run", default_value="false",
                              description="compute IK but do not publish"),
        DeclareLaunchArgument("rebuild", default_value="false",
                              description="rebuild RobotWrapper every step (slow, original behaviour)"),
        DeclareLaunchArgument("verbose", default_value="false", description="print every IK step"),
        DeclareLaunchArgument("keyboard", default_value="false", description="start in KEYBOARD mode"),
        DeclareLaunchArgument("use_traj", default_value="false",
                              description="use JointTrajectoryController instead of ForwardCommand"),
        DeclareLaunchArgument("no_rot_tracking", default_value="false",
                              description="position-only IK (disable orientation tracking)"),
        DeclareLaunchArgument("success_gate", default_value="false",
                              description="legacy freeze-on-IK-failure behaviour"),
    ]
    return LaunchDescription(decls + [OpaqueFunction(function=_setup)])
