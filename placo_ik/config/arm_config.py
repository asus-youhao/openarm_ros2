"""
arm_config.py — Single source of truth for Left / Right arm configuration.

All OpenArm IK nodes/trackers should import ARM_CONFIG from here instead of
defining their own copy, to prevent left-right inconsistencies.

Fields per arm
--------------
joint_names   : list of 7 joint name strings (URDF joint names)
base_link     : TF frame of robot base
ee_link       : TF frame of end-effector
home_joints   : [q1..q7] goal configuration for "go home" command
home_pose     : (x,y,z, qx,qy,qz,qw)  FK result at home_joints  ← verified FK
workspace     : dict {x:(lo,hi), y:(lo,hi), z:(lo,hi)}  rectangular fallback clamp
traj_topic    : JointTrajectoryController ROS2 topic (legacy homing only)
fwd_cmd_topic : ForwardPositionController ROS2 topic (hot-loop streaming)
latency_topic : Float32 IK latency topic
profile_topic : String JSON profiling topic
ee_delta_topic: PoseStamped EE delta input topic
"""
# ── ARM config ────────────────────────────────────────────────────────────────
ARM_CONFIG = {
    "left": {
        "joint_names":   [f"openarm_left_joint{i}"  for i in range(1, 8)],
        "base_link":     "world",
        "ee_link":       "openarm_left_link7",
        "home_joints":   [0.0, 0.0, 0.0, 1.5708, 0.0, 0.0, 0.0],
        "home_pose":     (0.2160, 0.1535, 0.4780, 0.7071, -0.0000, 0.7071, -0.0000),
        "workspace":     {"x": (-0.20, 0.62), "y": (0.05, 0.65), "z": (0.15, 0.8)},
        "traj_topic":    "/left_joint_trajectory_controller/joint_trajectory",
        "fwd_cmd_topic": "/left_forward_position_controller/commands",
        "latency_topic": "/left/delta_ik_latency_ms",
        "profile_topic": "/left/placo_profile",
        "ee_delta_topic": "/ee_delta/left",
    },
    "right": {
        "joint_names":   [f"openarm_right_joint{i}" for i in range(1, 8)],
        "base_link":     "world",
        "ee_link":       "openarm_right_link7",
        "home_joints":   [0.0, 0.0, 0.0, 1.5708, 0.0, 0.0, 0.0],
        "home_pose":     (0.216000, -0.153500, 0.478001,0.7071, 0.0000, 0.7071, 0.0000),
        "workspace":     {"x": (-0.20, 0.62), "y": (-0.65, -0.05), "z": (0.15, 0.8)},
        "traj_topic":    "/right_joint_trajectory_controller/joint_trajectory",
        "fwd_cmd_topic": "/right_forward_position_controller/commands",
        "latency_topic": "/right/delta_ik_latency_ms",
        "profile_topic": "/right/placo_profile",
        "ee_delta_topic": "/ee_delta/right",
    },
}