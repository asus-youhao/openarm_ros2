# MoveIt + O6 Hand 架构说明

## 🚨 重要限制

**MoveIt 当前配置不支持 O6 Hand 的手指控制！**

## 为什么？

MoveIt 的配置文件（SRDF）只定义了：
- `left_arm` group: 7 DOF（手臂关节）
- `right_arm` group: 7 DOF（手臂关节）  
- `left_gripper` group: 1 DOF（简单夹爪）
- `right_gripper` group: 1 DOF（简单夹爪）

**O6 Hand 有 6 个主动关节（拇指 2 + 4 指各 1），不在 MoveIt 配置范围内。**

## 正确的系统架构

```
┌─────────────────────────────────────────────┐
│          Hardware 层 (ros2_control)         │
│  openarm_o6_bimanual.launch.py             │
│                                             │
│  • V10 Arm (7 DOF) ✅                      │
│  • O6 Hand (6 DOF) ✅                      │
│  • Controllers:                             │
│    - left_joint_trajectory_controller       │
│    - right_joint_trajectory_controller      │
│    - left_hand_controller (O6)             │
│    - right_hand_controller (O6)            │
└──────────────────┬──────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────┐
│         Planning 层 (MoveIt)                │
│  move_group_only.launch.py                 │
│                                             │
│  • 只规划 Arm 运动（7 DOF）✅              │
│  • 不控制 O6 Hand ❌                        │
│  • 使用 hardware 层的 robot_description    │
└─────────────────────────────────────────────┘
```

## 如何使用

### 启动系统

```bash
# 使用便捷脚本（推荐）
cd ~/ros2_ws_yh/src/openarm_ros2/scripts
./start_moveit_o6_no_rviz.sh
```

**或手动启动：**

```bash
# Terminal 1: 启动硬件 + 所有控制器
ros2 launch openarm_bringup openarm_o6_bimanual.launch.py \
  right_can_interface:=can2 \
  left_can_interface:=can3 \
  right_o6_can_interface:=can0 \
  left_o6_can_interface:=can1 \
  use_fake_hardware:=false \
  launch_rviz:=false

# Terminal 2: 启动 MoveIt planning（等待 5 秒后）
ros2 launch openarm_bimanual_moveit_config move_group_only.launch.py
```

### 控制手臂（通过 MoveIt）

```python
from pymoveit2 import MoveIt2
from geometry_msgs.msg import Point

# 初始化 MoveIt（只控制手臂）
moveit2 = MoveIt2(
    node=node,
    joint_names=['openarm_left_joint1', ..., 'openarm_left_joint7'],
    base_link_name="world",
    end_effector_name="openarm_left_hand",
    group_name="left_arm",
    callback_group=callback_group
)

# 规划并执行运动
moveit2.move_to_pose(position=Point(x=0.3, y=0.2, z=0.5), quat_xyzw=[0,0,0,1])
moveit2.wait_until_executed()
```

### 控制 O6 Hand（通过 Action Client）

```python
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint

# 创建 action client
client = ActionClient(
    node,
    FollowJointTrajectory,
    '/right_hand_controller/follow_joint_trajectory'
)

# 构造目标（抓取姿态）
goal = FollowJointTrajectory.Goal()
goal.trajectory.joint_names = [
    'R_thumb_cmc_pitch', 'R_thumb_cmc_yaw',
    'R_index_mcp_pitch', 'R_middle_mcp_pitch',
    'R_ring_mcp_pitch', 'R_pinky_mcp_pitch'
]

point = JointTrajectoryPoint()
point.positions = [0.7, 0.3, 0.8, 0.8, 0.8, 0.8]  # 闭合
point.time_from_start.sec = 2

goal.trajectory.points = [point]
client.send_goal(goal)
```

**或使用命令行：**

```bash
ros2 action send_goal /right_hand_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: [R_thumb_cmc_pitch, R_thumb_cmc_yaw, 
                R_index_mcp_pitch, R_middle_mcp_pitch,
                R_ring_mcp_pitch, R_pinky_mcp_pitch]
  points:
    - positions: [0.7, 0.3, 0.8, 0.8, 0.8, 0.8]
      time_from_start: {sec: 2}
"
```

## 常见错误

### ❌ 错误：Joint 'R_thumb_cmc_pitch' not found in model

**原因：** 尝试直接运行 `move_group.launch.py`，它生成了不包含 O6 Hand 的新 robot_description。

**解决：** 使用 `move_group_only.launch.py` 或 `start_moveit_o6_no_rviz.sh`。

### ❌ 错误：Segmentation fault in move_group

**原因：** Hardware 层和 MoveIt 层使用了不同的 robot_description。

**解决：** 按顺序启动：先 hardware，后 move_group_only。

## 未来改进（需要修改 SRDF）

要让 MoveIt 支持 O6 Hand，需要：

1. 修改 `openarm_bimanual.srdf`，添加手指 group：
```xml
<group name="left_hand">
  <joint name="L_thumb_cmc_pitch"/>
  <joint name="L_thumb_cmc_yaw"/>
  <joint name="L_index_mcp_pitch"/>
  <joint name="L_middle_mcp_pitch"/>
  <joint name="L_ring_mcp_pitch"/>
  <joint name="L_pinky_mcp_pitch"/>
</group>
```

2. 添加手指的 collision pairs

3. 配置手指的 kinematics solver

但这需要大量工作，当前方案（分离控制）已足够。

## 相关文件

- [`move_group_only.launch.py`](../openarm_bimanual_moveit_config/launch/move_group_only.launch.py) - MoveIt planning node（无 hardware）
- [`start_moveit_o6_no_rviz.sh`](start_moveit_o6_no_rviz.sh) - 完整启动脚本
- [`openarm_o6_bimanual.launch.py`](../openarm_bringup/launch/openarm_o6_bimanual.launch.py) - Hardware + O6 controllers
- [`openarm_bimanual.srdf`](../openarm_bimanual_moveit_config/config/openarm_bimanual.srdf) - MoveIt 配置（不含 O6）

---

**更新时间:** 2026-04-14  
**状态:** ✅ 已验证可用  
**架构:** 分离式（Hardware + MoveIt Planning）
