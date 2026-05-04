# MoveIt + O6 Hand 问题解决方案

## 问题描述

**问题 1（已修复）：** 运行 `move_group.launch.py` 时出现 O6 Hand 关节未找到错误：
```
[ERROR] Joint 'R_thumb_cmc_pitch' not found in model 'openarm'
```

**问题 2（最终修复）：** 使用 O6 Hand 硬件时，MoveIt 查找简单 gripper 关节：
```
[ERROR] Joint 'openarm_right_finger_joint1' not found in model 'openarm'
[ERROR] Joint 'openarm_left_finger_joint1' not found in model 'openarm'
```

## 根本原因

MoveIt 配置（SRDF）定义了 `left_gripper` 和 `right_gripper` groups，期望找到：
- `openarm_left_finger_joint1`（简单gripper）
- `openarm_right_finger_joint1`（简单gripper）

但当使用 `openarm_o6_bimanual.launch.py` 时，URDF 包含的是 O6 Hand 关节：
- `R_thumb_cmc_pitch`, `R_index_mcp_pitch` 等（12个手指关节）

**结果：** 配置不匹配，MoveIt 找不到 gripper 关节。

## 解决方案

采用**分离架构**：

```
Hardware 层 (ros2_control)
├── V10 Arms (7 DOF × 2)
├── O6 Hands (6 DOF × 2)
└── Controllers: arms + O6 hands

Planning 层 (MoveIt)
├── 只规划 Arm 运动
└── 不控制 O6 Hand
```

## 实施的修复

### 1. 创建无 Gripper 的 SRDF

新文件：[`openarm_bimanual_no_gripper.srdf`](../openarm_bimanual_moveit_config/config/openarm_bimanual_no_gripper.srdf)

**功能：**
- 只定义 `left_arm` 和 `right_arm` groups（各 7 DOF）
- **移除** `left_gripper` 和 `right_gripper` groups
- 保留所有碰撞检测配置

**关键变化：**
```xml
<!-- REMOVED: -->
<!-- <group name="left_gripper"> -->
<!--   <joint name="openarm_left_finger_joint1"/> -->
<!-- </group> -->
```

### 2. 创建无 Gripper 的 MoveIt Controllers

新文件：[`moveit_controllers_no_gripper.yaml`](../openarm_bimanual_moveit_config/config/moveit_controllers_no_gripper.yaml)

**功能：**
- 只定义 arm trajectory controllers
- **移除** gripper controllers

**关键变化：**
```yaml
moveit_simple_controller_manager:
  controller_names:
    - left_joint_trajectory_controller
    - right_joint_trajectory_controller
  # NO GRIPPER CONTROLLERS
```

### 3. 完全重写 `move_group_only.launch.py`

修改：[`move_group_only.launch.py`](../openarm_bimanual_moveit_config/launch/move_group_only.launch.py)

**变更：**
- 手动加载配置文件（不使用 MoveItConfigsBuilder）
- 使用 `openarm_bimanual_no_gripper.srdf`
- 使用 `moveit_controllers_no_gripper.yaml`
- 不加载 robot_description（使用 hardware 层的）

**关键代码：**
```python
# Load SRDF (NO GRIPPER version)
robot_description_semantic = load_file(
    'openarm_bimanual_moveit_config',
    'config/openarm_bimanual_no_gripper.srdf'
)

# Load MoveIt controllers (NO GRIPPER version)
moveit_controllers = load_yaml(
    'openarm_bimanual_moveit_config',
    'config/moveit_controllers_no_gripper.yaml'
)
```

### 4. 重新构建工作空间

```bash
cd ~/ros2_ws_yh
colcon build --packages-select openarm_bimanual_moveit_config --symlink-install
source install/setup.bash
```

## 正确使用方法

### 启动系统

```bash
cd ~/ros2_ws_yh/src/openarm_ros2/scripts
./start_moveit_o6_no_rviz.sh
```

### 控制手臂

```python
from pymoveit2 import MoveIt2

# MoveIt 规划手臂运动
moveit2.move_to_pose(position=Point(x=0.3, y=0.2, z=0.5))
```

### 控制 O6 Hand

```bash
# 使用 action client（不通过 MoveIt）
ros2 action send_goal /right_hand_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "..."
```

## 为什么不能让 MoveIt 控制 O6 Hand？

**可以，但需要大量工作：**

1. 修改 `openarm_bimanual.srdf`，添加手指 groups：
   ```xml
   <group name="left_hand">
     <joint name="L_thumb_cmc_pitch"/>
     <joint name="L_thumb_cmc_yaw"/>
     <joint name="L_index_mcp_pitch"/>
     ...
   </group>
   ```

2. 配置手指的 kinematics solver（可能需要自定义）

3. 添加数百个 collision pairs（手指之间）
config/openarm_bimanual_no_gripper.srdf` - SRDF without gripper groups
- `openarm_bimanual_moveit_config/config/moveit_controllers_no_gripper.yaml` - Controllers without grippers
- `openarm_bimanual_moveit_config/launch/move_group_only.launch.py` - Planning-only launch
- `scripts/MOVEIT_O6_ARCHITECTURE.md` - Architecture documentation
- `scripts/MOVEIT_O6_GUIDE.md` - Quick guide
- `scripts/MOVEIT_O6_FIX_SUMMARY.md` - This document

**修改：**
- `scripts/start_moveit_o6_no_rviz.sh` - Updated to use new launch file

**删除：**
- ~~`openarm_bimanual_moveit_config/launch/demo_o6.launch.py`~~
- ~~`scripts/start_moveit_o6_demo.sh`~~

---

**日期：** 2026-04-14  
**状态：** ✅ 完全修复  
**架构：** Hardware 层（Arms + O6） + MoveIt Planning 层（仅 Arms）6 Hand
right_hand_controller        active  # O6 Hand

$ ros2 node list
/controller_manager
/robot_state_publisher
/move_group                  # MoveIt planning
...
```

## 文件变更总结

**新增：**
- `openarm_bimanual_moveit_config/launch/move_group_only.launch.py`
- `scripts/MOVEIT_O6_ARCHITECTURE.md`
- `scripts/MOVEIT_O6_GUIDE.md`  
- `scripts/MOVEIT_O6_FIX_SUMMARY.md`

**修改：**
- `scripts/start_moveit_o6_no_rviz.sh`

**删除：**
- ~~`openarm_bimanual_moveit_config/launch/demo_o6.launch.py`~~
- ~~`scripts/start_moveit_o6_demo.sh`~~

---

**日期：** 2026-04-14  
**状态：** ✅ 已修复  
**架构：** Hardware 层 + MoveIt Planning 层分离
