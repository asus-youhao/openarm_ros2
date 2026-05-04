# MoveIt + O6 Hand 快速指南

## ⚠️ 重要限制

**MoveIt 不能直接控制 O6 Hand 的手指！**

MoveIt 只规划和执行：
- ✅ Arm 运动（7 DOF × 2）  
- ❌ O6 Hand 手指（6 DOF × 2）

详细架构说明 → [MOVEIT_O6_ARCHITECTURE.md](MOVEIT_O6_ARCHITECTURE.md)

---

## 🚀 启动系统

```bash
cd ~/ros2_ws_yh/src/openarm_ros2/scripts
./start_moveit_o6_no_rviz.sh
```

这会启动：
1. Hardware（Arms + O6 Hands）  
2. MoveIt Planning（仅 Arms）

---

## 📡 CAN 接口

```
右臂: can2  |  左臂: can3
右手: can0  |  左手: can1
```

---

## 控制示例

### 1. 控制手臂（MoveIt）

```python
from pymoveit2 import MoveIt2
from geometry_msgs.msg import Point

# MoveIt 规划手臂运动
moveit2.move_to_pose(
    position=Point(x=0.3, y=0.2, z=0.5),
    quat_xyzw=[0, 0, 0, 1]
)
moveit2.wait_until_executed()
```

### 2. 控制 O6 Hand（Action Client）

```bash
# 右手抓取
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

---

## 🧪 验证

```bash
# 检查控制器
ros2 control list_controllers

# 应该看到：
# - left_joint_trajectory_controller ✅
# - right_joint_trajectory_controller ✅  
# - left_hand_controller ✅
# - right_hand_controller ✅
```

---

## 🔧 故障排查

### 错误：Joint 'R_thumb_cmc_pitch' not found

**原因：** 直接运行了 `move_group.launch.py`（它不知道 O6 Hand）

**解决：** 使用 `start_moveit_o6_no_rviz.sh` 或手动启动：
1. `openarm_o6_bimanual.launch.py`（先）  
2. `move_group_only.launch.py`（后）

---

## 相关文件

- [MOVEIT_O6_ARCHITECTURE.md](MOVEIT_O6_ARCHITECTURE.md) - 完整架构说明  
- [start_moveit_o6_no_rviz.sh](start_moveit_o6_no_rviz.sh) - 启动脚本  
- [move_group_only.launch.py](../openarm_bimanual_moveit_config/launch/move_group_only.launch.py) - MoveIt planning node

---

**更新:** 2026-04-14  
**状态:** ✅ 可用（分离架构）
