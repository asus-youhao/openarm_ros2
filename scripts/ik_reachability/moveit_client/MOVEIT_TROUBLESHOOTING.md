# MoveIt 没有移动问题 - 故障排查指南

## 🔍 问题诊断

### 问题：运行 `moveit_simple_example.py` 时，第二步没有移动到 xyz 位置

---

## ❌ 根本原因

### 1. **使用了仿真模式（假硬件）**

查看 `demo.launch.py` 第 193 行：
```python
DeclareLaunchArgument("use_fake_hardware", default_value="true"),  # ⚠️ 默认仿真！
```

**当前启动命令：**
```bash
ros2 launch openarm_bimanual_moveit_config demo.launch.py \
  right_can_interface:=can0 \
  left_can_interface:=can1
```

问题：虽然设置了 CAN 接口，但 **`use_fake_hardware` 默认为 `true`**，所以：
- ❌ 不会连接真实硬件
- ❌ 不会通过 CAN 总线通信
- ❌ 不会真正移动机械臂
- ✅ 只在仿真中"假装"移动

### 2. **RViz Motion Planning 插件干扰**

当运行 `demo.launch.py` 时，会启动：
- RViz 节点
- Motion Planning 插件（交互式标记）

这个插件也会创建 MoveIt 客户端，可能与您的 Python 脚本产生冲突：
- 两个客户端同时发送目标
- 可能互相覆盖轨迹
- 导致运动失败或不符合预期

---

## ✅ 解决方案

### 🏆 推荐方案：使用真实硬件 + 无 RViz 模式

#### 步骤 1：停止当前的 demo.launch.py
```bash
# 按 Ctrl+C 停止
```

#### 步骤 2：使用专用启动脚本（无 RViz 干扰）
```bash
cd ~/ros2_ws_yh/src/openarm_ros2/scripts
./start_moveit_no_rviz.sh
```

这个脚本会：
1. ✅ 启动真实硬件接口（`use_fake_hardware:=false`）
2. ✅ 启动 MoveIt move_group
3. ❌ **不启动 RViz**（避免干扰）

#### 步骤 3：运行调试版示例
```bash
# 在新终端中
cd ~/ros2_ws_yh/src/openarm_ros2/scripts
python3 moveit_debug_example.py
```

这个脚本会显示详细的调试信息，帮助您了解每一步的执行情况。

---

### 🔧 备选方案：修改 demo.launch.py 使用真实硬件

如果您想继续使用 demo.launch.py（带 RViz 可视化）：

```bash
ros2 launch openarm_bimanual_moveit_config demo.launch.py \
  use_fake_hardware:=false \
  right_can_interface:=can0 \
  left_can_interface:=can1
```

**重点：** 必须明确设置 `use_fake_hardware:=false`！

**注意：** 使用这个方案时，RViz 的 Motion Planning 插件可能仍会干扰。建议：
- 在 RViz 中**不要**使用交互式标记拖动机械臂
- 或者关闭 Motion Planning 插件的 "Plan & Execute" 功能

---

## 🧪 测试步骤

### 1. 验证硬件连接
```bash
# 检查 ros2_control_node 是否连接到真实硬件
ros2 topic echo /joint_states --once

# 您应该看到真实的关节位置数据
```

### 2. 检查是否有冲突节点
```bash
# 列出所有节点
ros2 node list

# 检查是否有多个 MoveIt 客户端
# 如果看到 /rviz2 和您的脚本同时运行，可能会冲突
```

### 3. 监控轨迹执行
```bash
# 在另一个终端中监控
ros2 topic echo /left_joint_trajectory_controller/joint_trajectory
```

当您的脚本发送目标时，应该能看到轨迹消息。

---

## 📋 对比表格

| 启动方式 | 硬件模式 | RViz | 适合用途 | 冲突风险 |
|---------|---------|------|---------|---------|
| **demo.launch.py** (默认) | 仿真 ❌ | 有 ✅ | 学习/测试 | 高 ⚠️ |
| **demo.launch.py** (加参数) | 真实 ✅ | 有 ✅ | 可视化调试 | 中 ⚠️ |
| **start_moveit_no_rviz.sh** | 真实 ✅ | 无 ❌ | 脚本控制 ⭐ | 低 ✅ |

---

## 🔍 调试检查清单

运行 Python 脚本前，确认：

- [ ] 使用 `use_fake_hardware:=false`
- [ ] CAN 接口正确设置（can0, can1）
- [ ] 机械臂已上电并连接
- [ ] ros2_control_node 正在运行
- [ ] MoveIt move_group 正在运行
- [ ] 没有其他程序控制机械臂（如 RViz 交互式标记）
- [ ] `/joint_states` topic 有数据输出

---

## 📝 示例输出（正常情况）

运行 `moveit_debug_example.py` 时，您应该看到：

```
🚀 启动 MoveIt 示例（带调试信息）...

📋 当前运行的节点: 8 个
  ⚠️  检测到: /move_group
  ⚠️  检测到: /move_group_private_*

🔧 初始化 MoveIt2 接口...

⏳ 等待 MoveIt 初始化...
   5 秒...
   4 秒...
   ...

============================================================
📍 示例 1: 移动到 Home 姿态 (所有关节归零)
============================================================
🎯 目标关节角度: [0, 0, 0, 0, 0, 0, 0]
🚀 开始规划和执行...
⏳ 等待执行完成...
✅ Home 姿态到达！

============================================================
📍 示例 2: 移动到 XYZ 位置
============================================================
🎯 目标位置: x=0.3, y=0.2, z=0.5
🚀 开始规划和执行（笛卡尔模式）...
⏳ 等待执行完成...
✅ 目标位置到达！   <-- 这里应该成功！
```

---

## 🆘 如果还是不行

### 检查 MoveIt 日志
```bash
# 在运行 move_group 的终端中，查看是否有错误信息
# 常见错误：
# - "Unable to connect to action server"
# - "Planning failed"
# - "IK solution not found"
```

### 查看控制器状态
```bash
ros2 control list_controllers

# 确认输出类似：
# left_joint_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] active
# right_joint_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] active
```

### 手动测试控制器
```bash
# 测试控制器是否能接收命令
ros2 action send_goal /left_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: ['openarm_left_joint1', ..., 'openarm_left_joint7']
  points: [{positions: [0,0,0,0,0,0,0], time_from_start: {sec: 2}}]
" --feedback
```

---

## 📁 相关文件

| 文件 | 说明 |
|------|------|
| `start_moveit_no_rviz.sh` | 启动脚本（无 RViz 干扰）⭐ |
| `moveit_debug_example.py` | 带调试信息的示例 |
| `moveit_simple_example.py` | 原始简单示例 |
| `PYMOVEIT2_GUIDE.md` | PyMoveIt2 使用指南 |

---

**最后更新:** 2026-04-13  
**状态:** ✅ 已验证所有解决方案
