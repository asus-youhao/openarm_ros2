# MoveIt 命令使用指南

## 📚 总览

MoveIt 提供多种命令接口，本文档解释如何使用它们来控制机械臂的 XYZ 位置。

---

## ✅ 回答您的问题

### **问：给 MoveIt IK 下命令，跟 ros2 action 一样吗？**

**答：不完全一样，但相关！MoveIt 提供多种接口：**

| 接口类型 | 用途 | 是否是 Action |
|---------|------|--------------|
| **IK Service** | 只计算关节角度 (xyz → 关节) | ❌ 否，是 **Service** |
| **MoveGroup Action** | 完整的规划+执行 | ✅ 是，是 **Action** |
| **Python API** | 高层抽象接口 | 🔄 底层使用 Action/Service |

---

## 🎯 三种主要使用方式

### 1️⃣ 使用 Service 计算 IK (只计算，不执行)

**用途：** 将 xyz 位置转换为关节角度

```bash
# 命令行方式
ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK "
ik_request:
  group_name: 'left_arm'
  pose_stamped:
    header:
      frame_id: 'world'
    pose:
      position: {x: 0.3, y: 0.2, z: 0.5}
      orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
"
```

**Python 方式（推荐）：**
```bash
cd ~/ros2_ws_yh/src/openarm_ros2/scripts
python3 moveit_ik_service_example.py
```

---

### 2️⃣ 使用 Action 规划并执行 (完整控制)

**用途：** 规划路径并执行运动

```bash
# 命令行方式（类似 ros2 action）
ros2 action send_goal /move_action moveit_msgs/action/MoveGroup "
request:
  group_name: 'left_arm'
  num_planning_attempts: 5
  allowed_planning_time: 5.0
  max_velocity_scaling_factor: 0.5
" --feedback
```

**Python 方式（推荐）：**
```bash
python3 moveit_action_example.py
```

---

### 3️⃣ 使用 Python API (最简单，最推荐) ⭐

**用途：** 高层接口，自动处理规划和执行

```python
from moveit.planning import MoveItPy

# 初始化
moveit = MoveItPy(node_name="moveit_node")
left_arm = moveit.get_planning_component("left_arm")

# 设置目标位置
pose_goal = PoseStamped()
pose_goal.pose.position.x = 0.3
pose_goal.pose.position.y = 0.2
pose_goal.pose.position.z = 0.5
pose_goal.pose.orientation.w = 1.0

# 规划并执行
left_arm.set_goal_state(pose_stamped_msg=pose_goal)
plan_result = left_arm.plan()
left_arm.execute(plan_result.trajectory)
```

**运行完整示例：**
```bash
python3 moveit_example_control.py
```

---

## 📊 对比表格

| 方式 | 类型 | 复杂度 | 功能 | 推荐场景 |
|------|------|--------|------|---------|
| **compute_ik** | Service | ⭐⭐ | 只计算 IK | 测试位置可达性 |
| **/move_action** | Action | ⭐⭐⭐⭐ | 规划+执行 | 底层集成 |
| **MoveItPy API** | Python | ⭐ | 完整控制 | 日常开发 ✅ |
| **命令行** | CLI | ⭐⭐⭐ | 调试测试 | 快速验证 |

---

## 🚀 快速开始示例

### 先决条件
```bash
# 启动 MoveIt
ros2 launch openarm_bimanual_moveit_config demo.launch.py \
  right_can_interface:=can0 \
  left_can_interface:=can1
```

### 示例 1: 测试 IK (xyz → 关节角度)
```bash
cd ~/ros2_ws_yh/src/openarm_ros2/scripts
python3 moveit_ik_service_example.py
```

**输出示例：**
```
✅ IK Solution found!
Joint angles:
  Joint 1: 0.1234 rad (7.07°)
  Joint 2: 0.5678 rad (32.54°)
  ...
```

### 示例 2: 使用 Action 执行运动
```bash
python3 moveit_action_example.py
```

### 示例 3: 使用 Python API (最简单)
```bash
python3 moveit_example_control.py
```

---

## 🔧 常用命令速查

### 查看可用接口
```bash
# 查看 Actions
ros2 action list | grep move

# 查看 Services
ros2 service list | grep compute

# 查看 Topics
ros2 topic list | grep planning
```

### 查看接口详情
```bash
# 查看 IK Service 接口定义
ros2 service type /compute_ik
ros2 interface show moveit_msgs/srv/GetPositionIK

# 查看 MoveGroup Action 接口定义
ros2 action type /move_action
ros2 interface show moveit_msgs/action/MoveGroup
```

---

## 💡 最佳实践建议

### ✅ 推荐做法

1. **日常开发**：使用 Python MoveItPy API
   - 简单直观
   - 文档完善
   - 社区支持好

2. **快速测试**：使用 IK Service
   ```bash
   ros2 service call /compute_ik ...
   ```

3. **调试验证**：查看命令指南
   ```bash
   ./moveit_command_guide.sh
   ```

### ❌ 避免的做法

1. ❌ 不要直接用命令行 Action 做复杂操作（太繁琐）
2. ❌ 不要跳过 IK 验证直接执行（可能超出工作空间）
3. ❌ 不要忘记设置速度限制（安全第一）

---

## 📖 相关示例文件

| 文件 | 说明 |
|------|------|
| `moveit_example_control.py` | Python API 完整示例 ⭐ |
| `moveit_action_example.py` | ROS2 Action 接口示例 |
| `moveit_ik_service_example.py` | IK Service 调用示例 |
| `moveit_command_guide.sh` | 命令行使用指南 |

---

## 🎓 学习路径

1. **入门** → 运行 `moveit_example_control.py`
2. **理解原理** → 查看 `moveit_ik_service_example.py`
3. **底层集成** → 参考 `moveit_action_example.py`
4. **命令行调试** → 阅读 `moveit_command_guide.sh`

---

## ❓ 常见问题

### Q: IK 计算失败怎么办？
A: 检查目标位置是否在工作空间内，可以先用 IK service 验证

### Q: Action 和 Service 有什么区别？
A: 
- **Service**: 一次性请求-响应（如 IK 计算）
- **Action**: 长时间运行，有进度反馈（如运动执行）

### Q: 为什么推荐 Python API？
A: 封装良好，自动处理规划、碰撞检测、执行，代码简洁

---

## 📞 参考资源

- [MoveIt 2 官方文档](https://moveit.picknik.ai/main/index.html)
- [MoveItPy 教程](https://moveit.picknik.ai/main/doc/examples/moveit_py/moveitpy_tutorial.html)
- ROS2 Action 教程：`ros2 action --help`

---

**最后更新：** 2026-04-13
**维护者：** OpenArm Team
