# MoveIt + O6 Hand 测试指南

## ✅ 修复完成

**问题：** MoveIt 查找简单 gripper 关节，但 O6 Hand URDF 中只有手指关节  
**解决：** 创建无 gripper 的 SRDF 和 controllers 配置

---

## 🧪 测试步骤

### 步骤 1: 确保环境加载

```bash
source ~/ros2_ws_yh/install/setup.bash
```

### 步骤 2: 清理旧进程

```bash
pkill -f "ros2_control_node|move_group"
sleep 2
```

### 步骤 3: 启动系统

```bash
cd ~/ros2_ws_yh/src/openarm_ros2/scripts
./start_moveit_o6_no_rviz.sh
```

### 步骤 4: 验证无错误

**期望：** 应该看到：
```
✅ MoveIt 启动完成（OpenArm V10 + O6 Hands）！
```

**不应该出现：**
```
❌ [ERROR] Joint 'openarm_right_finger_joint1' not found
❌ [ERROR] Joint 'R_thumb_cmc_pitch' not found
```

### 步骤 5: 检查节点

```bash
# 新终端
ros2 node list
```

**期望输出：**
```
/controller_manager
/joint_state_broadcaster
/left_hand_controller
/left_joint_trajectory_controller
/right_hand_controller
/right_joint_trajectory_controller
/robot_state_publisher
/move_group                    # ← MoveIt planning
```

### 步骤 6: 检查 MoveIt groups

```bash
ros2 param get /move_group planning_pipelines
```

应该能正常运行，无错误。

### 步骤 7: 测试规划（可选）

```bash
# 在新终端，进入 scripts 目录
cd ~/ros2_ws_yh/src/openarm_ros2/scripts
python3 moveit_simple_example.py
```

**期望：** 手臂移动到目标位置，无关节未找到错误。

---

## 🔍 故障排查

### 如果仍然看到 gripper 错误

原因：旧的环境变量  
解决：
```bash
# 完全重新加载
cd ~/ros2_ws_yh
source install/setup.bash

# 验证配置文件存在
ls -l install/openarm_bimanual_moveit_config/share/openarm_bimanual_moveit_config/config/*no_gripper*
```

### 如果 move_group 启动失败

检查 hardware 是否已启动：
```bash
ros2 node list | grep robot_state_publisher
```

如果没有，先手动启动 hardware：
```bash
ros2 launch openarm_bringup openarm_o6_bimanual.launch.py \
  use_fake_hardware:=false \
  launch_rviz:=false
```

---

## 📊 预期结果对比

| 状态 | 之前（错误） | 现在（修复后） |
|------|-------------|--------------|
| **SRDF Groups** | left_gripper + right_gripper | ❌ 无 gripper groups |
| **MoveIt Controllers** | 包含 gripper controllers | ❌ 无 gripper controllers |
| **关节匹配** | ❌ 不匹配 | ✅ 匹配（只 arm） |
| **错误消息** | 每秒多次 gripper 错误 | ✅ 无错误 |
| **MoveIt Planning** | ❌ 不稳定 | ✅ 正常工作 |

---

## 💡 验证成功的标志

1. ✅ **无关节错误** - 启动过程无 "Joint not found" 错误
2. ✅ **move_group 正常运行** - `/move_group` 节点存在
3. ✅ **控制器活跃** - `ros2 control list_controllers` 显示全部 active
4. ✅ **手臂可规划** - PyMoveIt2 脚本能正常规划路径

---

**测试日期:** 2026-04-14  
**预期结果:** ✅ 所有测试通过，无 gripper 关节错误
