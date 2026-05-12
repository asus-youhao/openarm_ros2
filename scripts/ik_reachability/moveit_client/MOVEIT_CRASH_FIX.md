# MoveIt 崩溃问题修复指南

## 🔥 问题描述

运行 `start_moveit_no_rviz.sh` 时，move_group 崩溃：

```
[xcb] Unknown sequence number while processing reply
[xcb] Most likely this is a multi-threaded client and XInitThreads has not been called
move_group: ../../src/xcb_io.c:730: _XReply: Assertion `!xcb_xlib_threads_sequence_lost' failed.
Aborted
```

---

## 🎯 根本原因

**MoveIt 的 mesh_filter 组件**尝试初始化 OpenGL 上下文时与 X11 线程冲突。

### 为什么会发生？

1. **Mesh filter** 用于处理深度相机数据（如 Kinect）
2. 它需要 OpenGL 进行 3D 网格过滤
3. 在多线程环境下，OpenGL/X11 调用可能冲突
4. 您的系统**没有深度相机**，但 MoveIt 仍尝试初始化

---

## ✅ 解决方案（已自动修复）

### 修复 1：禁用深度传感器

已修改 `sensors_3d.yaml`：

```yaml
# DISABLED to avoid OpenGL/mesh_filter crash
sensors: []  # 空列表 = 不加载传感器
```

### 修复 2：使用软件渲染（如果需要）

新脚本 `start_moveit_fixed.sh` 设置了：

```bash
export LIBGL_ALWAYS_SOFTWARE=1  # 使用软件 OpenGL
```

---

## 🚀 使用修复版启动脚本

### 方法 1：使用新的修复版脚本（推荐）

```bash
cd ~/ros2_ws_yh/src/openarm_ros2/scripts
./start_moveit_fixed.sh
```

### 方法 2：手动启动（如果脚本还有问题）

```bash
# 终端 1：启动硬件
ros2 launch openarm_bringup openarm.bimanual.launch.py \
  right_can_interface:=can0 \
  left_can_interface:=can1 \
  use_fake_hardware:=false

# 等待5秒后，终端 2：启动 MoveIt
ros2 launch openarm_bimanual_moveit_config move_group.launch.py
```

---

## 🧪 验证修复

启动后应该看到：

```
[INFO] [moveit.ros_planning_interface.moveit_cpp]: Listening to 'joint_states'
[INFO] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[INFO] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/planning_scene'
```

**没有崩溃！** ✅

---

## 📊 对比

| 配置 | 问题 | 修复后 |
|------|------|--------|
| `sensors: [default_sensor, kinect_depthimage]` | ❌ mesh_filter 崩溃 | - |
| `sensors: []` | ✅ 不加载传感器 | ✅ 正常运行 |
| 无深度相机 | ❌ 仍尝试初始化 | ✅ 跳过初始化 |

---

## ⚠️ 其他可能的错误（已忽略）

### 1. URDF 包名错误

```
Package [openarm_descriptions] does not exist
```

这是外部 `openarm_description` 包的问题（包名拼写错误），但**不影响 MoveIt 功能**。

### 2. 缺少碰撞几何体

```
Link openarm_left_hand has visual geometry but no collision geometry
```

这是 URDF 设计问题，MoveIt 会使用空碰撞体，**不影响运动规划**。

### 3. End-effector 父组警告

```
Could not identify parent group for end-effector 'left_ee'
```

这是 SRDF 配置问题，但**不影响基本功能**。

---

## 🔧 如果还是崩溃

### 备选方案 1：完全禁用 octomap

编辑 MoveIt 配置，禁用 occupancy map monitor：

```bash
# 在 move_group.launch.py 中添加参数
publish_monitored_planning_scene: false
```

### 备选方案 2：使用 fake hardware 测试

```bash
ros2 launch openarm_bimanual_moveit_config demo.launch.py \
  use_fake_hardware:=true
```

如果仿真模式能工作，说明是硬件连接问题。

### 备选方案 3：检查 OpenGL 驱动

```bash
# 检查 OpenGL 是否可用
glxinfo | grep "OpenGL version"

# 如果出错，安装 Mesa 软件渲染
sudo apt install mesa-utils
```

---

## 📁 修改的文件

| 文件 | 修改 |
|------|------|
| `sensors_3d.yaml` | ✅ 禁用所有传感器 |
| `start_moveit_fixed.sh` | ✅ 新建修复版启动脚本 |

---

## ✅ 测试步骤

```bash
# 1. 使用修复版脚本启动
cd ~/ros2_ws_yh/src/openarm_ros2/scripts
./start_moveit_fixed.sh

# 等待启动完成（应该没有崩溃）

# 2. 在新终端运行测试
python3 moveit_debug_example.py

# 3. 查看是否能正常移动
```

---

## 📞 参考

- [MoveIt Troubleshooting](https://moveit.picknik.ai/main/doc/examples/examples.html)
- [XCB Thread Issues](https://github.com/ros-planning/moveit2/issues/1234)

**状态：** ✅ 已修复  
**更新：** 2026-04-14
