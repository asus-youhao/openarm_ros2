# LEAP Hand 硬件接口集成说明

## 已完成的修改

### 1. Launch 文件修改
✅ `/home/asus/ros2_ws_yh/src/openarm_ros2/openarm_bringup/launch/openarm.bimanual.launch.py`
- 移除了 `right_hand_controller` 只在 fake_hardware 时启动的限制
- 现在只要 `ee_type=leap_hand_right` 就会启动控制器（无论真假硬件）

### 2. 新增硬件接口
✅ `/home/asus/ros2_ws_yh/src/openarm_ros2/openarm_hardware/include/openarm_hardware/leap_hand_hardware.hpp`
✅ `/home/asus/ros2_ws_yh/src/openarm_ros2/openarm_hardware/src/leap_hand_hardware.cpp`
- 创建了独立的 `LeapHandHardware` 类
- 实现了 ros2_control `SystemInterface`
- 自动处理 URDF (0=home) 和 LEAP (3.14=home) 坐标转换
- 支持串口自动搜索 (/dev/ttyUSB0, /dev/ttyUSB1, /dev/ttyUSB2)

### 3. Plugin 配置
✅ `/home/asus/ros2_ws_yh/src/openarm_ros2/openarm_hardware/openarm_hardware.xml`
- 注册了 `openarm_hardware/LeapHandHardware` plugin

### 4. 测试脚本更新
✅ `/home/asus/ros2_ws_yh/src/openarm_ros2/scripts/test_leap_hand.py`
- 支持 LEAP 坐标和 URDF 坐标自动转换

## 使用方法

### 启动系统（fake_hardware）：
```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py \\
  arm_type:=v10 \\
  ee_type:=leap_hand_right \\
  use_fake_hardware:=true
```

### 启动系统（真实硬件 - 需要完善 Dynamixel SDK 集成）：
```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py \\
  arm_type:=v10 \\
  ee_type:=leap_hand_right \\
  use_fake_hardware:=false
```

### 测试手指控制：
```bash
# 抓取姿势
python3 /home/asus/ros2_ws_yh/src/openarm_ros2/scripts/test_leap_hand.py grasp

# 展开姿势  
python3 /home/asus/ros2_ws_yh/src/openarm_ros2/scripts/test_leap_hand.py open

# 同时测试两个接口
python3 /home/asus/ros2_ws_yh/src/openarm_ros2/scripts/test_leap_hand.py grasp --controller --leap
```

### Dynamixel SDK 集成
当前硬件接口只是框架，需要完善实际的 Dynamixel 通信：

1. **添加 Dynamixel SDK 依赖**：
   - 在 CMakeLists.txt 中添加 `find_package(dynamixel_sdk REQUIRED)`
   - 参考 `/home/asus/LEAP_Hand_API/ros2_module/scripts/leap_hand_utils/dynamixel_client.py`

2. **实现串口通信函数**：
   - `connect_serial()` - 初始化 Dynamixel motors
   - `send_position_command()` - 写入目标位置
   - `read_joint_states()` - 读取当前位置/速度/电流

3. **参考代码**：
   ```cpp
   // 需要实现类似 Python 中的功能：
   // self.dxl_client = DynamixelClient(self.motors, '/dev/ttyUSB0', 4000000)
   // self.dxl_client.connect()
   // self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
   // output = self.dxl_client.read_pos_vel_cur()
   ```

## 坐标系统参考

| 系统 | Home Pose | 闭合方向 | 用途 |
|------|-----------|----------|------|
| URDF/ros2_control | 0.0 rad | 正值 | ROS 标准 |
| LEAP 原生 | 3.14 rad | > 3.14 | 硬件接口 |

**转换公式**：
- `URDF_position = LEAP_command - 3.14`
- `LEAP_command = URDF_position + 3.14`

