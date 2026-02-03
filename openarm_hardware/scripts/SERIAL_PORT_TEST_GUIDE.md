# LEAP Hand Serial Port 测试指南

## 🔧 测试工具

使用 `test_leap_serial_port.py` 独立测试 LEAP Hand 串口通信。

## 📋 安装与编译

```bash
cd /home/asus/ros2_ws_yh
colcon build --packages-select openarm_hardware
source install/setup.bash
```

## 🚀 使用方法

### 1. 扫描可用串口

```bash
ros2 run openarm_hardware test_leap_serial_port.py --scan
```

**输出示例**:
```
============================================================
Scanning for available serial ports...
============================================================

✓ Found ports using serial.tools.list_ports:
  • /dev/ttyUSB0
    Description: USB Serial Port
    Hardware ID: USB VID:PID=0403:6014
  • /dev/ttyUSB1
    Description: USB Serial Port
```

### 2. 测试特定串口连接

```bash
# 测试 /dev/ttyUSB0
ros2 run openarm_hardware test_leap_serial_port.py --port /dev/ttyUSB0
```

**输出示例**:
```
============================================================
Testing connection to: /dev/ttyUSB0
Baudrate: 4000000
Timeout: 1.0s
============================================================

[1/3] Opening serial port /dev/ttyUSB0...
✓ Serial port opened successfully
  Port: /dev/ttyUSB0
  Baudrate: 4000000
  Is open: True

[2/3] Testing port readability...
✓ Port readable (no data waiting)

[3/3] Testing port writability...
✓ Wrote 3 bytes

============================================================
✅ CONNECTION TEST PASSED
============================================================

Press Enter to disconnect...

============================================================
Testing disconnection...
============================================================

[1/2] Closing port /dev/ttyUSB0...
✓ Port closed

[2/2] Verifying port is closed...
✓ Port is confirmed closed

============================================================
✅ DISCONNECTION TEST PASSED
============================================================
```

### 3. 自动查找并测试第一个可用端口

```bash
ros2 run openarm_hardware test_leap_serial_port.py --auto-find
```

### 4. 测试重连稳定性（多次连接/断开循环）

```bash
# 测试 5 次重连循环
ros2 run openarm_hardware test_leap_serial_port.py --port /dev/ttyUSB0 --reconnect 5
```

**输出示例**:
```
============================================================
Testing reconnection cycles (x5)...
============================================================

--- Cycle 1/5 ---
✅ CONNECTION TEST PASSED
✅ DISCONNECTION TEST PASSED

--- Cycle 2/5 ---
✅ CONNECTION TEST PASSED
✅ DISCONNECTION TEST PASSED

...

============================================================
Reconnection Test Summary:
  Success: 5/5
  Result: ✅ PASSED
============================================================
```

## 🔍 与 Launch 文件配合测试

### Step 1: 启动系统（fake_hardware 模式）

```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py \
  arm_type:=v10 \
  ee_type:=leap_hand_right \
  use_fake_hardware:=true
```

### Step 2: 在另一个终端测试串口

```bash
# Terminal 2
source /home/asus/ros2_ws_yh/install/setup.bash
ros2 run openarm_hardware test_leap_serial_port.py --scan
```

### Step 3: 测试控制器通信

```bash
# Terminal 3
source /home/asus/ros2_ws_yh/install/setup.bash

# 发送 home 位置
ros2 topic pub --once /right_hand_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

# 发送 grasp 位置
ros2 topic pub --once /right_hand_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.525, 1.11, 0.86, 0.0, 0.525, 1.11, 0.86, 0.0, 0.525, 1.11, 0.86, 1.46, 0.56, 0.71, 1.36]}"
```

### Step 4: 启动真实硬件（需要连接 LEAP Hand）

```bash
# 先测试串口
ros2 run openarm_hardware test_leap_serial_port.py --port /dev/ttyUSB0

# 确认连接成功后启动系统
ros2 launch openarm_bringup openarm.bimanual.launch.py \
  arm_type:=v10 \
  ee_type:=leap_hand_right \
  use_fake_hardware:=false
```

## 🐛 常见问题排查

### 问题 1: 找不到串口设备

**症状**:
```
❌ CONNECTION TEST FAILED
Error: [Errno 2] could not open port /dev/ttyUSB0: [Errno 2] No such file or directory
```

**解决方案**:
```bash
# 1. 检查设备是否连接
ls -l /dev/ttyUSB*

# 2. 扫描所有串口
ros2 run openarm_hardware test_leap_serial_port.py --scan

# 3. 检查 dmesg 查看设备消息
dmesg | tail -20
```

### 问题 2: 权限不足

**症状**:
```
❌ CONNECTION TEST FAILED
Error: [Errno 13] could not open port /dev/ttyUSB0: [Errno 13] Permission denied
```

**解决方案**:
```bash
# 临时修改权限
sudo chmod 666 /dev/ttyUSB0

# 永久解决：将用户加入 dialout 组
sudo usermod -aG dialout $USER
# 然后注销重新登录

# 验证用户组
groups
```

### 问题 3: 端口被占用

**症状**:
```
❌ CONNECTION TEST FAILED
Error: [Errno 16] could not open port /dev/ttyUSB0: [Errno 16] Device or resource busy
```

**解决方案**:
```bash
# 查找占用进程
sudo lsof /dev/ttyUSB0

# 或
sudo fuser /dev/ttyUSB0

# 终止占用进程
sudo fuser -k /dev/ttyUSB0
```

### 问题 4: 波特率不匹配

**症状**: 能连接但无法通信

**解决方案**:
- LEAP Hand 使用 **4000000 bps** (4 Mbps)
- 确保设备支持该波特率
- 检查 Dynamixel 电机配置

## 📊 测试命令总结

| 命令 | 功能 |
|------|------|
| `--scan` | 扫描所有可用串口 |
| `--port /dev/ttyUSB0` | 测试指定串口 |
| `--auto-find` | 自动查找第一个可用端口 |
| `--reconnect 5` | 测试 5 次重连循环 |

## 🔗 相关文件

- **测试脚本**: [scripts/test_leap_serial_port.py](../scripts/test_leap_serial_port.py)
- **硬件接口**: [include/openarm_hardware/leap_hand_hardware.hpp](../include/openarm_hardware/leap_hand_hardware.hpp)
- **实现文件**: [src/leap_hand_hardware.cpp](../src/leap_hand_hardware.cpp)

## 📝 下一步

完成串口测试后：

1. **实现 Dynamixel SDK 集成**
   ```bash
   sudo apt install ros-humble-dynamixel-sdk
   ```

2. **更新 connect_serial() 函数**
   - 初始化 Dynamixel motors
   - 设置 PID 参数
   - 启用扭矩控制

3. **更新 send_position_command() 函数**
   - 实现同步写入
   - 坐标转换 (URDF → LEAP)

4. **更新 read_joint_states() 函数**
   - 读取位置/速度/电流
   - 坐标转换 (LEAP → URDF)

5. **测试完整系统**
   ```bash
   ros2 launch openarm_bringup openarm.bimanual.launch.py \
     arm_type:=v10 \
     ee_type:=leap_hand_right
   ```
