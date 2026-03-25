# O6 Hands Testing Guide

O6 Hand (LinkerHand) 硬體介面測試指南。

## 硬體連接

- **Right Hand**: CAN2 介面（ID: 0x027）
- **Left Hand**: CAN3 介面（ID: 0x027）

確認 CAN 介面已設定：
```bash
# 檢查 CAN 介面狀態
ip link show can2
ip link show can3

# 如果需要設定 CAN 介面
sudo ip link set can2 type can bitrate 1000000
sudo ip link set can2 up
sudo ip link set can3 type can bitrate 1000000
sudo ip link set can3 up
```

## 單手測試

### 測試 Right Hand

#### 方式 1: Joint Trajectory Controller (Action Server, 推薦)

```bash
# 啟動控制器 (預設使用 joint_trajectory_controller)
ros2 launch openarm_bringup o6_hand_hardware_test.launch.py \
    can_interface:=can2 \
    hand_type:=right

# 或明確指定
ros2 launch openarm_bringup o6_hand_hardware_test.launch.py \
    can_interface:=can2 \
    hand_type:=right \
    robot_controller:=joint_trajectory_controller

# 查看可用的 action servers
ros2 action list
# 輸出: /hand_controller/follow_joint_trajectory

# 發送握拳指令
ros2 action send_goal /hand_controller/follow_joint_trajectory \
    control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [R_thumb_cmc_yaw, R_thumb_cmc_pitch, R_index_mcp_pitch, R_middle_mcp_pitch, R_ring_mcp_pitch, R_pinky_mcp_pitch],
    points: [{positions: [0.3, 0.7, 0.8, 0.8, 0.8, 0.8], time_from_start: {sec: 2}}]
  }
}" --feedback

# 張開手指令
ros2 action send_goal /hand_controller/follow_joint_trajectory \
    control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [R_thumb_cmc_yaw, R_thumb_cmc_pitch, R_index_mcp_pitch, R_middle_mcp_pitch, R_ring_mcp_pitch, R_pinky_mcp_pitch],
    points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]
  }
}"
```

#### 方式 2: Forward Position Controller (Topic-based)

```bash
# 啟動控制器 (使用 forward_position_controller)
ros2 launch openarm_bringup o6_hand_hardware_test.launch.py \
    can_interface:=can2 \
    hand_type:=right \
    robot_controller:=forward_position_controller

# 查看可用的 topics
ros2 topic list | grep commands
# 輸出: /hand_controller/commands

# 發送握拳指令 (使用 topic)
ros2 topic pub /hand_controller/commands std_msgs/msg/Float64MultiArray \
    "{data: [0.3, 0.7, 0.8, 0.8, 0.8, 0.8]}" --once

# 張開手指令
ros2 topic pub /hand_controller/commands std_msgs/msg/Float64MultiArray \
    "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}" --once

# 自定義手勢
ros2 topic pub /hand_controller/commands std_msgs/msg/Float64MultiArray \
    "{data: [0.2, 0.3, 0.3, 0.3, 0.3, 0.1]}" --once
```

### 測試 Left Hand

#### 方式 1: Joint Trajectory Controller (Action Server)

```bash
# 啟動控制器
ros2 launch openarm_bringup o6_hand_hardware_test.launch.py \
    can_interface:=can3 \
    hand_type:=left \
    hand_prefix:=L_ \
    robot_controller:=joint_trajectory_controller

# 發送指令（注意關節名稱改為 L_ 前綴）
ros2 action send_goal /hand_controller/follow_joint_trajectory \
    control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [L_thumb_cmc_yaw, L_thumb_cmc_pitch, L_index_mcp_pitch, L_middle_mcp_pitch, L_ring_mcp_pitch, L_pinky_mcp_pitch],
    points: [{positions: [0.3, 0.7, 0.8, 0.8, 0.8, 0.8], time_from_start: {sec: 2}}]
  }
}" --feedback
```

#### 方式 2: Forward Position Controller (Topic-based)

```bash
# 啟動控制器
ros2 launch openarm_bringup o6_hand_hardware_test.launch.py \
    can_interface:=can3 \
    hand_type:=left \
    hand_prefix:=L_ \
    robot_controller:=forward_position_controller

# 發送指令
ros2 topic pub /hand_controller/commands std_msgs/msg/Float64MultiArray \
    "{data: [0.3, 0.7, 0.8, 0.8, 0.8, 0.8]}" --once
```

## 雙手測試

### 啟動雙手系統

#### 方式 1: Joint Trajectory Controller (Action Server, 推薦)

```bash
# 使用雙手專用 launch file (預設使用 joint_trajectory_controller)
ros2 launch openarm_bringup o6_bimanual_hardware_test.launch.py

# 或使用通用 launch file
ros2 launch openarm_bringup o6_hand_hardware_test.launch.py \
    bimanual:=true

# 自訂 CAN 介面
ros2 launch openarm_bringup o6_bimanual_hardware_test.launch.py \
    right_can:=can0 \
    left_can:=can1

# 啟動 RViz 視覺化
ros2 launch openarm_bringup o6_bimanual_hardware_test.launch.py \
    use_rviz:=true

# 查看可用的 action servers
ros2 action list
# 輸出:
# /left_hand_controller/follow_joint_trajectory
# /right_hand_controller/follow_joint_trajectory
```

#### 方式 2: Forward Position Controller (Topic-based)

```bash
# 使用 forward_position_controller
ros2 launch openarm_bringup o6_bimanual_hardware_test.launch.py \
    robot_controller:=forward_position_controller

# 或使用通用 launch file
ros2 launch openarm_bringup o6_hand_hardware_test.launch.py \
    bimanual:=true \
    robot_controller:=forward_position_controller

# 查看可用的 topics
ros2 topic list | grep commands
# 輸出:
# /left_hand_controller/commands
# /right_hand_controller/commands
```

### 使用測試腳本

```bash
# 張開雙手
python3 scripts/test_o6_bimanual.py open

# 握拳雙手
python3 scripts/test_o6_bimanual.py close

# 中等握力（抓取）
python3 scripts/test_o6_bimanual.py grasp

# 雙手指向（食指伸出）
python3 scripts/test_o6_bimanual.py point

# 鏡像測試（右手關閉，左手張開）
python3 scripts/test_o6_bimanual.py mirror
```

### 手動發送雙手指令

#### 使用 Action (Joint Trajectory Controller)

```bash
# Right hand 握拳
ros2 action send_goal /right_hand_controller/follow_joint_trajectory \
    control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [R_thumb_cmc_yaw, R_thumb_cmc_pitch, R_index_mcp_pitch, R_middle_mcp_pitch, R_ring_mcp_pitch, R_pinky_mcp_pitch],
    points: [{positions: [0.3, 0.7, 0.8, 0.8, 0.8, 0.8], time_from_start: {sec: 2}}]
  }
}"

# Left hand 張開
ros2 action send_goal /left_hand_controller/follow_joint_trajectory \
    control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [L_thumb_cmc_yaw, L_thumb_cmc_pitch, L_index_mcp_pitch, L_middle_mcp_pitch, L_ring_mcp_pitch, L_pinky_mcp_pitch],
    points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]
  }
}"
```

#### 使用 Topic (Forward Position Controller)

```bash
# Right hand 握拳
ros2 topic pub /right_hand_controller/commands std_msgs/msg/Float64MultiArray \
    "{data: [0.3, 0.7, 0.8, 0.8, 0.8, 0.8]}" --once

# Right hand 張開
ros2 topic pub /right_hand_controller/commands std_msgs/msg/Float64MultiArray \
    "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}" --once

# Right hand 自定義手勢
ros2 topic pub /right_hand_controller/commands std_msgs/msg/Float64MultiArray \
    "{data: [0.2, 0.3, 0.3, 0.3, 0.3, 0.1]}" --once

# Left hand 握拳
ros2 topic pub /left_hand_controller/commands std_msgs/msg/Float64MultiArray \
    "{data: [0.3, 0.7, 0.8, 0.8, 0.8, 0.8]}" --once

# Left hand 張開
ros2 topic pub /left_hand_controller/commands std_msgs/msg/Float64MultiArray \
    "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}" --once
```

## 關節資訊

### 關節順序（URDF order）
1. `thumb_cmc_yaw` - 大拇指 CMC 偏轉 (0.0 - 0.58 rad)
2. `thumb_cmc_pitch` - 大拇指 CMC 俯仰 (0.0 - 1.36 rad)
3. `index_mcp_pitch` - 食指 MCP 俯仰 (0.0 - 1.6 rad)
4. `middle_mcp_pitch` - 中指 MCP 俯仰 (0.0 - 1.6 rad)
5. `ring_mcp_pitch` - 無名指 MCP 俯仰 (0.0 - 1.6 rad)
6. `pinky_mcp_pitch` - 小指 MCP 俯仰 (0.0 - 1.6 rad)

### 映射關係
- **Range 值**: 0-255 (LinkerHandApi SDK)
- **弧度值**: 0.0-max rad (ROS2 Control)
- **映射方向**:
  - Range 255 → 0.0 rad (張開/open)
  - Range 0 → max rad (握拳/closed)

### 前綴
- Right hand: `R_` (例如: `R_thumb_cmc_yaw`)
- Left hand: `L_` (例如: `L_thumb_cmc_yaw`)

## 監控狀態

```bash
# 查看關節狀態
ros2 topic echo /joint_states

# 查看右手控制器狀態
ros2 topic echo /right_hand_controller/state

# 查看左手控制器狀態
ros2 topic echo /left_hand_controller/state

# 查看可用的 action servers
ros2 action list

# 查看 action 介面
ros2 action info /right_hand_controller/follow_joint_trajectory
ros2 action info /left_hand_controller/follow_joint_trajectory
```

## 故障排除

### CAN 介面未就緒
```bash
# 重新設定 CAN 介面
sudo ip link set can2 down
sudo ip link set can2 type can bitrate 1000000
sudo ip link set can2 up

# 檢查 CAN 訊息
candump can2
```

### 手部沒有回應
1. 確認 CAN 介面正常運作
2. 檢查硬體電源
3. 確認 LinkerHandApi SDK 已正確安裝
4. 查看 ros2_control_node 日誌中的錯誤訊息

### 位置數值異常
- 確認已使用最新版本的硬體介面（含映射修正）
- 檢查 joint_states 中的數值是否在合理範圍內
- 查看調試日誌中的 raw range values

## 開發歷史

### 已修復的問題
1. ✅ 關節順序錯誤 - API 順序（pitch, yaw）vs URDF 順序（yaw, pitch）
2. ✅ 映射方向反向 - 已修正為 255→0 rad（張開），0→max rad（握拳）
3. ✅ 數據驗證 - 加入 NaN/Inf 檢查、範圍驗證、錯誤處理

### 測試驗證
- ✅ 讀取位置正確
- ✅ 握拳指令正確
- ✅ 張開指令正確
- ✅ Action server 正常運作
- ✅ 雙向通訊無誤

## 文件結構

```
openarm_bringup/
├── launch/
│   ├── o6_hand_hardware_test.launch.py          # 單手測試（通用）
│   └── o6_bimanual_hardware_test.launch.py      # 雙手測試（專用）
├── config/v10_controllers/
│   ├── o6_hand_action_controllers.yaml          # 單手 action server 配置
│   ├── o6_hand_forward_controllers.yaml         # 單手 forward 配置
│   ├── o6_bimanual_action_controllers.yaml      # 雙手 action server 配置
│   └── o6_bimanual_forward_controllers.yaml     # 雙手 forward 配置
└── rviz/
    └── bimanual.rviz                            # 雙手視覺化配置

openarm_hardware/
├── src/
│   └── o6_hand_hardware.cpp                     # O6 手硬體介面
└── include/openarm_hardware/
    └── o6_hand_hardware.hpp                     # 標頭檔

openarm/
└── urdf/
    ├── o6_hand_standalone.urdf.xacro            # 單手 URDF
    └── o6_bimanual_standalone.urdf.xacro        # 雙手 URDF

scripts/
├── test_leap_hand.py                            # LEAP hand 測試腳本
└── test_o6_bimanual.py                          # O6 雙手測試腳本
```
