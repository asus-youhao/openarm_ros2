# Controller Interfaces Documentation

本文檔說明 OpenArm 雙臂系統各控制器使用的 ROS2 介面資料結構。

## 控制器概覽

系統包含三個主要控制器：
- **Left Arm** (左臂) - 7 DOF
- **Right Arm** (右臂) - 7 DOF  
- **Right LEAP Hand** (右手) - 16 DOF

---

## 1. Topic 模式 (Direct Position Control)

使用 `forward_position_controller` 進行直接位置控制。

### 資料結構
**Message Type**: `std_msgs/Float64MultiArray`

### Topic 列表

| Controller | Topic | DOF | 說明 |
|------------|-------|-----|------|
| left_arm | `/left_forward_position_controller/commands` | 7 | 左臂位置指令 |
| right_arm | `/right_forward_position_controller/commands` | 7 | 右臂位置指令 |
| right_leaphand | `/right_hand_forward_position_controller/commands` | 16 | 右手位置指令 |

### 資料格式
```python
# std_msgs/Float64MultiArray
{
    "data": [pos1, pos2, pos3, ...]  # 關節位置陣列 (單位: radians)
}
```

### 使用範例
```python
from std_msgs.msg import Float64MultiArray

# 發布左臂指令
msg = Float64MultiArray()
msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 7個關節
left_arm_pub.publish(msg)

# 發布右手指令
msg = Float64MultiArray()
msg.data = [0.0] * 16  # 16個關節
right_hand_pub.publish(msg)
```

---

## 2. Action 模式 (Trajectory Control)

使用 `joint_trajectory_controller` 進行軌跡控制。

### 資料結構
**Action Type**: `control_msgs/action/FollowJointTrajectory`

### Action Server 列表

| Controller | Action Server | DOF | 說明 |
|------------|--------------|-----|------|
| left_arm | `/left_joint_trajectory_controller/follow_joint_trajectory` | 7 | 左臂軌跡控制 |
| right_arm | `/right_joint_trajectory_controller/follow_joint_trajectory` | 7 | 右臂軌跡控制 |
| right_leaphand | `/right_hand_controller/follow_joint_trajectory` | 16 | 右手軌跡控制 |

### 資料格式
```python
# control_msgs/action/FollowJointTrajectory
Goal:
  trajectory:
    joint_names: [joint1, joint2, ...]  # 關節名稱列表
    points:
      - positions: [pos1, pos2, ...]     # 關節位置
        velocities: []                    # (可選) 速度
        accelerations: []                 # (可選) 加速度
        effort: []                        # (可選) 力矩
        time_from_start:                  # 到達時間
          sec: 0
          nanosec: 500000000             # 500ms
```

### 使用範例
```python
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# 創建 goal
goal_msg = FollowJointTrajectory.Goal()
goal_msg.trajectory.joint_names = [
    'openarm_left_joint1', 'openarm_left_joint2', 
    # ... 其他關節
]

# 創建軌跡點
point = JointTrajectoryPoint()
point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
point.time_from_start = Duration(sec=0, nanosec=500000000)

goal_msg.trajectory.points = [point]

# 發送 goal
action_client.send_goal_async(goal_msg)
```

---

## 3. 關節名稱對照表

### Left Arm (7 DOF)
```python
left_arm_joints = [
    'openarm_left_joint1',   # 基座旋轉
    'openarm_left_joint2',   # 肩部俯仰
    'openarm_left_joint3',   # 肩部側擺
    'openarm_left_joint4',   # 肘部
    'openarm_left_joint5',   # 腕部旋轉1
    'openarm_left_joint6',   # 腕部俯仰
    'openarm_left_joint7'    # 腕部旋轉2
]
```

### Right Arm (7 DOF)
```python
right_arm_joints = [
    'openarm_right_joint1',  # 基座旋轉
    'openarm_right_joint2',  # 肩部俯仰
    'openarm_right_joint3',  # 肩部側擺
    'openarm_right_joint4',  # 肘部
    'openarm_right_joint5',  # 腕部旋轉1
    'openarm_right_joint6',  # 腕部俯仰
    'openarm_right_joint7'   # 腕部旋轉2
]
```

### Right LEAP Hand (16 DOF)
```python
right_hand_joints = [
    # 食指 (Index)
    'right_index_mcp_side',     # 側擺
    'right_index_mcp_forward',  # 前彎
    'right_index_pip',          # 近指間關節
    'right_index_dip',          # 遠指間關節
    
    # 中指 (Middle)
    'right_middle_mcp_side',    # 側擺
    'right_middle_mcp_forward', # 前彎
    'right_middle_pip',         # 近指間關節
    'right_middle_dip',         # 遠指間關節
    
    # 無名指 (Ring)
    'right_ring_mcp_side',      # 側擺
    'right_ring_mcp_forward',   # 前彎
    'right_ring_pip',           # 近指間關節
    'right_ring_dip',           # 遠指間關節
    
    # 拇指 (Thumb)
    'right_thumb_mcp_side',     # 側擺
    'right_thumb_mcp_forward',  # 前彎
    'right_thumb_pip_joint',    # 近指間關節
    'right_thumb_dip_joint'     # 遠指間關節
]
```

---

## 4. 工具程式說明

### 4.1 Record/Replay Script
**檔案**: `record_replay_commands.py`

#### 錄製指令 (Recording)
- **訂閱 Topics**: Topic 模式的 `Float64MultiArray` 訊息
- **記錄格式**: JSON
  ```json
  {
    "timestamp": 0.0,
    "controller": "left_arm|right_arm|right_leaphand",
    "positions": [...]
  }
  ```

#### 重播指令 (Replay)

**Topic 模式** (`-t`):
```bash
python3 record_replay_commands.py replay -t --file recording.json
```
- 發布到: `Float64MultiArray` topics
- 即時重播，無平滑處理

**Action 模式** (`-a`):
```bash
python3 record_replay_commands.py replay -a --file recording.json --batch-size 16
```
- 發送到: `FollowJointTrajectory` action servers
- 批次處理軌跡點
- 平滑軌跡執行

### 4.2 GUI Controllers

#### bimanual_gui_controller_hybrid.py
支援雙模式切換：

**Topic 模式**:
```bash
python3 bimanual_gui_controller_hybrid.py --mode topic
```
- 使用 `Float64MultiArray` 發布

**Action 模式**:
```bash
python3 bimanual_gui_controller_hybrid.py --mode action
```
- 使用 `FollowJointTrajectory` action

---

## 5. 資料流程圖

### Topic 模式
```
GUI/Script → Float64MultiArray → forward_position_controller → Hardware
```

### Action 模式
```
GUI/Script → FollowJointTrajectory → joint_trajectory_controller → Hardware
              (Goal)                    (Trajectory Planning)
```

---

## 6. 選擇建議

| 使用情境 | 建議模式 | 原因 |
|---------|---------|------|
| 即時控制 | Topic | 低延遲，直接控制 |
| 軌跡執行 | Action | 平滑運動，插值處理 |
| 錄製重播 | Topic (錄) + Action (播) | 錄製快速，重播平滑 |
| 手動操作 | Topic | 即時響應 |
| 自動化任務 | Action | 軌跡規劃完整 |

---

## 7. 範例程式

完整範例請參考：
- `scripts/record_replay_commands.py` - 錄製與重播
- `scripts/bimanual_gui_controller_hybrid.py` - 混合模式 GUI
- `scripts/bimanual_gui_controller_topic.py` - Topic 模式 GUI
- `scripts/bimanual_gui_controller_action.py` - Action 模式 GUI

---

## 8. 常見問題

### Q: Topic 模式和 Action 模式有什麼差別？
A: 
- **Topic 模式**: 直接發送位置指令，無軌跡規劃，適合即時控制
- **Action 模式**: 發送軌跡目標，控制器會進行插值和平滑處理，適合預定軌跡執行

### Q: 如何選擇批次大小 (batch-size)？
A: 
- 較小值 (如 8-16): 更頻繁的反饋，適合複雜軌跡
- 較大值 (如 32-64): 減少通訊開銷，適合簡單軌跡

### Q: 為什麼右手控制器名稱不同？
A: 
- Topic 模式: `/right_hand_forward_position_controller/commands`
- Action 模式: `/right_hand_controller/follow_joint_trajectory`
- 這是因為 LEAP Hand 使用不同的控制器配置

---

## 更新日誌
- 2026-02-02: 初始版本，支援三控制器 (left_arm, right_arm, right_leaphand)
