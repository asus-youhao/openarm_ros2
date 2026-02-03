# Right Hand Action Client 使用說明

本文件說明如何透過 ROS2 Action 介面，對右手控制器 `/right_hand_controller/follow_joint_trajectory` 送出目標姿態（16 DOF）。

---

## 簡介
- **控制器**: `joint_trajectory_controller`
- **Action**: `control_msgs/action/FollowJointTrajectory`
- **Server 名稱**: `/right_hand_controller/follow_joint_trajectory`
- **關節數**: 16（食指、中指、無名指、拇指各 4 關節）

---

## 關節順序（16 DOF）
依序填入 16 個弧度值（radians），對應下列關節：
```
right_index_mcp_side,
right_index_mcp_forward,
right_index_pip,
right_index_dip,
right_middle_mcp_side,
right_middle_mcp_forward,
right_middle_pip,
right_middle_dip,
right_ring_mcp_side,
right_ring_mcp_forward,
right_ring_pip,
right_ring_dip,
right_thumb_mcp_side,
right_thumb_mcp_forward,
right_thumb_pip_joint,
right_thumb_dip_joint
```

---

## 快速使用（推薦）
已提供可直接執行的腳本：`scripts/right_hand_action_client.py`

- 預設張開手（全 0）
```bash
python3 scripts/right_hand_action_client.py
```

- 使用預設握拳姿勢（grasp），指定到達時間 1.0s
```bash
python3 scripts/right_hand_action_client.py --preset grasp --duration 1.0
```

- 自訂 16 個關節角度（單位：rad），到達時間 0.8s
```bash
python3 scripts/right_hand_action_client.py --positions \
  0 0 0.525 1.11  \
  0 0 0.525 1.11  \
  0 0 0.525 1.11  \
  0 1.46 0.56 0.71 \
  --duration 0.8
```

- 指定非預設 server 名稱（若有命名空間或改名）
```bash
python3 scripts/right_hand_action_client.py --server /right_hand_controller/follow_joint_trajectory
```

---

## 參數說明
- `--preset {open,grasp}`: 使用內建手勢（預設 `open` 全 0；`grasp` 為抓握姿勢）
- `--positions P1 ... P16`: 明確指定 16 個關節角度（radians）
- `--duration SEC`: 到達目標的時間（秒），會設定到 `JointTrajectoryPoint.time_from_start`
- `--server NAME`: Action Server 名稱（預設 `/right_hand_controller/follow_joint_trajectory`）

> 注意：`--preset` 與 `--positions` 互斥，擇一使用；未指定則使用 `open`。

---

## 範例程式片段（Python API）
以下展示以程式送出單一軌跡點的方式：
```python
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

client = ActionClient(node, FollowJointTrajectory, '/right_hand_controller/follow_joint_trajectory')
client.wait_for_server()

goal = FollowJointTrajectory.Goal()
goal.trajectory.joint_names = RIGHT_HAND_JOINTS

pt = JointTrajectoryPoint()
pt.positions = positions_16_float  # 長度需為 16
pt.time_from_start = Duration(sec=0, nanosec=800_000_000)  # 0.8s

goal.trajectory.points = [pt]

send_future = client.send_goal_async(goal)
# 等待接受並等待結果...
```

---

## 常見問題
- **Server 無回應**：確認右手控制器已啟動（`/right_hand_controller/follow_joint_trajectory` 可見）且無命名空間衝突。
- **長度錯誤**：`positions` 陣列必須是 16 筆數值，順序需與上方關節列表一致。
- **運動不平滑 / 太快**：適度增加 `--duration` 時間；或改用多點軌跡（多個 `JointTrajectoryPoint`）。

---

## 進階：多點軌跡（說明）
若需要更平滑或分段動作，可在 `trajectory.points` 中加入多個 `JointTrajectoryPoint`，每個點的 `time_from_start` 需遞增。腳本目前範例為單點，若需要我可協助擴充為多點版本或載入 JSON 手勢。

---

## 相關檔案
- `scripts/right_hand_action_client.py`: 可直接執行的 Action Client 腳本
- `scripts/CONTROLLER_INTERFACES.md`: 系統整體介面與資料結構說明
