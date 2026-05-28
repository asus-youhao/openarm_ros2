# Real-time Joint Actions Plotter

## 功能

實時顯示 `/joint_actions`（命令）和 `/joint_states`（實際位置）的對比圖表。

### 特點
- ✅ **實時滾動顯示**：自動更新，顯示最近 N 秒的數據
- ✅ **自動檢測配置**：從 `/joint_actions` 自動識別關節數量和名稱
- ✅ **色彩編碼**：
  - 🔵 左臂（藍色）
  - 🟠 右臂（橙色）
  - 🟢 左手（綠色）
  - 🔴 右手（紅色）
- ✅ **靈活顯示**：可選擇顯示所有關節、僅手臂或特定關節
- ✅ **自動縮放**：Y 軸自動調整範圍

## 使用方法

### 基本用法
```bash
# 顯示所有關節（預設）
ros2 run openarm_ros2 plot_joint_actions_realtime.py

# 或直接運行
cd /home/asus/ros2_ws_yh/src/openarm_ros2/scripts
python3 plot_joint_actions_realtime.py
```

### 僅顯示手臂關節
```bash
# 只顯示 14 個手臂關節（7+7），不顯示手掌
ros2 run openarm_ros2 plot_joint_actions_realtime.py --arms-only
```

### 顯示特定關節
```bash
# 只顯示前 4 個關節（例如：左臂前 4 個 + 右臂前 4 個）
ros2 run openarm_ros2 plot_joint_actions_realtime.py --joints 0,1,2,3,7,8,9,10
```

### 調整時間窗口
```bash
# 顯示最近 20 秒的數據（預設 10 秒）
ros2 run openarm_ros2 plot_joint_actions_realtime.py --window 20

# 顯示最近 5 秒的數據（更快速滾動）
ros2 run openarm_ros2 plot_joint_actions_realtime.py --window 5
```

### 調整更新頻率
```bash
# 30 Hz 更新（更流暢，預設 20 Hz）
ros2 run openarm_ros2 plot_joint_actions_realtime.py --rate 30

# 10 Hz 更新（較省資源）
ros2 run openarm_ros2 plot_joint_actions_realtime.py --rate 10
```

### 組合使用
```bash
# 只顯示手臂，5 秒窗口，30 Hz 更新
ros2 run openarm_ros2 plot_joint_actions_realtime.py --arms-only --window 5 --rate 30
```

## 前置條件

### 1. 確保 topics 正在發布
```bash
# 檢查是否有這些 topics
ros2 topic list | grep -E "(joint_actions|joint_states)"

# 應該看到：
# /joint_actions
# /joint_states
```

### 2. 安裝依賴
```bash
pip install matplotlib numpy
```

### 3. 啟動系統
```bash
# Terminal 1: 啟動硬體或模擬器
ros2 launch openarm_bringup openarm_o6_bimanual.launch.py

# Terminal 2: 啟動 IK（雙臂 + aggregator）
ros2 run openarm_ros2 placo_ik_online_profiler_ws_mesh.py --arm both

# Terminal 3: 運行繪圖工具
python3 plot_joint_actions_realtime.py
```

## 圖表說明

### 每個子圖顯示
- **實線**：命令值（來自 `/joint_actions`）
- **虛線**：實際值（來自 `/joint_states`）
- **X 軸**：時間（秒）
- **Y 軸**：關節位置（弧度）

### 理想情況
- 實線和虛線應該**非常接近**
- 虛線可能有輕微延遲（幾毫秒到幾十毫秒）
- 如果差距很大，可能表示：
  - 控制器調參需要優化
  - 存在機械阻力或負載問題
  - IK 命令變化太快（超出硬體能力）

## 關節順序（26 joints for o6_both）

```
0-6   : openarm_left_joint1-7    (左臂)
7-13  : openarm_right_joint1-7   (右臂)
14-19 : L_thumb/index/middle/ring/pinky (左手 O6)
20-25 : R_thumb/index/middle/ring/pinky (右手 O6)
```

## 快速診斷

### 檢查追蹤誤差
如果想量化誤差而不只是視覺檢查，可以：

```bash
# 終端中訂閱並計算誤差
ros2 topic echo /joint_actions --once > /tmp/cmd.txt &
ros2 topic echo /joint_states --once > /tmp/state.txt
# 然後手動比對數值
```

### 常見問題

**Q: 圖表一直空白**
- A: 確認 `/joint_actions` 和 `/joint_states` 有在發布
  ```bash
  ros2 topic hz /joint_actions
  ros2 topic hz /joint_states
  ```

**Q: 某些關節沒有顯示**
- A: 檢查 joint_names 是否匹配
  ```bash
  ros2 topic echo /joint_actions --once | grep name
  ros2 topic echo /joint_states --once | grep name
  ```

**Q: matplotlib 錯誤**
- A: 安裝正確的 backend
  ```bash
  pip install PyQt5  # 或 pip install tk
  ```

**Q: 圖表卡頓**
- A: 降低更新頻率
  ```bash
  python3 plot_joint_actions_realtime.py --rate 10
  ```

## 進階用法

### 只監控特定手臂
```bash
# 左臂 (joints 0-6)
python3 plot_joint_actions_realtime.py --joints 0,1,2,3,4,5,6

# 右臂 (joints 7-13)
python3 plot_joint_actions_realtime.py --joints 7,8,9,10,11,12,13

# 雙手 (joints 14-25)
python3 plot_joint_actions_realtime.py --joints 14,15,16,17,18,19,20,21,22,23,24,25
```

### 開發/調試模式
```bash
# 超短窗口，快速更新（適合調試）
python3 plot_joint_actions_realtime.py --window 3 --rate 50
```

## 效能考量

- **更新頻率建議**：
  - 正常使用：20 Hz（預設）
  - 流暢顯示：30-50 Hz
  - 省資源：10 Hz

- **時間窗口建議**：
  - 快速診斷：3-5 秒
  - 正常觀察：10 秒（預設）
  - 長期趨勢：20-30 秒

- **CPU 使用**：
  - 26 joints @ 20 Hz ≈ 15-20% CPU（單核）
  - 14 joints @ 20 Hz ≈ 10-15% CPU
  - 使用 `--arms-only` 可顯著降低負載

## 相關工具

- `check_joint_actions_topics.sh` - 檢查 topics 是否正確發布
- `analy_arm_controller.py` - 離線分析（CSV + 圖表）
- `joint_actions_aggregator.py` - 整合多個控制來源

## 截圖示例

```
┌─────────────────────────────────────────────────────────────┐
│ Joint Actions vs States - Real-time | Cmds: 1234 States: 567│
├─────────────────┬─────────────────────────────────────────────┤
│ openarm_left_j1 │     openarm_left_j2                        │
│   ┌─────────┐   │       ┌─────────┐                         │
│   │ ─── cmd │   │       │ ─── cmd │                         │
│   │ - - state   │       │ - - state                         │
│   └─────────┘   │       └─────────┘                         │
├─────────────────┼─────────────────────────────────────────────┤
│      ...        │         ...                                 │
└─────────────────┴─────────────────────────────────────────────┘
```

## 貢獻

如果需要新功能（例如誤差統計、頻譜分析等），請參考此文件並擴展代碼！
