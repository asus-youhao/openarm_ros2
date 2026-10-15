# 13 README：u22 的安裝與 fake 步驟，差異表改成實測結果

## 問題

README 的 u22 只寫「還沒實測，之後補上」，差異表的 Humble 欄位也全是推論；第 2 步還留著 merge 前的分支說明。

## 根因

10–12 才在 Humble 上實測並修好 U1、P2，README 還沒更新。

## 改動

`README.md`：

- 新增「u22（Ubuntu 22.04 + ROS 2 Humble）：安裝與開啟 fake」，6 個步驟、指令寫死 `humble`；說明 Humble 的
  launch log 會多幾行來自 `openarm_bringup` 設定寫法的 `[Deprecated]` 警告。
- 差異表改成實測結果：mock 怎麼知道 dip 要跟著動（Humble 讀 ros2_control 參數、Jazzy 讀 URDF，這個 repo 兩種都給、
  F4 保證一致）、mimic 關節帶 command interface 的檢查、真機 K3、記憶體鎖定、`ros2 control` 與夾爪 controller 套件。
  Humble 同時有 `fake_components` 與 `mock_components`（在容器裡查 `hardware_interface` 的 plugin 描述確認）。
- 「目前的驗證狀態」改成兩個版本都 `FULL TEST PASSED`，並註明夾爪 controller 只在 u22 容器裡驗到、接上真機的部分都還沒驗證。
- 第 3 步列出 rosdep 會裝 `ros2controlcli`；拿掉「merge 前改用 feature 分支」的說明（已 merge）。
- 「測試」一節加上 `run_in_docker.sh humble`。

## 行為變化

無，只有文件。

## 測試

README u22 第 3–6 步在 `ros:humble-ros-base` 容器（由 `run_in_docker.sh` 的 image，即照 README 安裝）裡照抄執行：

```
rosdep install ...                   →  #All required rosdeps installed successfully
colcon build --packages-select ...   →  Summary: 2 packages finished
ros2 pkg prefix openarm_description  →  /root/openarm_ws/install/openarm_description
ros2 topic hz /joint_states          →  average rate: 500.064
（送 README 的左臂、左手軌跡）       →  left_joint1/4: 0.4 0.8   L_thumb_cmc_pitch/dip: 0.6 1.374
```

當時 `ros2 control list_controllers` 失敗，因此找到 P2（12 已修，之後的 image 有 `ros2controlcli`，C1 通過）。

完整測試：u24 Jazzy、u22 Humble 都是 FULL TEST PASSED（結果同 12）。

## 後續

無。
