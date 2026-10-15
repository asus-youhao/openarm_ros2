# 07 README：u24 一步一步的安裝、build、launch 與讓它動

## 問題

06 的 README 用 `$ROS_DISTRO` 同時寫 u22 和 u24，但 u22（Humble）還沒實測；而且只有 build 和 launch，
沒有前置（rosdep / colcon 怎麼裝）、沒有「起來之後怎麼讓它動」，也沒寫實際踩過的兩個坑：
同一個 `ROS_DOMAIN_ID` 開兩個 fake 會互相干擾，以及 `~/.bashrc` source 了別的 openarm 工作區時會用錯 description。

## 根因

06 是照「兩個版本都適用」寫的，u24 的使用者要自己把變數代換掉；後兩個問題是 06 之後實際使用時才遇到的
（同時開 O6 與 hand 版，兩邊的 spawner 打到同一個 `/controller_manager`，RViz 也收到另一個模型）。

## 改動

`README.md` 的「只用 fake」一節改寫成「u24（Ubuntu 24.04 + ROS 2 Jazzy）：安裝與開啟 fake」，6 個步驟，指令全部寫死 `jazzy`：

1. 前置：`python3-rosdep`、`python3-colcon-common-extensions`、`rosdep init` / `rosdep update`
2. 取得原始碼（merge 前改用 `-b feat/integrate-openarm-description`）
3. `rosdep install`（只針對 `openarm_description`、`openarm_bringup`）
4. `colcon build --packages-select openarm_description openarm_bringup`
5. `ulimit -l 64` + launch（O6 版；另列 hand 版與 `launch_rviz:=false`）；停止方式與「一次只開一個」
6. 另一個終端機確認（`ros2 pkg prefix`、`list_controllers`、`topic hz`）並送軌跡讓左臂動、讓 O6 左手握起來

另外加上 `~/.bashrc` source 其他工作區時的處理。u22 標明「還沒實測，之後補上」。

## 行為變化

無，只有文件。

## 測試

README 第 3–6 步的指令原樣在乾淨環境（只 source `/opt/ros/jazzy`、`ROS_DOMAIN_ID=96`、不開 RViz）的全新副本上執行：

```
rosdep install --simulate ...           →  sudo -H apt-get install -y ros-jazzy-ros2-controllers（這台只缺這個）
colcon build --packages-select ...      →  Summary: 2 packages finished
ros2 pkg prefix openarm_description     →  <副本>/install/openarm_description
ros2 control list_controllers           →  5 個全部 active
ros2 topic hz /joint_states             →  average rate: 500.020
（送 README 裡的左臂、左手軌跡，3 秒後讀 /joint_states）
left_joint1/4: 0.4 0.8   L_thumb_cmc_pitch/dip: 0.6 1.374     （1.374 = 0.6 × 2.29）
```

第 1 步的 `sudo` 指令沒有執行（這台已經裝好）。結束後沒有殘留 domain 96 的 process 或 ros2 daemon。

完整測試：FULL TEST PASSED（結果同 05）。

## 後續

- u22 / Humble 實測後，在 README 補上 u22 的步驟並更新差異表的驗證狀態。
