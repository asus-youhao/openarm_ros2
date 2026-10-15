# 06 README：安裝、fake 使用流程、u22 / u24 差異

## 問題

openarm_ros2 的 README 只有上游的英文簡介，沒有說明：

- description 已經整合進來，不用再 clone `openArm_leapHand_urdf`；
- 只用 fake 時要裝什麼、編哪些 package（整包編會卡在沒有 LinkerHand SDK 的 `openarm_hardware`）；
- 啟動 fake 的指令，以及起來之後應該看到什麼；
- u22（Humble）和 u24（Jazzy）的差異，以及真機 O6 選擇加 `mimic="false"` 的原因；
- 舊工作區要先移除另一份 `openarm_description`，否則 colcon 會因為 package 名稱重複而停止；
- 怎麼跑完整測試。

## 根因

這些都是這次整合（01–05）才產生或才確認的事，README 還沒跟上。

## 改動

- `README.md`（openarm_ros2 根目錄）新增「安裝與 fake 模式（Ubuntu 22.04 / 24.04）」一節：
  - **只用 fake**：clone、`rosdep install`（只針對 `openarm_description`、`openarm_bringup`）、`colcon build --packages-select`。
  - **啟動**：`ulimit -l 64` 加上兩支 launch，並附上預期的 controller 與 `/joint_states` 表格，以及檢查指令。
  - **u22 / u24 差異表**：fake plugin、URDF mimic、fake 與真機的 O6 dip、記憶體鎖定、夾爪 controller 套件；
    並明確標出 Humble 欄位目前是推論、尚未實測。
  - **真機 K3 的選擇**：加 `mimic="false"`，Jazzy 需要、Humble 忽略，待上機驗證。
  - **從舊 repo 搬過來**、**測試方式**。
- `openarm_description/README.md`：說明這個 package 的來源與只保留的內容、只看模型的指令，以及到根目錄 README、
  `CLAUDE.md`、`docs/` 的連結；修正原本指向 package 內並不存在的 `CODE_OF_CONDUCT.md` 的連結。

## 行為變化

無，只有文件。

## 測試

照 README 的步驟，從這條分支全新 clone 到暫存工作區，用乾淨的環境（只 source `/opt/ros/jazzy`）執行：

```
rosdep install --simulate ...   →  sudo -H apt-get install -y ros-jazzy-ros2-controllers   （這台只缺這個）
colcon build --packages-select openarm_description openarm_bringup   →  Summary: 2 packages finished
colcon list --packages-select openarm_description   →  src/openarm_ros2/openarm_description（只有一筆）
ros2 launch openarm_description display_openarm.launch.py --show-args   →  正常列出參數
ulimit -l 64
openarm_o6_bimanual.launch.py use_fake_hardware:=true
  joint_state_broadcaster / left,right_joint_trajectory_controller / left,right_hand_controller 全部 active
  /joint_states average rate: 500.042
openarm.bimanual.launch.py use_fake_hardware:=true
  joint_state_broadcaster / left,right_joint_trajectory_controller active
  /joint_states average rate: 99.999
```

結束後沒有殘留的 `ros2_control_node`。（原本這裡寫「也沒有殘留 ros2 daemon」是錯的，見 08 的更正。）

這台沒有裝 `ros-jazzy-ros2-controllers`（rosdep 顯示會補裝），所以 `openarm.bimanual.launch.py` 的
`left/right_gripper_controller` 沒有起來；README 表格列的是照 README 用 rosdep 裝好之後應有的狀態，
這一點在這台機器上沒有驗證。

完整測試：FULL TEST PASSED（結果同 05）。

## 後續

- u22 / Humble 實測（容器或 u22 機器）後，更新 README 的「目前的驗證狀態」與差異表。
- README 的 clone 指令用 `-b develop`；這條分支 merge 進 `develop` 之後才會正確。
