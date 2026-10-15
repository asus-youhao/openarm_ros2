# ROS 2 packages for OpenArm robots

ROS 2 (Robot Operating System 2) is a modern, open-source framework for building robotic software: https://www.ros.org/

This repository provides ROS 2 integration packages. See the [documentation](https://docs.openarm.dev/software/ros2/install) for details.

## 安裝與 fake 模式（Ubuntu 22.04 / 24.04）

URDF（`openarm_description`）已經整合進這個 repo，**不需要再 clone `openArm_leapHand_urdf`**。

| 系統 | ROS 2 |
|---|---|
| Ubuntu 22.04（u22） | Humble |
| Ubuntu 24.04（u24） | Jazzy |

### 只用 fake（不接硬體）

fake 模式只需要 `openarm_description` 和 `openarm_bringup`。`openarm_hardware`、`openarm_can`、LinkerHand SDK
都不需要；`openarm_hardware` 沒有 LinkerHand SDK 會編不過，所以只編這兩個 package。
下面的 `ROS_DISTRO` 在 u22 是 `humble`，在 u24 是 `jazzy`。

```bash
mkdir -p ~/openarm_ws/src && cd ~/openarm_ws/src
git clone -b develop https://github.com/asus-youhao/openarm_ros2.git
cd ~/openarm_ws
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src/openarm_ros2/openarm_description src/openarm_ros2/openarm_bringup --ignore-src -y
colcon build --packages-select openarm_description openarm_bringup
source install/setup.bash
```

`rosdep install` 會裝 `xacro`、`robot_state_publisher`、`rviz2`、`controller_manager`、`ros2_controllers`（含夾爪 controller）等，
不會裝 Gazebo 或 RealSense。

啟動（同一個終端機）：

```bash
ulimit -l 64
ros2 launch openarm_bringup openarm_o6_bimanual.launch.py use_fake_hardware:=true    # 雙臂 + O6 手
ros2 launch openarm_bringup openarm.bimanual.launch.py use_fake_hardware:=true       # 雙臂 + openarm_hand 夾爪（預設 ee_type）
```

起來之後應該看到：

| launch | controller（全部 `active`） | `/joint_states` |
|---|---|---|
| `openarm_o6_bimanual.launch.py` | `joint_state_broadcaster`、`left/right_joint_trajectory_controller`、`left/right_hand_controller` | 36 個關節，約 500 Hz |
| `openarm.bimanual.launch.py` | `joint_state_broadcaster`、`left/right_joint_trajectory_controller`、`left/right_gripper_controller` | 16 個關節，約 100 Hz |

```bash
ros2 control list_controllers
ros2 topic hz /joint_states
```

**為什麼要 `ulimit -l 64`**：Jazzy 的 controller_manager 預設 `lock_memory: true`。memlock 上限大到 mlockall 會成功、
但又不是無限時（例如設成 2 GB 的 RT 機器），之後的記憶體配置會失敗，controller 會報 `Resource temporarily unavailable`。
把上限調小之後只會印一行 `Unable to lock the memory` 警告，其餘照常。fake 不需要鎖記憶體；在 Humble 上這行也無害。

### u22 與 u24 的差異

同一個分支、同一份檔案，兩個版本都能用。差異都在 ros2_control 的處理方式：

| 項目 | u22 / Humble | u24 / Jazzy | 這個 repo 的做法 |
|---|---|---|---|
| fake 的硬體 plugin | `mock_components` 與舊名 `fake_components` 都有 | 只有 `mock_components` | 一律用 `mock_components/GenericSystem`、參數 `mock_sensor_commands` |
| URDF 的 `<mimic>` 關節（O6 每隻手 5 個 `*_dip`） | ros2_control 不讀 URDF 的 mimic | 會讀；mimic 關節不能有 command interface，否則解析時 abort | 見下兩列 |
| fake 模式的 O6 dip | 只有 state interface，dip 停在 0 | 只有 state interface，mock 依 URDF 倍率自動算出 dip | dip 只宣告 state interface |
| 真機模式的 O6 dip（K3） | 忽略 `mimic` 屬性，行為不變 | **需要** `mimic="false"` 才能通過解析 | 保留 command interface，加 `mimic="false"`，由 `OpenArm_v10HW` 自己驅動 dip |
| controller_manager 記憶體鎖定 | — | 預設 `lock_memory: true` | 啟動前 `ulimit -l 64` |
| 夾爪 controller 套件 | `ros-humble-gripper-controllers` | `ros-jazzy-gripper-controllers` | 由 `ros2_controllers` 帶進來 |

目前的驗證狀態：u24 / Jazzy 已用完整測試驗證（見下方「測試」）；u22 / Humble 的編譯與啟動**尚未實測**，
表中 Humble 欄位是依 ros2_control 在兩個版本間的差異推論。

O6 dip 的連動倍率（URDF）：`L_thumb_dip` = 2.29 × `L_thumb_cmc_pitch`、`R_thumb_dip` = 1.86 × `R_thumb_cmc_pitch`，
其他四指 = 0.89 × 同一指的 `*_mcp_pitch`。

**真機 K3 的選擇**：真機模式的 O6 dip 選擇「加 `mimic="false"`」。Jazzy 需要這個屬性，否則 `ros2_control_node`
會在解析 URDF 時直接 abort（`Activated mimic joints cannot have command interfaces`）；Humble 會忽略它。
dip 仍由 `OpenArm_v10HW` 自己匯出、驅動與回報。這個改動已確認 Jazzy 真機模式的 URDF 能通過解析，
**接上真機的完整行為還需要上機驗證**。細節見 [openarm_description/docs/04-o6-mimic-real-hardware.md](openarm_description/docs/04-o6-mimic-real-hardware.md)。

### 從 `openArm_leapHand_urdf` 搬過來

舊的工作區如果還有另一份 `openarm_description`（例如 `src/openarm_description` 指向 `openArm_leapHand_urdf`），
要先移除，否則 colcon 會因為 package 名稱重複而停止：

```bash
colcon list --packages-select openarm_description
```

應該只列出 `src/openarm_ros2/openarm_description` 一筆。

### 測試

```bash
~/openarm_ws/src/openarm_ros2/openarm_description/test/run_full_test.sh
```

腳本會用乾淨的環境（只 source `/opt/ros/<distro>`）編譯 `openarm_description` 與 `openarm_bringup`，跑 lint、
`package.xml` 依賴檢查、URDF 檢查（24 組參數），並實際啟動 fake 確認 controller、關節與動作，最後要出現 `FULL TEST PASSED`。
機器上同時有好幾個 ROS 版本時，用 `OPENARM_ROS_DISTRO=humble` 或 `jazzy` 指定。
檢查項目與每個修正的紀錄在 [openarm_description/docs/](openarm_description/docs/README.md)。

## Related links

- 📚 Read the [documentation](https://docs.openarm.dev/software/ros2/install)
- 💬 Join the community on [Discord](https://discord.gg/FsZaZ4z3We)
- 📬 Contact us through <openarm@enactic.ai>

## License

[Apache License 2.0](LICENSE)

Copyright 2025 Enactic, Inc.

## Code of Conduct

All participation in the OpenArm project is governed by our [Code of Conduct](CODE_OF_CONDUCT.md).
