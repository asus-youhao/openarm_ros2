# ROS 2 packages for OpenArm robots

ROS 2 (Robot Operating System 2) is a modern, open-source framework for building robotic software: https://www.ros.org/

This repository provides ROS 2 integration packages. See the [documentation](https://docs.openarm.dev/software/ros2/install) for details.

## 安裝與 fake 模式（Ubuntu 22.04 / 24.04）

URDF（`openarm_description`）已經整合進這個 repo，**不需要再 clone `openArm_leapHand_urdf`**。

| 系統 | ROS 2 |
|---|---|
| Ubuntu 22.04（u22） | Humble |
| Ubuntu 24.04（u24） | Jazzy |

### u24（Ubuntu 24.04 + ROS 2 Jazzy）：安裝與開啟 fake

fake 模式只需要 `openarm_description` 和 `openarm_bringup`。`openarm_hardware`、`openarm_can`、LinkerHand SDK
都不需要；`openarm_hardware` 沒有 LinkerHand SDK 會編不過，所以只編這兩個 package。
u22 的步驟見下一節，只差在 `jazzy` 換成 `humble`。

**1. 前置（每台機器做一次）**：已裝好 ROS 2 Jazzy（`ros-jazzy-ros-base` 或 `ros-jazzy-desktop`）。

```bash
sudo apt install python3-rosdep python3-colcon-common-extensions
sudo rosdep init        # 這台第一次用 rosdep 才需要
rosdep update
```

**2. 取得原始碼**

```bash
mkdir -p ~/openarm_ws/src && cd ~/openarm_ws/src
git clone -b develop https://github.com/asus-youhao/openarm_ros2.git
```

**3. 安裝依賴**

```bash
cd ~/openarm_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/openarm_ros2/openarm_description src/openarm_ros2/openarm_bringup --ignore-src -y
```

會裝 `xacro`、`robot_state_publisher`、`rviz2`、`joint_state_publisher_gui`、`controller_manager`、
`ros2_controllers`（含夾爪 controller）、`ros2controlcli`（`ros2 control` 指令）等，不會裝 Gazebo 或 RealSense。

**4. Build**

```bash
cd ~/openarm_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select openarm_description openarm_bringup
```

**5. Launch fake（會一起開 RViz）**

```bash
source /opt/ros/jazzy/setup.bash
source ~/openarm_ws/install/setup.bash
ulimit -l 64
ros2 launch openarm_bringup openarm_o6_bimanual.launch.py use_fake_hardware:=true
```

其他變化：

```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py use_fake_hardware:=true                    # 雙臂 + openarm_hand 夾爪
ros2 launch openarm_bringup openarm_o6_bimanual.launch.py use_fake_hardware:=true launch_rviz:=false   # 不開 RViz
```

停止：在 launch 的終端機按 Ctrl+C。同一個 `ROS_DOMAIN_ID` 上一次只開一個，兩個 fake 同時開會搶同一個
`/controller_manager` 和 `/robot_description`，controller 會互相干擾、RViz 也會混到另一個模型。

**6. 確認與讓它動**（另開一個終端機）

```bash
source /opt/ros/jazzy/setup.bash
source ~/openarm_ws/install/setup.bash
ros2 pkg prefix openarm_description      # 應該是 ~/openarm_ws/install/openarm_description
ros2 control list_controllers
ros2 topic hz /joint_states
```

| launch | controller（全部 `active`） | `/joint_states` |
|---|---|---|
| `openarm_o6_bimanual.launch.py` | `joint_state_broadcaster`、`left/right_joint_trajectory_controller`、`left/right_hand_controller` | 36 個關節，約 500 Hz |
| `openarm.bimanual.launch.py` | `joint_state_broadcaster`、`left/right_joint_trajectory_controller`、`left/right_gripper_controller` | 16 個關節，約 100 Hz |

送一段軌跡給左臂（2 秒到位），RViz 裡左臂會動：

```bash
ros2 topic pub --once -w 1 /left_joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: [openarm_left_joint1, openarm_left_joint2, openarm_left_joint3, openarm_left_joint4, openarm_left_joint5, openarm_left_joint6, openarm_left_joint7], points: [{positions: [0.4, 0.3, 0.0, 0.8, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]}"
```

O6 版可以再讓左手握起來；dip 會依 URDF 倍率自動跟著動（例如 `L_thumb_dip` = 2.29 × `L_thumb_cmc_pitch`）：

```bash
ros2 topic pub --once -w 1 /left_hand_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: [L_thumb_cmc_yaw, L_thumb_cmc_pitch, L_index_mcp_pitch, L_middle_mcp_pitch, L_ring_mcp_pitch, L_pinky_mcp_pitch], points: [{positions: [0.5, 0.6, 1.0, 1.0, 1.0, 1.0], time_from_start: {sec: 2}}]}"
```

右臂、右手把 topic 換成 `right_...`、關節名稱換成 `openarm_right_joint1..7` / `R_...`。

**為什麼要 `ulimit -l 64`**：Jazzy 的 controller_manager 預設 `lock_memory: true`。memlock 上限大到 mlockall 會成功、
但又不是無限時（例如設成 2 GB 的 RT 機器），之後的記憶體配置會失敗，controller 會報 `Resource temporarily unavailable`。
把上限調小之後只會印一行 `Unable to lock the memory` 警告，其餘照常。fake 不需要鎖記憶體。

**`~/.bashrc` 有 source 其他 openarm 工作區時**：一定要最後 source `~/openarm_ws/install/setup.bash`，並用
`ros2 pkg prefix openarm_description` 確認用到的是這一份。

### u22（Ubuntu 22.04 + ROS 2 Humble）：安裝與開啟 fake

步驟跟 u24 一樣，只是 `jazzy` 換成 `humble`。已在 `ros:humble-ros-base` 容器裡照這些指令實測過（見下方「測試」）。

**1. 前置（每台機器做一次）**：已裝好 ROS 2 Humble（`ros-humble-ros-base` 或 `ros-humble-desktop`）。

```bash
sudo apt install python3-rosdep python3-colcon-common-extensions
sudo rosdep init        # 這台第一次用 rosdep 才需要
rosdep update
```

**2. 取得原始碼**

```bash
mkdir -p ~/openarm_ws/src && cd ~/openarm_ws/src
git clone -b develop https://github.com/asus-youhao/openarm_ros2.git
```

**3. 安裝依賴**

```bash
cd ~/openarm_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/openarm_ros2/openarm_description src/openarm_ros2/openarm_bringup --ignore-src -y
```

**4. Build**

```bash
cd ~/openarm_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select openarm_description openarm_bringup
```

**5. Launch fake（會一起開 RViz）**

```bash
source /opt/ros/humble/setup.bash
source ~/openarm_ws/install/setup.bash
ulimit -l 64
ros2 launch openarm_bringup openarm_o6_bimanual.launch.py use_fake_hardware:=true
```

**6. 確認與讓它動**：跟 u24 第 6 步的指令完全一樣（另開終端機時 `source /opt/ros/humble/setup.bash`）。

Humble 的 launch log 會多幾行 `[Deprecated]` 警告（例如 robot description 參數、`allow_nonzero_velocity_at_trajectory_end`），
來自 `openarm_bringup` 的設定寫法，不影響 fake。

### u22 與 u24 的差異

同一個分支、同一份檔案，兩個版本都能用。差異都在 ros2_control 的處理方式：

| 項目 | u22 / Humble | u24 / Jazzy | 這個 repo 的做法 |
|---|---|---|---|
| fake 的硬體 plugin | `mock_components`（舊名 `fake_components` 也還在） | 只有 `mock_components` | 一律用 `mock_components/GenericSystem`、參數 `mock_sensor_commands` |
| mock 怎麼知道 O6 dip 要跟著動 | 讀 ros2_control 關節裡的 `<param name="mimic">` / `multiplier` | 讀 URDF 的 `<mimic>` | fake 模式的 dip 兩種都有，而且由測試（F4）保證一致；dip 只宣告 state interface |
| URDF mimic 關節帶 command interface | 不檢查 | 解析時 abort | fake 的 dip 不帶 command interface |
| 真機模式的 O6 dip（K3） | 忽略 `mimic` 屬性 | **需要** `mimic="false"` 才能通過解析 | 保留 command interface，加 `mimic="false"`，由 `OpenArm_v10HW` 自己驅動 dip |
| controller_manager 記憶體鎖定 | 一樣加 `ulimit -l 64`，無害 | 預設 `lock_memory: true` | 啟動前 `ulimit -l 64` |
| `ros2 control` 指令、夾爪 controller | `ros-humble-ros2controlcli`、`ros-humble-gripper-controllers` | `ros-jazzy-ros2controlcli`、`ros-jazzy-gripper-controllers` | 都由 rosdep 依 `package.xml` 裝好 |

目前的驗證狀態（2026-10-15）：u24 / Jazzy 在本機、u22 / Humble 在 `ros:humble-ros-base` 容器裡，完整測試都是
`FULL TEST PASSED`：兩個版本渲染出來的 URDF 完全相同；O6 fake 5 個 controller active、36 個關節約 500 Hz、
手臂與手照軌跡動、dip 依倍率跟隨；openarm_hand 版在 u22 容器裡 5 個 controller（含夾爪）active（u24 這台沒裝夾爪
controller，所以只驗到手臂的 3 個）；真機模式的 O6 URDF 能通過解析。
接上真機的部分兩個版本都還沒驗證。

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

在 u24 機器上測 u22（Humble），用 Docker 跑同一套測試（第一次會建 image，需要網路）：

```bash
~/openarm_ws/src/openarm_ros2/openarm_description/test/run_in_docker.sh humble
```
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
