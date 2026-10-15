# openarm_description 整合紀錄（`feat/integrate-openarm-description`）

把 `asus-youhao/openArm_leapHand_urdf` 的 `src/openarm_description` 整合進 openarm_ros2，
只帶 URDF 需要的東西，並讓它在 Ubuntu 22.04（ROS 2 Humble）與 24.04（ROS 2 Jazzy）都能用。

- 分支起點：openarm_ros2 `origin/develop` `c47a870`
- 匯入來源：`openArm_leapHand_urdf` 的 `fix/jazzy-fake-hardware` `6b8d6a7`（已含 J1–J3 的修正，細節見該分支的 `docs/jazzy_fake/`）
- 只在本機，沒有 push

## 文件索引

| # | 文件 | 內容 |
|---|---|---|
| 01 | [01-import.md](01-import.md) | 匯入；只帶需要的檔案；測試框架、golden 與 mesh 檢查；記錄 K4、K5 |
| 02 | [02-package-deps.md](02-package-deps.md) | P1：`package.xml` 依賴對齊實際使用，Humble / Jazzy 都可安裝 |
| 03 | [03-openarm-hand-collision-path.md](03-openarm-hand-collision-path.md) | K4：openarm_hand 碰撞 mesh 路徑 |
| 04 | [04-o6-mimic-real-hardware.md](04-o6-mimic-real-hardware.md) | K3：真機 O6 dip 標 `mimic="false"`，Jazzy 真機能過 URDF 解析；新增 H1 |
| 05 | [05-bimanual-launch-default-ee.md](05-bimanual-launch-default-ee.md) | X1：`openarm.bimanual.launch.py` 預設 `ee_type` 改成 `openarm_hand` |
| 06 | [06-readme-install-fake.md](06-readme-install-fake.md) | README：安裝、fake 使用流程、u22 / u24 差異、K3 說明 |

## 問題 ID 對照

| ID | 問題 | 狀態 |
|---|---|---|
| J1 | fake 模式的 plugin 是 Jazzy 沒有的 `fake_components/GenericSystem` | 匯入前已在舊 repo 修好 |
| J2 | fake 模式傳 `fake_sensor_commands`，Jazzy 的 mock 只讀 `mock_sensor_commands` | 匯入前已在舊 repo 修好 |
| J3 | fake 模式的 O6 dip（URDF mimic）帶 command interface，Jazzy 解析時 abort | 匯入前已在舊 repo 修好 |
| P1 | `package.xml` 宣告了沒用到的 `ros_gz`（整套 Gazebo）、`realsense2_description`，卻沒宣告實際用到的 `robot_state_publisher` | 已修（02） |
| K1 | `ee_type` 不是 `openarm_hand` 時，ros2_control 仍宣告 URDF 裡沒有的 `finger_joint1` | 已知，未修 |
| K2 | `ee_type:=o6_left` / `o6_right` 渲染失敗：`o6_left_arguments.xacro` 不存在 | 已知，未修 |
| K3 | 真機模式的 O6 dip 仍帶 command interface，Jazzy 上真機會在解析時 abort | 已修（04），待上機驗證 |
| K4 | openarm_hand 的碰撞 mesh 路徑是 `package://openarm_descriptions/`（多一個 s），載不到 | 已修（03） |
| K5 | `leap_hand_left` 用 `meshes/leap_hand/visual/mcp_joint_left.obj`，這個檔案在新舊 repo 都不存在 | 已知，不猜幾何，未修 |
| X1 | `openarm.bimanual.launch.py` 預設 `ee_type:=default`，沒有對應的 xacro（單臂 `openarm.launch.py` 沒有這個參數，不受影響） | 已修（05） |
| X2 | Jazzy 的 controller_manager 預設 `lock_memory: true`；memlock 上限大到 mlockall 會成功、但又不是無限時，之後配置會失敗（`Resource temporarily unavailable`）。用 `ulimit -l 64` 避開 | 執行環境，只記錄 |
| X3 | `openarm.bimanual.launch.py` 的夾爪 controller 需要 apt 套件 `ros-<distro>-gripper-controllers` | 執行環境，只記錄 |
