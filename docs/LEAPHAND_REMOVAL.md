# LEAP Hand 完整移除說明

> 分支：`feature/remove-leaphand`（自 `develop` 切出）
> 備份標籤（移除前）：`pre-leaphand-removal`
> 里程碑標籤（移除後）：`leaphand-removed`
> 日期：2026-06-26

---

## 1. 目標

將 **LEAP Hand**（16-DOF Dynamixel 靈巧手）從 `openarm_ros2` 工作區
**完整移除**：包含原始碼、ros2_control plugin、建置依賴（`dynamixel_sdk`）、
URDF、launch / config、輔助腳本與文件。

**O6 Hand（LinkerHand，11-DOF，經由 CAN）功能完整保留，不受影響。**

驗證標準：`colcon build` 全工作區編譯通過。

---

## 2. 移除範圍總覽

| 類別 | 動作 |
|------|------|
| 硬體 plugin（C++） | 刪除獨立 plugin + 從核心 `OpenArm_v10HW` 類別抽離 LEAP 控制路徑 |
| 建置依賴 | 移除 `dynamixel_sdk`（LEAP 專用，無其他使用者） |
| URDF / xacro | 刪除全部 `leap_hand/*` xacro 與 standalone urdf |
| Launch / config | 刪除 LEAP 測試 launch / controllers；清除 bimanual launch 的 LEAP spawner |
| MoveIt 設定 | 移除 `joint_limits.yaml` 內 16 個 LEAP 手指關節限制 |
| 輔助腳本 | 刪除 LEAP 專用腳本；從 GUI / 控制腳本抽離 LEAP（保留雙臂功能） |
| 文件 | 刪除孤立的 LEAP 串口測試指南；修正 IK 權重文件誤標 |

---

## 3. 已刪除的檔案（`git rm`）

**硬體 plugin**
- `openarm_hardware/include/openarm_hardware/leap_hand_hardware.hpp`
- `openarm_hardware/src/leap_hand_hardware.cpp`
- `openarm_hardware/scripts/test_leap_serial_port.py`
- `openarm_hardware/scripts/SERIAL_PORT_TEST_GUIDE.md`（其搭配腳本已刪，孤立文件）

**URDF / xacro**
- `openarm/urdf/leap_hand/leap_hand_arguments.xacro`
- `openarm/urdf/leap_hand/leap_hand_left.xacro`
- `openarm/urdf/leap_hand/leap_hand_left_arguments.xacro`
- `openarm/urdf/leap_hand/leap_hand_right.xacro`
- `openarm/urdf/leap_hand/leap_hand_right_arguments.xacro`
- `openarm/urdf/leap_hand_standalone.urdf.xacro`

**Launch / config**
- `openarm_bringup/launch/leap_hand_test.launch.py`
- `openarm_bringup/config/v10_controllers/leap_hand_test_controllers.yaml`

**腳本 / 文件**
- `LEAP_HAND_INTEGRATION.md`
- `scripts/test_leap_hand.py`
- `scripts/controller_layer/analy_leaphand_controller.py`
- `scripts/hardware_layer/analy_leaphand_joint.py`
- `scripts/controller_layer/analy_leaphand_controller/`（整個 LEAP 除錯資料目錄）
- LEAP 除錯 CSV / PNG（`scripts/hardware_layer/analye_joint/` 內 `*leap*`）

---

## 4. 核心硬體類別重構（`OpenArm_v10HW`）

LEAP 並非單純獨立 plugin，而是深度嵌入核心 `OpenArm_v10HW` 類別，需要外科式抽離。

### `include/openarm_hardware/v10_simple_hardware.hpp`
- 移除 `#include "dynamixel_sdk/dynamixel_sdk.h"`
- 移除常數 `LEAP_HAND_DOF`、成員 `has_leap_hand_`
- 移除 LEAP 控制執行緒區塊（`leap_control_thread_`、`leap_thread_running_`、
  `leap_command_mutex_`、`leap_state_mutex_`、`serial_mutex_`、
  `leap_pos_cmd_buffer_`、`leap_pos_state_buffer_`）
- 移除 LEAP CSV 除錯成員、`HealthStatus::leap_healthy`、`leap_state_filter_`
- 移除 `leap_control_loop()` 宣告與整段 Dynamixel Protocol 2.0 支援
  （位址常數、connect/disconnect/send/read 宣告、`urdf_to_leap`/`leap_to_urdf`）

### `src/v10_simple_hardware.cpp`
- `parse_config`：移除 `has_leap_hand_` 解析與設定 log
- 夾爪索引條件 `(hand_ && !has_o6_hand_ && !has_leap_hand_)` → `(hand_ && !has_o6_hand_)`
- 移除 LEAP 手指關節名稱產生、修正 `expected_joints` 與 `o6_start_idx` 算術
- 移除 LEAP 初始化、執行緒啟動/停止、`read()`/`write()` 的 LEAP 緩衝區段
- 移除四個 LEAP 函式（connect / disconnect / send / read_leap_hand_states）
- 移除 `init_kdl_dynamics` 內 LEAP tip-candidate 區塊（退回標準 `link7`）
- 移除整段 `leap_control_loop()`、`state_read_loop()` 的 LEAP 讀取、健康檢查

### `include/openarm_hardware/v10_lpf_hardware.hpp` + `src/v10_lpf_hardware.cpp`
- 移除 `leap_cmd_filter_` 成員與相關註解
- `on_init`：移除 LEAP 命令 LPF 初始化、修正 arm filter size、清除 RCLCPP_INFO LEAP 欄位
- `on_activate`：清除 LEAP 相關註解
- `write()`：移除整段 `[2] LEAP Hand` 命令 LPF 區塊、修正 O6 `o6_start` 偏移算術
  （移除 `(has_leap_hand_ ? LEAP_HAND_DOF : 0)` 等項）

---

## 5. 建置依賴移除

LEAP 為 `dynamixel_sdk` 的唯一使用者（已 grep 確認 `dynamixel`/`DXL`/`serial_mutex_`
皆為 LEAP 專用），因此安全移除：

- `openarm_hardware/CMakeLists.txt`：移除 `find_package(dynamixel_sdk REQUIRED)`
  與 `ament_target_dependencies` 內的 `dynamixel_sdk`
- `openarm_hardware/package.xml`：移除 `<depend>dynamixel_sdk</depend>`
- `openarm_hardware/openarm_hardware.xml`：移除 `LeapHandHardware` plugin 宣告，
  並從 `OpenArm_v10LPF_HW` 描述移除 "LEAP Hand"

---

## 6. Launch / config 清理

- `openarm_bringup/launch/openarm.bimanual.launch.py`：移除
  `conditional_hand_controller_spawner`（僅在 `ee_type==leap_hand_right` 觸發的
  `right_hand_controller` spawn）、其 `TimerAction` 與 LaunchDescription 中的引用
- `openarm_bringup/launch/openarm_o6_bimanual.launch.py`：移除描述字串中提及
  LEAP 外部 URDF 路徑的句子
- `openarm_bimanual_moveit_config/config/joint_limits.yaml`：移除 16 個 LEAP
  右手手指關節限制（index / middle / ring / thumb）
- `.gitignore`：移除 `scripts/controller_layer/analy_leaphand_controller/*`

---

## 7. 輔助腳本與文件

- GUI / 控制腳本（`bimanual_gui_controller_{topic,action,hybrid}.py`、
  `action_chunk/action_chunk_controller.py`、`gr00t_state_publisher.py`）：
  抽離 LEAP（即腳本中的 "right_hand" 16-DOF 手指、`right_leaphand` controller、
  `/right_hand_forward_position_controller`），**保留左右雙臂功能**且可正常執行
- `scripts/CONTROLLER_INTERFACES.md`、`scripts/o6_test/O6_HAND_TESTING.md`：清除 LEAP 段落
- IK 權重文件（`placo_ik/docs/ik_solver_weights.md`、
  `scripts/ik_reachability/ik_controllers/README_ik_solver_weights.md`）：
  修正誤標——`q[7:18]`/`q[25:36]` 的 11-joint 手部實為 **O6 Hand**，原誤寫為 "LEAP Hand"

### 刻意保留項目
- 各腳本中的檔案系統路徑字串 `/home/asus/openArm_leapHand_urdf/...`：
  此為使用者磁碟上 URDF 套件的實際路徑（資料夾恰好以 leapHand 命名），
  **非 LEAP 程式碼或依賴**，更動會使腳本找不到 URDF，故保留原狀。
- 歷史性參考文件（`docs/VLA_CONTROL_SYSTEM_REPORT.md`、
  `docs/HARDWARE_BRINGUP_WORKFLOW_ETHERCAT.md`、`docs/code_flow.drawio.svg`）：
  描述當時系統架構的歷史紀錄，保留以供回溯。

---

## 8. 驗證

```bash
colcon build --packages-up-to openarm_hardware openarm_bringup openarm_bimanual_moveit_config
# Summary: 5 packages finished — 全數編譯通過（含重構後的 openarm_hardware）
```

---

## 9. 還原方式

若需取回 LEAP 程式碼：

```bash
git checkout pre-leaphand-removal      # 移除前的完整狀態
```
