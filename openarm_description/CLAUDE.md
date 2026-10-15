# CLAUDE.md（openarm_description）

給在 `openarm_description/` 工作的 Claude（以及人）看的工作說明。

## 這個 package 是什麼

OpenArm V10 的 URDF（xacro）、mesh 與設定。原本放在獨立的 repo `asus-youhao/openArm_leapHand_urdf`
（`src/openarm_description/`），現在整合進 openarm_ros2，舊 repo 之後會收掉。

| 路徑 | 內容 |
|---|---|
| `urdf/robot/v10.urdf.xacro` | 入口。`bimanual`、`ee_type`、`ros2_control`、`use_fake_hardware` 等參數 |
| `urdf/ros2_control/` | ros2_control 宣告。跟 `openarm_hardware` 的 `OpenArm_v10HW` 綁在一起（plugin 名稱、參數、O6 關節順序） |
| `urdf/arm`、`urdf/body`、`urdf/ee` | 模型本體；`ee/` 有 openarm_hand、O6、LEAP hand |
| `meshes/` | 只放 URDF 實際用到的 mesh（含 `.obj` 會帶到的 `.mtl`） |
| `config/` | 運動學、慣量、關節限位 |
| `launch/`、`rviz/` | 顯示用 |
| `test/` | 完整測試（不會被安裝） |
| `docs/` | 每個改動的文件 |

## 修改的工作規則

使用者指定，動到這個 package（以及為了它動到 `openarm_bringup`）都照做：

1. **每一個改動都要有一份 doc md**：`docs/NN-<主題>.md`，章節固定為
   問題 → 根因 → 改動 → 行為變化（含 u22 / u24 相容性）→ 測試（含完整測試結果）→ 後續。
   同一個 commit 裡要更新 `docs/README.md` 的文件索引與問題 ID 對照表。
2. **分批 commit**：一批一個主題。commit message 用 conventional commits（英文），內文依序寫
   問題、根因、改動、行為變化、測試結果、對應文件；結尾附 `Co-Authored-By:` trailer。
3. **每次 commit 前都跑完整測試**，必須出現 `FULL TEST PASSED`。修 bug 時先讓測試在舊版上失敗，
   修好再跑一次；修前／修後的證據寫進 doc。
4. **只在本機做**：不要 push、不要開 PR，除非使用者明確要求。
5. **真機看得到的改動要明示**：ros2_control 區塊的任何變化都會讓 G1 失敗，測試輸出會印出逐行差異。
   刻意的改動用 `--update-golden` 重產 `test/golden.json` 並放進同一個 commit（區塊以內容雜湊為 key，
   檔案 diff 會是整段換掉），逐行差異寫進 doc，並標註需要上機驗證。
6. 過程中新發現的問題寫進 doc；跟當前主題無關的記成已知問題，不要順手改。

## 完整測試

```bash
~/openarm_descint_ws/src/openarm_ros2/openarm_description/test/run_full_test.sh
```

用 `env -i` 重新執行自己，只 source `/opt/ros/<distro>`（`OPENARM_ROS_DISTRO` 可指定 humble / jazzy）。

u22（Humble）在這台 u24 上用 Docker 跑（第一次會建 image，需要網路）：

```bash
~/openarm_descint_ws/src/openarm_ros2/openarm_description/test/run_in_docker.sh humble
```

改到 xacro、launch 或測試時，u24 與 u22 都要跑；commit 的門檻是兩邊都 `FULL TEST PASSED`
（10 這一批 u22 還有已知的 U1，是唯一的例外，記錄在 doc 裡）。

| 步驟 | 內容 | 失敗條件 |
|---|---|---|
| 1 | preflight | 工作區裡的 `openarm_description` 不只一個，或不是這一份 |
| 2 | `colcon build --cmake-clean-first`：`openarm_description`、`openarm_bringup` | 編譯失敗 |
| 3 | `urdf/` 裡所有 `.xacro` / `.urdf` 跑 `xmllint --noout` | 任何一個不是合法 XML |
| 4 | 兩個 package 的 ament linters，跟 `test/lint_baseline.txt` 比 | 任何一項比基準線多 |
| 5 | `test/deps_check.py`：`package.xml` 的依賴 | D1 宣告與實際使用不一致；D2 有依賴在 Humble（jammy）或 Jazzy（noble）解析不到 |
| 6 | `test/urdf_checks.py`：24 組參數的渲染與檢查 | 見下表；K* 只回報 |
| 7 | `test/fake_smoke.py`：用 `openarm_bringup` 實際啟動 fake | 見下表 |

`urdf_checks.py`：

| ID | 內容 |
|---|---|
| R1 / R2 / R3 | xacro 渲染成功 / `check_urdf` 通過 / ros2_control 的關節都存在於 URDF |
| M1 | URDF 引用的每個 mesh 都在安裝後的 package 裡 |
| F1 / F2 | fake 模式用 `mock_components/GenericSystem`、傳 `mock_sensor_commands` |
| F3 | URDF 裡是 mimic、又沒標 `mimic="false"` 的 ros2_control 關節，不能有 command interface（Jazzy 的解析規則） |
| G1 | 跟 `test/golden.json` 一致：整份 URDF 的雜湊，加上 ros2_control 的逐行摘要 |

`fake_smoke.py`（case：`bimanual_default`（不指定 `ee_type`）、`bimanual_o6`、`real_o6_parse_probe`；每個 case 用一個沒有任何 process 在用的 `ROS_DOMAIN_ID`（60–101）、只在 localhost discovery、`ulimit -l 64`）：

| ID | 內容 |
|---|---|
| S1 | 必要的 controller 都到 `active`（夾爪 controller 只有裝了 `gripper_controllers` 才要求） |
| S2 | `/joint_states` 的關節剛好是 ros2_control 宣告的關節，頻率不低於下限 |
| S3 | 每個 JointTrajectoryController 都能把關節帶到目標 |
| S4 | URDF mimic 關節都等於「倍率 × 跟隨的關節 + offset」 |
| H1 | 真機模式（`use_fake_hardware:=false`）的 URDF 能通過 ros2_control 解析，停在載入硬體 plugin。只在**沒有**安裝 `openarm_hardware` 時執行，有裝就 SKIP，不會啟動真機 |

工作區配置（這台 NB）：測試工作區 `~/openarm_descint_ws`，`src/openarm_ros2` 是 openarm_ros2 的 worktree
（分支 `feat/integrate-openarm-description`）。主 repo `~/openarm_ws/src/openarm_ros2` 停在 `feature/placo_ik`，不要動。

## 不變式

- 匯入後的 package 渲染出來的 URDF，跟 `openArm_leapHand_urdf@6b8d6a7` 完全一樣，除非 golden 有明示的更新。（G1）
- URDF 引用的 mesh 都存在，已知問題除外。（M1）
- `package.xml` 的執行期依賴剛好是實際用到的 package，且在 Humble 與 Jazzy 都解析得到。（D1、D2）
- fake 模式只用 `mock_components`；兩種模式都沒有「未標 `mimic="false"` 的 mimic 關節」帶 command interface。（F1–F3）
- 真機模式的 O6 URDF 能通過 Jazzy 的 ros2_control 解析。（H1）
- O6 雙臂與預設（openarm_hand）雙臂的 fake 都能起來、會照軌跡動，O6 dip 照 URDF 倍率跟隨。（S1–S4）
