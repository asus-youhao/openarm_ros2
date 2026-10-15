# 04 真機模式的 O6 dip 標 `mimic="false"`（K3）

## 問題

**K3**：`openarm_o6_bimanual.launch.py use_fake_hardware:=false` 在 Jazzy 上，ros2_control_node 一解析 URDF 就 abort，
跟 fake 模式的 J3 是同一個錯誤，發生在載入硬體 plugin 之前：

```
[ros2_control_node-2]   what():  Joint 'L_thumb_dip' has mimic attribute not set to false:
                                 Activated mimic joints cannot have command interfaces.
```

也就是說，用這份 description，O6 雙臂的真機在 u24（Jazzy）上起不來。

## 根因

O6 每隻手 5 個 `*_dip` 在 URDF 是 `<mimic>` 關節。真機模式的 ros2_control 用 `configure_joint` 宣告它們，
帶 position / velocity / effort 三個 command interface。Jazzy 解析 URDF 時，遇到「URDF 是 mimic、
又有 command interface、而且沒標 `mimic="false"`」的關節會直接丟例外。Humble 沒有這個檢查。

真機模式不能跟 fake 一樣改成只有 state interface：`OpenArm_v10HW` 自己會匯出這些關節的 command interface、
自己算 dip 的位置（openarm_ros2 `c47a870` `openarm_hardware/src/v10_simple_hardware.cpp:170-174`、`:1397-1401`），
URDF 沒宣告它卻匯出，Jazzy 的 ResourceManager 怎麼處理沒有驗證過。

## 改動

使用者選定做法：真機模式的 dip 保留跟原本一樣的 interface，加上 `mimic="false"`，告訴 ros2_control
「這個關節不要當 mimic 處理，由硬體自己負責」。

- `urdf/ros2_control/openarm.bimanual.ros2_control.xacro`：`configure_mimic_joint` 的真機分支不再呼叫
  `configure_joint`，改成直接寫出同樣的 joint（3 個 command、3 個 state interface、同樣的初始值），加上
  `mimic="false"`。不用 `xacro:attribute` 是因為它放在 `xacro:if` 裡不會生效（xacro 2.1.1 實測，屬性被丟掉）。
  interface 寫在兩處，但 G1 的摘要會逐一比對每個關節的 interface，任何一邊抄錯都會被抓到。
- `test/urdf_checks.py`：F3 改成真機模式也把關（拿掉 K3 已知）。
- `test/fake_smoke.py`：新增 **H1** case `real_o6_parse_probe`：在**沒有安裝 `openarm_hardware`** 的環境啟動
  `openarm_o6_bimanual.launch.py use_fake_hardware:=false launch_rviz:=false`，要看到 launch 停在
  「載入 `openarm_hardware/OpenArm_v10HW` 失敗」，而不是 mimic 錯誤。環境裡有裝 `openarm_hardware` 時
  H1 直接 SKIP，**絕不會啟動真機**。
- `test/golden.json`：重產。只有 `bimanual:=true ee_type:=o6 use_fake_hardware:=false` 這一組改變。

## 行為變化

- **u24（Jazzy）**：真機 O6 雙臂的 URDF 能通過 ros2_control 解析了，接下來才會去載入 `OpenArm_v10HW`。
- **u22（Humble）**：Humble 的解析器不讀 `mimic` 屬性，行為跟修改前相同（推論，Humble 驗證在後面的批次）。
- 真機的 dip 跟以前一樣有 command interface，由 `OpenArm_v10HW` 自己驅動與回報；hand controller 本來就只控制
  6 個主動關節，不受影響。
- fake 模式不變（fake 的 dip 早就只有 state interface）。
- **需要上機驗證**：這個改動讓真機在 Jazzy 上能進到載入 plugin 這一步，但之後 `OpenArm_v10HW` 在 Jazzy 上的完整行為
  （含這 10 個關節）只能上機確認。

ros2_control 摘要的差異（測試輸出，左手；右手同樣 5 行）：

```
-  joint L_thumb_dip cmd=position,velocity,effort state=position,velocity,effort
+  joint L_thumb_dip mimic=false cmd=position,velocity,effort state=position,velocity,effort
-  joint L_index_dip cmd=position,velocity,effort state=position,velocity,effort
+  joint L_index_dip mimic=false cmd=position,velocity,effort state=position,velocity,effort
   （middle、ring、pinky 同樣）
```

golden.json 的 ros2_control 區塊用內容雜湊當 key，所以檔案 diff 是整段區塊換掉（54 行），逐行的差異如上。

## 測試

修正前（F3 / H1 已加入、xacro 還沒改）：

```
urdf_checks: 154 passed, 1 failed, 17 known
  FAIL F3 [bimanual:=true ee_type:=o6 use_fake_hardware:=false ros2_control:=true] mimic joints with command interfaces:
          ['L_index_dip', 'L_middle_dip', 'L_pinky_dip', 'L_ring_dip', 'L_thumb_dip', 'R_index_dip', ...]
[real_o6_parse_probe] openarm_o6_bimanual.launch.py use_fake_hardware:=false launch_rviz:=false
  FAIL H1 ros2_control rejected the URDF: Activated mimic joints cannot have command interfaces
FULL TEST FAILED (2): urdf checks, fake smoke
```

修正後的完整測試：

```
urdf_checks: 155 passed, 0 failed, 17 known
[bimanual_openarm_hand]  SKIP S1 gripper / PASS S1 3 active / S2 16 joints at 100 Hz / S3 14 joints reached target
[bimanual_o6]            PASS S1 5 active / S2 36 joints at 501 Hz / S3 26 joints reached target / S4 10 mimic joints follow
[real_o6_parse_probe]    PASS H1 URDF parsed; stopped at loading openarm_hardware/OpenArm_v10HW (not installed)
FULL TEST PASSED
（known：K1 13、K5 4）
```

H1 的 launch log：`LibraryLoadExceptionE while loading hardware: According to the loaded plugin descriptions the class
openarm_hardware/OpenArm_v10HW with base class type hardware_interface::SystemInterface does not exist.`

## 後續

- 上機驗證（u24 + O6 雙臂）：bringup 能啟動、手的 6 個主動關節照指令動、`/joint_states` 裡 dip 的值合理。
- `OpenArm_v10HW` 的左拇指 dip 倍率寫死 1.86，URDF 是 2.29；openarm_ros2 的 `fix/hw-bringup-safety`（`604e4e1`）已經改成依 URDF。
