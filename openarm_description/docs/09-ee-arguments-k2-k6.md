# 09 末端參數檔的兩個問題（K2、K6）：分析紀錄，未修

這份只記錄分析與證據，沒有改程式。兩個都會改到模型，K6 還會改變預設末端的 TCP，修之前需要使用者確認實體尺寸。

## K2：`ee_type:=o6_left` / `o6_right` 渲染失敗

**問題**：xacro 直接失敗：

```
No such file or directory: .../share/openarm_description/urdf/ee/o6_left_arguments.xacro
```

**根因**：`urdf/robot/openarm_robot.xacro:56` 依 `ee_type` include 參數檔：

```xml
<xacro:include filename="$(find openarm_description)/urdf/ee/${ee_type}_arguments.xacro" />
```

| `ee_type` | 會找 | 實際檔名 |
|---|---|---|
| `o6` | `o6_arguments.xacro` | 有 |
| `o6_left` | `o6_left_arguments.xacro` | `o6_hand_left_arguments.xacro` |
| `o6_right` | `o6_right_arguments.xacro` | `o6_hand_right_arguments.xacro` |

這兩個檔案在 `openArm_leapHand_urdf` 的 2c09eba（2026-02-13）加入時就是這個名字，而 `openarm_robot.xacro`
判斷手的地方（163、167、233、238 行）用的是 `o6_left` / `o6_right`，所以從那時起這兩個選項就沒辦法用。

**實驗**（暫存區，把兩個檔案各複製一份成 `o6_left/right_arguments.xacro`）：

| 組合 | 結果 |
|---|---|
| `bimanual:=true ee_type:=o6_left/o6_right`（fake / 真機） | 渲染成功、`check_urdf` 通過；ros2_control 只宣告有手那一側的關節；mimic 規則（F3）也通過 |
| `bimanual:=false ee_type:=o6_left/o6_right` | 渲染成功，但單臂的 `openarm.ros2_control.xacro` 沒有 O6 分支：手的關節一個都沒宣告，反而宣告 URDF 裡沒有的 `openarm_finger_joint1`（K1） |

所以只改檔名，雙臂單手可以用；單臂 O6 還要補單臂 ros2_control 的 O6 分支（跟 K1 一起處理比較合理）。
另外，`o6_hand_*_arguments.xacro` 裡的 O6 專屬預設值（例如 `tcp_xyz="0 -0.048 -0.0112"`）就算改了檔名也不會生效，原因見 K6。

## K6：openarm_hand 的安裝位置與 TCP 被 O6 的預設值蓋掉

**問題**：`ee_type:=openarm_hand`（預設末端）渲染出來，手裝在 link7 上的位置和 TCP 都是 0：

| 關節 | 現在 | `openarm_hand_arguments.xacro` 的預設 |
|---|---|---|
| `left/right_openarm_hand_joint` | `xyz="0 0 0"` | `xyz="0 -0.0 0.1001"` |
| `openarm_left/right_hand_tcp_joint` | `xyz="0 0 0"` | `xyz="0 -0.0 0.08"` |

（比對方式：同一個 xacro，不給參數 vs 給 `xyz_ee:="0 -0.0 0.1001" tcp_xyz:="0 -0.0 0.08"`，只有這 4 個關節不同。）
也就是手往內縮了約 10 cm，TCP 少了 8 cm，RViz、MoveIt、IK 用的 TCP 位置差將近 18 cm。ros2_control 不受影響，所以真機的關節控制不變。
LEAP 手不受影響（它的 macro 不讀這些參數，比對 0 個關節不同）。

**根因**：xacro 的 `xacro:arg` 重複宣告時，**第一個宣告的預設值生效**（最小範例實測：先宣告 `"0 0 0"`、再宣告
`"0 -0.048 -0.0112"`，結果是 `"0 0 0"`）。2c09eba 在 `openarm_robot.xacro:50` 無條件 include `o6_arguments.xacro`，
排在第 56 行的 `${ee_type}_arguments.xacro` 之前，所以不管選哪個末端，`xyz_ee`、`rpy_ee`、`tcp_xyz`、`tcp_rpy`
都是 O6 的值。

這個錯誤在舊 repo 就存在；01 的匯入照原樣搬過來，`test/golden.json` 也是照這個結果建的。

**修法（待決定）**：拿掉第 50 行的無條件 include，讓每種末端只讀自己的參數檔。但 `openarm_hand_arguments.xacro`
沒有定義 `rpy_ee`（現在用的是 O6 給的 `0 0 1.57079`），要先確認 openarm_hand 正確的朝向與
`0 0 0.1001` / `0 0 0.08` 是否為正確的實體尺寸，改完用 RViz 確認位置，並更新 golden。

## 測試

只有文件。完整測試：FULL TEST PASSED（結果同 08）。
