# 11 u22（Humble）fake 模式的 O6 dip 跟著動（U1）

## 問題

**U1**：u22 / Humble 上 `openarm_o6_bimanual.launch.py use_fake_hardware:=true` 可以起來、手的主動關節會動，
但 10 個 `*_dip` 一直停在 0（10 的 Humble 完整測試，S4 失敗）：

```
FAIL S4 mimic joints (actual, expected): {'R_thumb_dip': (0.0, 0.279), 'R_index_dip': (0.0, 0.178), ...}
```

## 根因

fake 模式的 dip 只宣告 state interface，靠 mock 算出位置。兩個版本的 `mock_components/GenericSystem` 算法不同：

| | 從哪裡知道 dip 要跟誰、倍率多少 |
|---|---|
| Jazzy | URDF 的 `<mimic joint=... multiplier=...>` |
| Humble | ros2_control 關節裡的 `<param name="mimic">` 與 `<param name="multiplier">`；不讀 URDF |

原本只有 URDF 的 `<mimic>`，所以 Humble 的 mock 不知道 dip 是 mimic 關節。

## 改動

- `urdf/ros2_control/openarm.bimanual.ros2_control.xacro`：`configure_mimic_joint` 多兩個參數 `mimic_of`、`multiplier`，
  fake 分支在關節裡加上 `<param name="mimic">${mimic_of}</param>` 與 `<param name="multiplier">${multiplier}</param>`。
  20 個呼叫依 URDF 的 `<mimic>` 填值：拇指跟 `*_thumb_cmc_pitch`（左 2.29、右 1.86），其他四指跟同一指的 `*_mcp_pitch`（0.89）。
  真機分支不變。
- `test/urdf_checks.py`：
  - 新增 **F4**：fake 模式裡每個 URDF mimic 關節都要有 `mimic` / `multiplier` 參數，而且跟 URDF 的 `<mimic>` 一致。
    倍率因此寫在兩處（URDF 與 ros2_control），F4 保證它們不會不同步。
  - G1 的 ros2_control 摘要加上關節參數（`params=...`），改到這些參數時 golden 的差異看得到。
- `test/golden.json`：重產；只有 `bimanual:=true ee_type:=o6 use_fake_hardware:=true`（含 `fake_sensor_commands:=true`）兩組改變，
  各 10 行多了 `params=mimic=...,multiplier=...`。真機組合不變。

## 行為變化

- **u22 / Humble**：fake 的 O6 dip 會依倍率跟著動，跟 u24 一樣。
- **u24 / Jazzy**：行為不變（仍由 URDF 的 `<mimic>` 計算；log 裡沒有任何關於這兩個參數的警告）。
- 真機模式不變。

## 測試

修正前（F4 已加入、xacro 還沒改）：

```
urdf_checks: 167 passed, 2 failed, 17 known
  FAIL F4 [bimanual:=true ee_type:=o6 use_fake_hardware:=true ...]
          ['L_thumb_dip: params mimic=None multiplier=None, URDF L_thumb_cmc_pitch x2.29', ...]
```

Humble 的 S4 失敗見 10。

改完 xacro、還沒更新 golden：F4 通過，G1 只在上面兩組失敗（ros2_control 摘要多了參數），符合預期。

修正後：

```
u24 Jazzy（本機）：
urdf_checks: 169 passed, 0 failed, 17 known
  PASS S4 10 mimic joints follow their parent
FULL TEST PASSED

u22 Humble（run_in_docker.sh humble）：
  PASS D1 / D2 humble / D2 jazzy
urdf_checks: 169 passed, 0 failed, 17 known
[bimanual_default]    PASS S1 5 controllers active / S2 16 joints at 100 Hz / S3 14 joints reached target
[bimanual_o6]         PASS S1 5 controllers active / S2 36 joints at 501 Hz / S3 26 joints reached target / S4 10 mimic joints follow their parent
[real_o6_parse_probe] PASS H1
FULL TEST PASSED
```

## 後續

- README 補 u22 的步驟並把差異表改成實測結果（12）。
