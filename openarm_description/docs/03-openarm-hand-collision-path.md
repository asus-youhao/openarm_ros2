# 03 openarm_hand 碰撞 mesh 路徑（K4）

## 問題

**K4**：`ee_type:=openarm_hand` 時，左右手掌（`hand`）的碰撞 mesh 路徑是
`package://openarm_descriptions/meshes/ee/openarm_hand/collision/hand.stl`，package 名稱多了一個 `s`，
系統裡沒有這個 package，所以碰撞模型載不到。視覺模型正常，所以 RViz 看起來沒事，
但 MoveIt 的碰撞檢查、模擬器的接觸都會少掉手掌。

## 根因

`urdf/ee/openarm_hand_macro.xacro:36`：

```xml
<mesh filename="package://${description_pkg}s/meshes/ee/${ee_type}/collision/${name}.stl" .../>
```

`${description_pkg}` 後面多打了一個 `s`。同一個 macro 的視覺 mesh（第 30 行）是對的。

## 改動

- 拿掉多出來的 `s`。檔案 `meshes/ee/openarm_hand/collision/hand.stl` 本來就有匯入。
- `test/urdf_checks.py`：把 K4 從已知清單移除，之後 M1 會直接擋下這類錯誤。
- `test/golden.json`：重產。只有 6 組 openarm_hand 組合的 URDF 雜湊改變，ros2_control 摘要完全不變。

## 行為變化

- openarm_hand 的手掌有碰撞模型了（左右各一個 link）。
- 渲染後的 URDF 只差左右手各一行；在 `bimanual:=true ee_type:=openarm_hand use_fake_hardware:=false` 下比對舊版：

  ```
  741c741
  <   <mesh filename="package://openarm_descriptions/meshes/ee/openarm_hand/collision/hand.stl" .../>
  >   <mesh filename="package://openarm_description/meshes/ee/openarm_hand/collision/hand.stl" .../>
  827c827
  （右手同樣一行）
  ```
- ros2_control 不變，真機和 fake 的控制行為都不受影響。
- u22 / u24：純路徑修正，兩個版本相同。

## 測試

修正前（K4 已從已知清單移除，xacro 還沒改）：

```
urdf_checks: 148 passed, 6 failed, 18 known
  FAIL M1 [bimanual:=true ee_type:=openarm_hand use_fake_hardware:=true ...] missing mesh
          package://openarm_descriptions/meshes/ee/openarm_hand/collision/hand.stl
  （openarm_hand 的 6 組組合都一樣）
```

改完 xacro、還沒更新 golden：M1 通過，G1 在同樣 6 組失敗，訊息是
`URDF differs from golden (ros2_control summary unchanged)`，符合預期。
重產 golden 後 `golden.json` 的 diff 是 6 行 `sha256`，ros2_control 區塊沒有變。

修正後的完整測試：

```
urdf_checks: 154 passed, 0 failed, 18 known
（known：K1 13、K3 1、K5 4）
FULL TEST PASSED
```

## 後續

無。
