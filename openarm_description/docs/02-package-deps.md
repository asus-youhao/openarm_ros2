# 02 `package.xml` 依賴對齊實際使用（P1）

## 問題

**P1**：`package.xml` 宣告的依賴跟 package 實際用到的不一致：

- 宣告了 `ros_gz` 和 `realsense2_description`，但 URDF 和 launch 都沒用到。`rosdep install` 會因此裝進整套
  Gazebo（`ros-<distro>-ros-gz`）和 RealSense 的模型，在 u22 / u24 新機器上白白多裝一大堆東西。
- `launch/display_openarm.launch.py` 會起 `robot_state_publisher`，卻沒有宣告；只是剛好大部分機器都有裝。
- `joint_state_publisher_gui` 用 `<depend>` 宣告，但它只在執行期需要。

## 根因

宣告是從上游範本留下來的，之後 URDF 的 Gazebo / RealSense 部分拿掉（現在只剩註解），依賴沒有跟著整理；
`robot_state_publisher` 則是一開始就漏掉。

## 改動

`package.xml` 的執行期依賴改成剛好是用到的四個，全部用 `<exec_depend>`：

```xml
<exec_depend>joint_state_publisher_gui</exec_depend>
<exec_depend>robot_state_publisher</exec_depend>
<exec_depend>rviz2</exec_depend>
<exec_depend>xacro</exec_depend>
```

測試：新增 `test/deps_check.py`，完整測試第 5 步執行。

- **D1**：`<depend>` / `<exec_depend>` 必須剛好等於實際用到的 package：launch 裡的 `Node(package=...)`、
  `import xacro`，以及 xacro 裡 `$(find ...)` / `package://...` 指到的其他 package。
- **D2**：每個依賴（含 buildtool / test）都要能用 rosdep 解析：Humble 用 `ubuntu:jammy`、Jazzy 用 `ubuntu:noble`。
  沒有 rosdep 快取時記為 SKIP。

## 行為變化

- `rosdep install --from-paths openarm_description` 不再裝 Gazebo 和 RealSense 的套件，會補裝 `robot_state_publisher`（如果還沒有）。
- 編譯與執行不受影響：這四個依賴都只在執行期用到，CMake 沒有 `find_package` 它們。
- **u22 / u24**：8 個依賴在兩個版本都解析得到。Humble 實際會裝的套件（`rosdep install --simulate`）：
  `ros-humble-ament-cmake`、`ament-cmake-pytest`、`ament-lint-auto`、`ament-lint-common`、`joint-state-publisher-gui`、
  `robot-state-publisher`、`rviz2`、`xacro`。
- URDF 輸出不變（G1 全部通過）。

## 測試

修正前（`deps_check.py` 已加入、`package.xml` 還是舊的）：

```
  FAIL D1 declared but not used: ['realsense2_description', 'ros_gz']; used but not declared: ['robot_state_publisher']
  PASS D2 humble (ubuntu:jammy): 9 keys resolve
  PASS D2 jazzy (ubuntu:noble): 9 keys resolve
deps_check: FAILED
```

修正後的完整測試：

```
  openarm_description flake8: 40 failures (baseline 40) ok
  openarm_description pep257: 0 failures (baseline 0) ok
  openarm_bringup flake8: 743 failures (baseline 743) ok
  openarm_bringup pep257: 20 failures (baseline 20) ok
  PASS D1 runtime dependencies match use: ['joint_state_publisher_gui', 'robot_state_publisher', 'rviz2', 'xacro']
  PASS D2 humble (ubuntu:jammy): 8 keys resolve
  PASS D2 jazzy (ubuntu:noble): 8 keys resolve
urdf_checks: 148 passed, 0 failed, 24 known
[bimanual_openarm_hand]  S1 3 controllers active / S2 16 joints at 100 Hz / S3 14 joints reached target（gripper SKIP）
[bimanual_o6]            S1 5 controllers active / S2 36 joints at 501 Hz / S3 26 joints reached target / S4 10 mimic joints follow
FULL TEST PASSED
```

（第一次跑時 `deps_check.py` 自己有一個 flake8 Q001，flake8 變成 41 超過基準線，修掉 regex 的引號寫法後回到 40。）

## 後續

- D2 只證明「套件名稱在兩個版本都存在」。Humble 上實際編譯、渲染、啟動 fake 的驗證在後面的批次。
