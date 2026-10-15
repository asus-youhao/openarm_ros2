# 10 在 Docker 裡跑 u22（Humble）的完整測試

## 問題

README 寫了 u22 / Humble 的差異，但從來沒有在 Humble 上實際跑過；這台 NB 是 u24，沒有 Humble 可以測。
差異表的 Humble 欄位全是推論。

## 根因

完整測試只會用本機 `/opt/ros/` 底下的 ROS，沒有辦法在別的版本上跑。

## 改動

- `test/docker/Dockerfile`：`ros:<distro>-ros-base`，依賴用 README 同樣的方式（`rosdep install` 兩個 package 的
  `package.xml`）安裝，再加上測試需要的 `git`、`libxml2-utils`（xmllint）、`liburdfdom-tools`（check_urdf）、colcon。
  `rosdep update` 不限定 distro，D2 才能同時解析 humble 與 jazzy。
- `test/run_in_docker.sh [humble|jazzy]`：用工作樹裡的兩個 `package.xml` 建 image（`openarm-<distro>-test:local`），
  把 repo 以唯讀方式掛進容器，執行 `run_full_test.sh`；build / install / log 都留在容器裡，跑完就丟。
  容器用 Docker 預設的 bridge 網路：`ament_xmllint` 要下載 `package.xml` 的 schema（`--network none` 時會多 1 個 xmllint 失敗，
  實測確認），DDS 則留在容器自己的網路 namespace 裡。
- `run_full_test.sh`：容器裡沒有這個 worktree 的 git 資料，印 repo 版本那行改成讀不到時顯示 `git n/a`，不再噴 `fatal`。

## 行為變化

無，只多了一種跑測試的方式。

## 測試

u24 本機完整測試：FULL TEST PASSED。

u22（`run_in_docker.sh humble`，這個 commit 的程式）：

```
ROS      : humble
  openarm_description flake8: 0 failures (baseline 40) ok      ← Humble 的 flake8 設定比較寬鬆
  openarm_description xmllint: 0 failures (baseline 0) ok
  openarm_bringup flake8: 95 failures (baseline 743) ok
  openarm_bringup pep257: 20 failures (baseline 20) ok
  PASS D1 runtime dependencies match use: [...]
  PASS D2 humble (ubuntu:jammy): 8 keys resolve
  PASS D2 jazzy (ubuntu:noble): 8 keys resolve
urdf_checks: 155 passed, 0 failed, 17 known                    ← 含 G1：Humble 的 xacro 渲染結果跟 Jazzy 完全一樣
[bimanual_default]  PASS S1 5 controllers active（夾爪 controller 也起來了）/ S2 16 joints at 100 Hz / S3 14 joints reached target
[bimanual_o6]       PASS S1 5 controllers active / S2 36 joints at 501 Hz / S3 26 joints reached target
                    FAIL S4 mimic joints (actual, expected): {'R_thumb_dip': (0.0, 0.279), ...}
[real_o6_parse_probe] PASS H1 URDF parsed; stopped at loading openarm_hardware/OpenArm_v10HW (not installed)
FULL TEST FAILED (1): fake smoke
```

也就是說，u22 上只有一個問題：**U1，fake 模式的 O6 dip 不會跟著動**（停在 0）。Humble 的
`mock_components` 不讀 URDF 的 `<mimic>`，要靠 ros2_control 裡的 `<param name="mimic">` / `multiplier`。修正見 11。

其他在 Humble 上確認的事：README 的 `rosdep install` 一次就裝好（含 `ros-humble-ros2-controllers` 與夾爪 controller）；
真機 O6 的 `mimic="false"` 在 Humble 上不影響解析。

## 後續

- 11：修 U1。
- CI：之後可以把 `run_in_docker.sh humble` / `jazzy` 放進 GitHub Actions。
