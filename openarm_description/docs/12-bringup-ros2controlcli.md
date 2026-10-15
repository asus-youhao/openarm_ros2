# 12 `openarm_bringup` 依賴 `ros2controlcli`（P2）

## 問題

**P2**：照 README 在全新機器上用 rosdep 安裝後，README 第 6 步的 `ros2 control list_controllers` 不能用：

```
ros2: error: argument ...: invalid choice: 'control' (choose from 'action', 'bag', 'component', 'daemon', 'doctor', ...)
```

在 u22（Humble 容器）實測到；u24 也一樣會發生，只是這台 NB 剛好裝了 `ros-jazzy-ros2-control` meta 套件才沒遇到。

## 根因

`ros2 control` 子指令在 `ros2controlcli` 套件裡。`openarm_bringup` 只宣告 `controller_manager` 與 `ros2_controllers`，
兩者都不依賴 `ros2controlcli`（Jazzy 上它只被 `ros-jazzy-ros2-control` 這個 meta 套件帶進來）。

## 改動

- `openarm_bringup/package.xml`：加上 `<exec_depend>ros2controlcli</exec_depend>`。rosdep key 在兩個版本都解析得到
  （`ros-humble-ros2controlcli`、`ros-jazzy-ros2controlcli`）。
- `test/fake_smoke.py`：新增 **C1**：`ros2 control -h` 要能執行（README 的驗證步驟會用到）；用 `-h` 不會啟動 ros2 daemon。

## 行為變化

- `rosdep install`（README 第 3 步）會多裝 `ros2controlcli`，`ros2 control list_controllers` / `switch_controllers` 直接可用。
- 執行期行為不變（launch 用的是 `controller_manager` 的 spawner，不是 CLI）。

## 測試

修正前（C1 已加入，Humble image 是用舊的 `package.xml` 照 README 裝的）：

```
  FAIL C1 ros2: error: argument ...: invalid choice: 'control' (choose from 'action', 'bag', 'component', 'daemon', 'doctor'
```

修正後（`run_in_docker.sh` 用新的 `package.xml` 重建 image，等於重新照 README 裝一次）：

```
u24 Jazzy：  PASS C1 ros2 control -h works ... FULL TEST PASSED
u22 Humble： PASS C1 ros2 control -h works / urdf_checks 169 passed, 0 failed, 17 known / S4 PASS / H1 PASS ... FULL TEST PASSED
```

## 後續

無。
