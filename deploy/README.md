# placo_ik — 部署到其他 x86_64 PC

把 `ik_node/placo_ik_main.py` 跑起來所需的依賴安裝、Docker 環境，以及
`ros2 launch` 包裝。目標環境：**ROS 2 Humble / Ubuntu 22.04 / Python 3.10**。

> 圖文版（含流程圖、依賴分層）：[`INSTALL.html`](INSTALL.html)

---

## TL;DR

```bash
cd placo_ik/deploy
./shell.sh                                   # 進容器（第一次會自動 build image）
# 容器內：
ros2 launch /work/openarm_ros2/placo_ik/launch/placo_ik.launch.py arm:=right
```

---

## 兩種安裝路徑

### A. Docker（推薦）

依賴全部烤進 `placo-ik-humble:latest`，裝一次、之後秒進，且已設好跟本機 ROS 2 通訊。

```bash
cd placo_ik/deploy
./shell.sh                       # 互動 shell，落在 ik_node/
./shell.sh ros2 topic list       # 一次性指令
REBUILD=1 ./shell.sh             # 強制重 build image
```

`shell.sh` 自動帶入：`--network host`、`--ipc host`、`--user $(id -u):$(id -g)`、
`ROS_DOMAIN_ID=10`、`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`，並預設
`OPENARM_URDF=/work/openarm_ros2/placo_ik/ik_solver/v10_o6.urdf`。

| 環境變數 | 預設 | 說明 |
|---|---|---|
| `ROS_DOMAIN_ID` | `10` | 對齊本機 `~/.bashrc` |
| `RMW` | `rmw_fastrtps_cpp` | Humble 預設；改 `rmw_cyclonedds_cpp` 可切換 |
| `REBUILD` | `0` | `=1` 強制重 build |
| `ROOT` | `0` | `=1` 以 root 跑（會犧牲本機 ROS 通訊，見下） |

### B. 直接裝在本機

該機已有 ROS 2 Humble、想把依賴裝進系統 Python 時用：

```bash
cd placo_ik/deploy
./install_deps.sh        # apt 裝 ROS 套件 + 升級 pip + pip 裝 Python 依賴
./verify_deps.sh         # import 煙霧測試 + 跑 --help
export OPENARM_URDF=/path/to/openarm.urdf
```

`install_deps.sh` 可調的環境變數：`SKIP_APT=1`、`SKIP_PIP=1`、`PIP="pip install --user"`、`ROS_DISTRO=humble`。

---

## 會安裝哪些依賴

**APT（系統，7 個）**
`python3-pip`、`ros-humble-rclpy`、`ros-humble-geometry-msgs`、`ros-humble-sensor-msgs`、
`ros-humble-std-msgs`、`ros-humble-trajectory-msgs`、`ros-humble-tf2-ros-py`

**PIP（科學運算）**
`numpy>=2.2,<2.3`、`scipy`、`matplotlib`、`PyYAML`

**placo + cmeel 子樹（鎖死版本）**
`placo==0.9.20` 及其全部預編譯依賴：`pin`、`libpinocchio`、`coal`、`libcoal`、`eigenpy`、
`eiquadprog`、`cmeel`、`cmeel-boost`、`cmeel-urdfdom`、`cmeel-tinyxml2`、`cmeel-console-bridge`、
`cmeel-octomap`、`cmeel-qhull`、`cmeel-assimp`、`cmeel-zlib`、`rhoban-cmeel-jsoncpp`、`meshcat`、
`ischedule`。完整版本見 [`requirements.txt`](requirements.txt)。

---

## 用 `ros2 launch` 啟動（不改動腳本）

[`../launch/placo_ik.launch.py`](../launch/placo_ik.launch.py) 用 `ExecuteProcess` 包裝腳本，
把 launch 參數轉成腳本既有的 argparse flag——**你的 Python code 完全不動**。

```bash
# 列出所有參數
ros2 launch placo_ik/launch/placo_ik.launch.py -s

# 右臂
ros2 launch placo_ik/launch/placo_ik.launch.py arm:=right

# 右臂、dry-run、跳過 home
ros2 launch placo_ik/launch/placo_ik.launch.py arm:=right dry_run:=true home:=false

# 容器內（注意用 /work/... 的掛載路徑）
./deploy/shell.sh ros2 launch /work/openarm_ros2/placo_ik/launch/placo_ik.launch.py arm:=right
```

**參數對照**（`launch_arg:=value` → 腳本 flag）：

| launch 參數 | 腳本 flag | 預設 |
|---|---|---|
| `arm` | `--arm` | `both` |
| `config` | `--config` | （空＝腳本預設） |
| `rate` | `--rate` | `50.0` |
| `horizon` | `--horizon` | auto |
| `max_iter` | `--max-iter` | 腳本預設 |
| `ws_mesh` | `--ws-mesh` | auto-detect |
| `csv` / `plot` | `--csv` / `--plot` | 空 |
| `calib_yaw` / `calib_rpy` | `--calib-yaw` / `--calib-rpy` | 空 |
| `wrist_vel_cap` | `--wrist-vel-cap` | 腳本預設 |
| `lpf_alpha` / `ori_lpf_alpha` | `--lpf-alpha` / `--ori-lpf-alpha` | 空 |
| `home` | `--no-home-first`（當 `home:=false`） | `true` |
| `dry_run` | `--dry-run` | `false` |
| `rebuild` | `--rebuild` | `false` |
| `verbose` | `--verbose` | `false` |
| `keyboard` | `--keyboard` | `false` |
| `use_traj` | `--use-traj` | `false` |
| `no_rot_tracking` | `--no-rot-tracking` | `false` |
| `success_gate` | `--success-gate` | `false` |

> 值類型參數留空字串時不會傳給腳本，等同沿用腳本自身的預設值。

執行 IK 時仍需本機有 robot driver、且 tracker 在發 `/ee_delta/{arm}`。

---

## 驗證

```bash
# 1) 依賴煙霧測試（不需硬體）
./verify_deps.sh

# 2) 在全新 ros:humble 容器驗證安裝腳本可用
./test_docker_humble.sh

# 3) 本機 ROS 2 通訊測試（容器發、本機收）
./shell.sh ros2 topic pub /ping std_msgs/msg/String "{data: hi}" -r 5   # 一個終端
ros2 topic echo --once /ping                                            # 另一終端，需 ROS_DOMAIN_ID=10
```

---

## 關鍵約束（打包到別台必看）

1. **numpy 必須 2.2.x，不能 `<2`。** `placo 0.9.20 → pin 3.8.0 → cmeel-boost 1.89.0` 在 cp310
   硬性要求 `numpy>=2.2,<2.3`；壓 `<2` 會 `ResolutionImpossible`。

2. **cmeel 子樹整組鎖死。** 這些是預編譯 `.so`，以 soname 連結
   （`liburdfdom_sensor.so.4.0`、`libtinyxml2.so.10`…）。`placo`/`pin` 只宣告鬆約束，全新解析會抓到
   soname 更新的 wheel，導致 `import placo` 報 `lib<x>.so.<n>: cannot open shared object file`。

3. **容器要以本機 UID 跑。** FastDDS 透過 `/dev/shm` 的 per-UID 共享記憶體段做 discovery；root 容器
   建的段本機（uid 1000）寫不進去 → `ros2 topic echo` 只會 timeout。`shell.sh` 已用
   `--user $(id -u):$(id -g)` 解決。

4. **URDF 不是依賴、是資料。** placo 載入前會剝掉 `<visual>`/`<collision>`，**不需帶 meshes**，
   只要一個 `.urdf`。設 `OPENARM_URDF` 指到它即可。

> 若 `ros2 topic echo` 噴 `xmlrpc Fault: !rclpy.ok()`，是本機 ros2 daemon 壞了，
> 跑 `ros2 daemon stop` 重啟即可，與容器無關。

---

## 檔案清單

| 檔案 | 用途 |
|---|---|
| `requirements.txt` | pip 依賴清單（含版本鎖與原因） |
| `install_deps.sh` | 本機安裝：apt + pip |
| `verify_deps.sh` | import 煙霧測試 + `--help` |
| `Dockerfile` | 把依賴烤進 `placo-ik-humble:latest` |
| `shell.sh` | 進容器（host network + UID + RMW/domain 對齊） |
| `test_docker_humble.sh` | 在乾淨 `ros:humble` 容器驗證安裝腳本 |
| `INSTALL.html` | 圖文版安裝文件 |
| `../launch/placo_ik.launch.py` | `ros2 launch` 包裝（不改腳本） |
