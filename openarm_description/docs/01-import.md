# 01 匯入 openarm_description

## 問題

`openarm_description` 原本在另一個 repo（`asus-youhao/openArm_leapHand_urdf`）的子目錄裡。要讓 openarm_ros2 起來，
一定要同時 clone 兩個 repo，而那個 repo 有 1 GB 以上的東西跟 URDF 無關：根目錄的 `usd/`（418 MB）、
`release/`（473 MB）、`leap_hand/`、`o6_urdf/`，package 裡面也還有 `usd/`、`urdf_transform/` 和沒用到的 mesh。
使用者要收掉那個 repo，只留 description。

## 根因

舊 repo 同時當作模型原始檔、Isaac Sim（USD）匯出、URDF 轉換腳本和 ROS package 的存放處，
ROS 只需要其中的 package，但每個人都得 clone 整包。

## 改動

從 `openArm_leapHand_urdf@6b8d6a7`（`fix/jazzy-fake-hardware`，已含 J1–J3）取出
`src/openarm_description` 的下列內容，放到 `openarm_ros2/openarm_description/`：

| 帶進來 | 內容 |
|---|---|
| `CMakeLists.txt`、`package.xml`、`LICENSE.txt`、`README.md` | 原樣 |
| `urdf/`（22 個 xacro）、`config/`、`launch/`、`rviz/` | 原樣 |
| `meshes/` 66 個檔案，88.7 MB | URDF 實際引用的 mesh，加上 `.obj` 透過 `mtllib` 帶到的 `.mtl` |

| 沒帶 | 原因 |
|---|---|
| `usd/`（35 MB）| Isaac Sim 的 USD 與匯出的 URDF，不屬於 ROS package |
| `urdf_transform/`（35 MB）| 一次性的 URDF 座標轉換腳本與產物 |
| `meshes/` 其餘 38 個檔案，92.5 MB | URDF 沒有引用：FreeCAD 原檔（`.FCStd`）與說明、截圖、沒用到的 `.dae` / `.glb`、舊版 mesh（`*_old1202.stl` 等） |
| `.github/`、`dev/`、`.pre-commit-config.yaml`、`.cmake-format.py`、`CODE_OF_CONDUCT.md`、`CONTRIBUTING.md`、`.gitignore` | 獨立 repo 時的設定，openarm_ros2 根目錄已經有自己的 |

「需要的 mesh」怎麼算：用舊 package 渲染 `bimanual`（true/false）× `ee_type`（openarm_hand、o6、leap_hand_right、
leap_hand_left、none）共 10 組，收集所有 `<mesh filename>`（去掉註解），再沿著 `.obj` 的 `mtllib`、`.mtl` 的貼圖、
`.dae` 的 `<init_from>` 找出間接需要的檔案。openarm_ros2 裡沒有其他地方直接引用 description 的 mesh。

整個 package 從 243 MB 變成 107 個檔案、86 MB。package 名稱不變，所以
`openarm_bringup`、`openarm_bimanual_moveit_config`、`openarm`（O6 standalone xacro）都不用改。

測試框架從舊 repo 移過來，放在 `test/`，並針對整合後的 repo 調整：

- `run_full_test.sh`：可以用 `OPENARM_ROS_DISTRO` 指定 humble / jazzy；preflight 改成確認工作區裡只有一個
  `openarm_description`、而且就是這一份；linters 同時比 `openarm_bringup`。
- `urdf_checks.py`：舊的 R4（真機輸出跟 base commit 比）在單一 repo 裡沒有 base 可比，改成 **G1**：
  跟 `test/golden.json` 比。golden 存每組參數「整份 URDF 正規化後的 SHA-256」，以及 ros2_control 區塊的
  逐行摘要（plugin、參數、每個關節的 command / state interface 與 `mimic` 屬性，13 種，36 KB）。
  刻意改到 ros2_control 時用 `--update-golden` 重產，diff 會直接出現在 commit 裡。
  渲染結果裡的安裝路徑會換成 `$(find openarm_description)`，所以 golden 跟安裝位置無關。
- 新增 **M1**：URDF 引用的每個 mesh 都要存在於安裝後的 package。
- F3 現在兩種模式都檢查；真機模式的違反記成 K3。
- `test/*.py` 放進 package 之後也會被 ament linters 檢查，已修到 0 個問題。

## 行為變化

無。`golden.json` 是用**舊 repo**（`~/openarm_descfix_ws`，`6b8d6a7`）編出來的 package 產生的，
新 package 渲染出來的 24 組 URDF 全部跟它一致（G1），代表匯入沒有遺漏、也沒有改到任何東西。

u22 / u24：這一批只搬檔案，相容性跟舊 repo 相同。Jazzy 已經測過；Humble 的驗證在後面的批次。

## 測試

```
=== [1] preflight: exactly one openarm_description in the workspace, and it is this one
openarm_description: 1 found (src/openarm_ros2/openarm_description)
=== [3] xmllint on every xacro / urdf in the package
xmllint: 22 files, 0 not well-formed
=== [4] ament linters vs test/lint_baseline.txt
  openarm_description flake8: 40 failures (baseline 40) ok
  openarm_description pep257: 0 failures (baseline 0) ok
  openarm_description lint_cmake: 0 failures (baseline 0) ok
  openarm_description xmllint: 0 failures (baseline 0) ok
  openarm_bringup flake8: 743 failures (baseline 743) ok
  openarm_bringup pep257: 20 failures (baseline 20) ok
  openarm_bringup lint_cmake: 0 failures (baseline 0) ok
  openarm_bringup xmllint: 0 failures (baseline 0) ok
=== [5] URDF checks (render matrix, meshes, fake rules, golden)
urdf_checks: 148 passed, 0 failed, 24 known
=== [6] fake launch smoke tests (test/fake_smoke.py)
[bimanual_openarm_hand] openarm.bimanual.launch.py use_fake_hardware:=true ee_type:=openarm_hand launch_rviz:=false
  SKIP S1 ['left_gripper_controller', 'right_gripper_controller']: package gripper_controllers not installed
  PASS S1 3 controllers active
  PASS S2 16 joints at 100 Hz
  PASS S3 14 joints reached target
[bimanual_o6] openarm_o6_bimanual.launch.py use_fake_hardware:=true launch_rviz:=false
  PASS S1 5 controllers active
  PASS S2 36 joints at 501 Hz
  PASS S3 26 joints reached target
  PASS S4 10 mimic joints follow their parent
FULL TEST PASSED
```

known 的分布：K1 13 項（R3）、K3 1 項（F3）、K4 6 項（M1）、K5 4 項（M1）。
`openarm_description` flake8 的 40 項全部在 `launch/display_openarm.launch.py`（37 個雙引號 Q000，
加上 import 順序 I100 / I101 / I201），是原樣搬過來的檔案；`openarm_bringup` 的基準線是 develop 原本的數字。

## 後續

- K4（碰撞 mesh 路徑多一個 s）、K3（真機 mimic）、X1（launch 預設 `ee_type`）在接下來的批次處理。
- K5 的 `mcp_joint_left.obj` 在新舊 repo 都找不到（舊 repo 只有右手的 `mcp_joint.obj` 和 `pip_left.obj`），
  用右手模型代替會畫出錯的形狀，需要有人提供左手的 mesh。
- `display_openarm.launch.py` 的 40 個 flake8 問題只是格式，留到需要改這支 launch 時再一起修。
