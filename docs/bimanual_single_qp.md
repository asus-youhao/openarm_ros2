# 雙臂合併單一 QP IK（Bimanual Single-QP）— v1 設計與改動說明

> 日期：2026-06-12
> 背景：roadmap 第 10 項（見 `ik_optimization_roadmap_20260612.md`）。
> 原架構是兩個 `PlacoOnlineProfiler` node 各跑一個 7-DOF `PlacoSession`；
> 本版改為**一個 node、一個 14-DOF QP、兩個 end-effector task**。
> 全部是新檔案，**未修改任何既有檔案**（共用元件以 import 取用）。

---

## 新增檔案

| 檔案 | 角色 |
|------|------|
| `ik_node/placo_ik_session_bimanual.py` | `PlacoBimanualSession` — 14-DOF 單一 QP，雙 EE task，純 Python 無 ROS |
| `ik_node/placo_ik_node_bimanual.py` | `PlacoBimanualNode` — 單一 ROS node、單一 hot loop、per-arm 狀態管理 |
| `ik_node/placo_ik_online_profiler_bimanual.py` | CLI 入口（取代 `placo_ik_online_profiler_ws_mesh.py --arm both`） |
| `docs/bimanual_single_qp.md` | 本文件 |

從既有檔案 import（未修改）：`placo_ik_node.py`（TfPoller / AsyncCsvWriter /
quaternion helpers / `_parse_lpf_alpha`）、`kbd_controller.py`、`ws_boundary.py`、
`placo_ik_solver.py`（URDF / `_HUMAN_*` / `_quat_to_rot`）、`arm_config.py`、`paths.py`。

## 執行方式

```bash
# 裸跑即可（左臂 calib 預設 180°，不再需要 --config 才能安全跑雙臂）
conda run -n pico_teleop_py python3 placo_ik_online_profiler_bimanual.py

# 帶 YAML（沿用 config/bimanual.yaml 的 common/right/left 結構）
python3 placo_ik_online_profiler_bimanual.py --config ../config/bimanual.yaml

# dry-run
python3 placo_ik_online_profiler_bimanual.py --dry-run
```

Topic 介面與舊版相容（兩臂 fwd commands / latency / `_arm_ik_commands` 不變），
合併 profile 改發 `/bimanual/placo_profile`。

Service：`/bimanual/go_home`（std_srvs/Trigger，兩臂同時回 home；
設計見 `home_service.md`——callback 只掛旗標，homing 由 hot loop 執行）。

---

## 設計總覽

```
/ee_delta/right ─┐                       ┌─ /right_forward_position_controller/commands
                 ├─ PlacoBimanualNode ───┤
/ee_delta/left ──┘   （單一 hot loop）    └─ /left_forward_position_controller/commands
                          │
                 PlacoBimanualSession
                  單一 KinematicsSolver（14 DOF）
                  ├─ pos/ori task × right EE
                  ├─ pos/ori task × left EE
                  ├─ naturalness × 2（per-arm）
                  ├─ per-arm damping task（σ_min 低的臂才加）
                  └─ global regularization（λ_base 固定）
```

**重要認知**：兩臂是運動學獨立鏈，合併 QP 不改變 IK 解本身。價值在：
(1) 共享約束（EE 近距節流、未來的 collision constraint / relative task）、
(2) 單一 hot loop 兩臂 timing 不互漂、(3) 一份 RobotWrapper 記憶體減半。

hot loop 語意：任一臂收到有效 tracker 訊息才 solve（兩臂一起解）；
沒收到訊息的臂 target 維持上次值，由 QP hold-in-place。

---

## 與舊版（雙 node）的行為差異

### 1. per-arm adaptive DLS（取代全域 λ）

舊版 `add_regularization_task(λ_dls)` 是全域的 → 合併後一臂進奇異點會把
另一臂也阻尼變慢。本版：

- 全域 regularization 固定在 `λ_base = 6e-5`（純數值穩定）。
- 每臂各算 6×7 Jacobian 的 σ_min；σ_min < 0.15 的臂額外加一個
  **拉向 seed 的 JointsTask**，權重 `λ_extra = λ_max × ratio²`（λ_max=5e-2）。
  效果：只阻尼該臂的關節運動，另一臂完全不受影響。
- 注意這是「以 JointsTask 模擬 per-arm regularization」的近似：
  regularization 懲罰的是速度，JointsTask(seed) 懲罰的是本 step 內累積位移，
  在單 step 尺度下效果等價。

### 2. 統一 Δq 縮放（roadmap #1，取代逐軸 jump guard）

舊版的問題鏈：velocity cap 定義在 per-iteration（`solver.dt`×`solve()` 次數）
→ 奇異點 iter boost ×4 時等效限速放寬 4 倍 → 10° jump guard 逐軸獨立 clip
→ 破壞 7 軸耦合、EE 走歪（docs issue #5）。

本版：solve 完後對每臂

```
Δq = q_sol − seed
s  = min(1, budget_rad / max|Δq|)        # budget 預設 10°/tick
q_pub = seed + s × Δq                     # 整臂統一比例
```

- per-tick 速度預算與 iteration 數**解耦**（iter boost 只影響收斂品質）。
- 統一縮放保持解的方向 → EE 沿直線慢走而不是被 clip 走歪。
- 預算單位天然是 deg/tick，換 rate 不改變安全性（docs issue #11 一併解掉）。
- node 端不再有 `_joint_jump_guard`；CSV 改記 `{r,l}_budget_scale`。

### 3. EE 近距節流（防撞減速帶）

每步計算兩 EE 距離（seed FK = dist_before，solve 後 = dist_after）：

```
if dist_after < dist_before:                                  # 接近中才管
    ramp = clamp((dist_before − hard 0.04m) / (soft 0.10m − hard), 0, 1)
           # dist_before ≥ soft 時 ramp = 1（不減速）
    cap  = min(1, 0.5 × (dist_before − hard) / (dist_before − dist_after))
           # 每步最多縮小剩餘間距的一半 → 幾何收斂、單步不穿越 floor
    prox = min(ramp, cap)
    兩臂 Δq 預算 ×= prox
```

設計重點（dry-run 測試時抓到的死鎖 bug 修正）：ramp 必須以「目前實際構型」
的 dist_before 計，**不能用 QP 解構型的 dist_after**——奇異點 iter boost 時
QP 解可以一步衝很遠，dist_after 直接低於 hard floor → prox=0 → seed 永不
前進 → 兩臂相距 30cm 也被凍住。dist_after 只用來判斷方向（接近/遠離）。
只節流「正在接近」的步——遠離方向永遠全速，不會把操作者鎖死。
（dry-run 實測：從 30.8cm 接近，漸減速停在 5.4cm；遠離時 prox 恆 1.0。）
**⚠ 這是減速帶不是防撞保證**：只看 EE 點距，不看前臂/手肘 link。
正解是 placo self-collision constraint，但目前 `_build_kin_urdf()` 會把
collision geometry 從 URDF 剝掉（placo 需要 mesh-free URDF），所以暫不可用
（見 TODO）。

### 4. 動態 W_ORI（roadmap #6 / docs j3-4）

σ_min < 0.15 的臂，姿態權重從 2.0 依 `σ/σ_thresh` 線性降至下限 0.5——
奇異點附近讓位置主導，wrist 不再為硬湊姿態而擺動。
用 `placo_ik_session_bimanual.py` 的 `_DYNAMIC_W_ORI = False` 可關閉
（關掉後與舊版固定 2.0 行為一致，方便 A/B）。

### 5. scale 改變 → 自動 re-anchor（roadmap #4 修正）

tracker delta 是「相對 anchor 的累積量 × scale」。舊版按 1-9/+/- 改 scale
不 reanchor → 整段 offset 被瞬間重縮放 → 手臂跳。
本版 node 在 hot loop 偵測 `kbd.scale` 變化，立即對兩臂觸發 reanchor。

### 6. 兩臂 calib 預設 0°（docs issue N2 修正，取代舊 N1）

**v1 初版**（2026-06-12）誤設 right=0° / left=180°，以為左臂需 180° 修「鏡像」。

**2026-06-15 實測校正（N2）**：右臂 0° 動作正常；左臂 180° 反而造成方向反：

| 動作 | 左臂（舊 180°）實測 | 判定 |
|------|--------------------|------|
| delta_x +（前） | arm 往後 | X 反 ✗ |
| delta_x −（後） | arm 往前 | X 反 ✗ |
| delta_y +（左） | arm 往右 | Y 反 ✗ |
| delta_y −（右） | arm 往左 | Y 反 ✗ |
| Z（上下） | 一致 | ✓ |

「X 反 + Y 反 + Z 對」正是 **180° yaw 旋轉的特徵**——左臂多套了一個翻轉。

**根因**：Pico 左右手 controller 的**位置**都在同一個 global tracking space，
無左右鏡像；右臂 0° 既然正常，左臂理應相同。舊版「左臂需 180° 修鏡像」
的假設經實測證偽。

**結論：兩臂都用 `calib_yaw=0°`。** `bimanual.yaml` 與裸跑預設已同步更新。
YAML / CLI `--calib-yaw-right / --calib-yaw-left` 仍可個別覆寫。

### 7. 其他小改

| 項目 | 舊版 | 本版 |
|------|------|------|
| fail 時 `_pose` 簿記 | solver 內部 EE（與發布值不符） | `ee_cmd_xyz` = 縮放後實際發布關節的 FK |
| reanchor 事件 | 不記錄 | CSV `event` 欄（`reanchor_right+reanchor_left`，docs #15） |
| iter boost 觸發 | 該臂 σ | 任一臂 σ（QP 是聯合迭代）；速度由 Δq 預算管，安全 |
| early exit | 單臂收斂即停 | **兩臂都**收斂才停 |
| CSV | 兩份單臂 CSV | 一份合併 CSV（`r_*` / `l_*` 前綴 + 全域欄位） |

### 沿用不變（便於 A/B 對照）

調參常數全部同值：`_POS_TOL/_ORI_TOL/_*_RELAX`、`_W_POS/_W_ORI/_W_JOINTS`、
`λ_base/λ_max/σ_thresh`、iter boost ×4、wrist cap 4.0 rad/s、arm cap 1°/iter、
j3/j4 coupling 參數（預設 off）、SoftClamp/BoundaryMonitor、輸入端 SLERP、
輸出端 LPF（預設 0.25 統一）、anchor/gap/首訊息丟棄語意、Set2-D always-publish。

---

## v1 限制（刻意縮小範圍）

- **keyboard teleop 不支援**：鍵盤只做全域控制（scale/pause/reset/home/quit）。
- **只走 ForwardCommandController**（無 `--use-traj`）。
- **placo self-collision constraint 未啟用**：kin-URDF 無 collision geometry。
- 與 roadmap 其餘項目（TF 閉迴路校正、One-Euro、延遲補償、Ruckig OTG）
  刻意不混入本版，保持單一變因方便對照測試。

## TODO（之後做）

1. **真 collision constraint**：產一份保留 collision primitive（capsule/box，
   非 mesh）的 URDF 變體，RobotWrapper 改不帶 `ignore_collisions`，
   啟用 placo self-collision 約束 → 取代 EE 近距節流。
2. **relative task 雙手協調**：placo `add_relative_position_task` 做
   「雙手保持相對位姿」模式（搬箱）。
3. roadmap #3（TF 慢速閉迴路）與 #2（task-space One-Euro）疊加進來。
4. 實機 A/B：本版 vs 雙 node 版，比較 CSV 的 pos_err / budget_scale 觸發率 /
   奇異點行為 / 雙臂接近時的節流體感。

## 測試備忘

已完成的離線 smoke test（2026-06-12，無 ROS、純 session）：

- FK 與 `ARM_CONFIG` home_pose 一致（±0.1535 對稱）。
- 雙臂 14-DOF QP：首步 ~1.4ms、穩態 **mean 0.14ms / p95 0.15ms**
  （50Hz 預算 20ms）；home+2cm 目標兩臂皆 success、pos_err 收斂至 ~1mm。
- 近距節流：接近時 30.8cm → 停在 5.4cm（> hard floor 4cm，無死鎖）；
  遠離時 prox 恆 1.0 全速。

實機測試（待做）：

- 先 `--dry-run` 確認 QP 數值正常（兩臂 pos_err、σ_min、iterations）。
- 實機首測建議 `--tick-budget-deg 5`（保守一半）+ 單臂先動。
- 雙臂互相靠近測 prox 節流：觀察 CSV `ee_dist_mm`/`prox_scale`，
  確認遠離時不被節流。
- 改 scale 時觀察 console 應印 `auto re-anchor both arms` 且手臂不跳。
