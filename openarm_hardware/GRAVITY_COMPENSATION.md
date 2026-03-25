# Gravity Compensation 重力補償

## 功能說明

已為 `v10_simple_hardware` 添加基於 KDL (Kinematics and Dynamics Library) 的重力補償功能。

## 實作細節

### 1. 技術架構
- **URDF 解析**: 從 robot_description 參數讀取 URDF 模型
- **KDL 樹構建**: 將 URDF 轉換為 KDL 運動學樹
- **動力學求解**: 使用 `KDL::ChainDynParam` 計算重力力矩
- **MIT Control 整合**: 將重力補償作為前饋力矩加入控制迴路

### 2. 控制流程

```cpp
// 每個控制週期 (100Hz)
compute_gravity_compensation(gravity_comp);  // 計算重力力矩

for (size_t i = 0; i < ARM_DOF; ++i) {
    feedforward_tau = tau_commands_[i] + gravity_comp[i];
    
    arm_params.push_back({
        kp_[i], kd_[i],           // PD 增益
        pos_commands_[i],         // 目標位置
        vel_commands_[i],         // 目標速度
        feedforward_tau           // 前饋力矩 = 指令力矩 + 重力補償
    });
}
```

### 3. 重力補償計算

```cpp
// 1. 讀取當前關節位置
q(i) = pos_states_[i];

// 2. 使用 KDL solver 計算重力力矩
kdl_solver_->JntToGravity(q, gravity_torques_);

// 3. 輸出到控制迴路
gravity_torques[i] = gravity_torques_(i);
```

## 啟用方式

### 方法 1: URDF 硬體參數設定

在 URDF 的 `<ros2_control>` 標籤中添加參數：

```xml
<ros2_control name="OpenArmV10System" type="system">
  <hardware>
    <plugin>openarm_hardware/OpenArm_v10HW</plugin>
    <param name="use_gravity_compensation">true</param>
    <!-- URDF 會自動作為 robot_description 參數傳遞 -->
  </hardware>
  ...
</ros2_control>
```

### 方法 2: 檢查當前配置

重力補償預設為**關閉**，需要手動啟用。

查看啟動日誌：
```bash
# 如果啟用成功
[INFO] [OpenArm_v10HW]: KDL dynamics initialized: chain from openarm_body_link0 to openarm_right_hand with 7 joints
[INFO] [OpenArm_v10HW]: Gravity compensation enabled

# 如果未啟用
[INFO] [OpenArm_v10HW]: Gravity compensation disabled
```

## 效果與優勢

### ✅ 有重力補償的效果
1. **減少穩態誤差** - 重力造成的下垂被補償
2. **降低馬達負載** - 前饋力矩抵消重力
3. **更精準的控制** - 特別是低增益時
4. **減少能耗** - 馬達不需持續對抗重力

### 📊 與 Teleop 比較

| 項目 | Teleop | ros2_control (本實作) |
|------|--------|----------------------|
| 重力補償 | ✅ KDL | ✅ KDL |
| 摩擦補償 | ❌ (參數=0) | ❌ (未實作) |
| 科氏力補償 | ❌ (未使用) | ❌ (未實作) |
| 控制頻率 | 500 Hz | 100 Hz |
| Kp 增益 | 240.0 (J1-4) | 240.0 (J1-4) ✅ |
| Kd 增益 | 3.0 (J1-4) | 3.0 (J1-4) ✅ |

## 測試建議

### 1. 不啟用重力補償測試
```bash
# 使用提高的 Kp/Kd 增益
ros2 launch openarm_bringup openarm.bimanual.launch.py
```

### 2. 啟用重力補償測試
修改 URDF，添加 `use_gravity_compensation` 參數後：
```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py
```

### 3. 比較 Steady-State Error
```bash
# 監控關節狀態
ros2 topic echo /joint_states --field position

# 監控指令與實際差異
ros2 run openarm_scripts monitor_joint_error.py  # (需自行創建)
```

## 故障排除

### 錯誤: "Failed to parse URDF for KDL dynamics"
- **原因**: robot_description 參數不存在或格式錯誤
- **解決**: 檢查 URDF 檔案是否正確加載

### 錯誤: "Failed to get KDL chain from openarm_body_link0 to openarm_right_hand"
- **原因**: URDF 中連結名稱不匹配
- **解決**: 確認 URDF 中的 link 名稱與代碼一致

### 警告: "Gravity compensation disabled"
- **原因**: `use_gravity_compensation` 參數未設為 true
- **解決**: 在 URDF hardware 參數中添加設定

## 備份檔案

原始版本已備份至：
```
openarm_hardware/src/v10_simple_hardware.cpp.backup_velocity_feedback
```

## 編譯

```bash
cd ~/ros2_ws_yh
colcon build --packages-select openarm_hardware
source install/setup.bash
```

## 相關檔案

- `include/openarm_hardware/v10_simple_hardware.hpp` - 添加 KDL 成員變數和函數聲明
- `src/v10_simple_hardware.cpp` - 實作 KDL 初始化和重力計算
- `package.xml` - 添加 kdl_parser, orocos_kdl, urdf 依賴
- `CMakeLists.txt` - 添加 KDL 相關 find_package 和連結
