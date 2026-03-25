# 頻率診斷功能使用說明

## 功能說明

頻率診斷功能可以實時監控控制循環的執行性能，包括：
- 實際運行頻率（目標：500 Hz）
- 循環執行時間統計（平均、最小、最大）

## 啟用方法

### 方法 1：URDF 參數（推薦）

在機器人描述檔案的 `<ros2_control>` 區塊中添加參數：

```xml
<ros2_control name="OpenArm_v10" type="system">
  <hardware>
    <plugin>openarm_hardware/OpenArm_v10HW</plugin>
    
    <!-- 啟用頻率診斷 -->
    <param name="enable_frequency_diagnostics">true</param>
    
    <!-- 其他參數 -->
    <param name="can_interface">can0</param>
    <param name="arm_prefix"></param>
    <!-- ... -->
  </hardware>
</ros2_control>
```

### 方法 2：環境變數

```bash
# 臨時啟用
ros2 launch openarm_bringup openarm.launch.py \
  enable_frequency_diagnostics:=true
```

## 輸出範例

啟用後，每 5 秒會輸出一次統計信息：

```
[OpenArm_v10HW_Thread]: Arm control loop started (target: 500Hz, diagnostics: ON)
[OpenArm_v10HW_Thread]: Loop stats: freq=498.7 Hz (target=500), 
                        exec_time: avg=850 us, min=650 us, max=1200 us
```

## 性能指標解讀

| 指標 | 理想值 | 可接受範圍 | 警告閾值 |
|------|-------|----------|---------|
| **頻率 (freq)** | 500 Hz | 490-510 Hz | < 480 Hz |
| **平均執行時間 (avg)** | < 1500 us | < 1800 us | > 2000 us |
| **最大執行時間 (max)** | < 1800 us | < 1900 us | > 2000 us |

### 注意事項

- 循環週期是 2000 us（1/500秒）
- 執行時間必須小於 2000 us 才能維持 500 Hz
- 如果 max > 2000 us，可能會偶爾掉幀

## 默認狀態

**默認為關閉**，不影響正常運行性能。

僅在需要性能調試時啟用。

## 故障排查

### 頻率低於預期（< 490 Hz）

**可能原因：**
1. CAN 通訊延遲
2. 補償計算耗時過長
3. CPU 負載過高

**解決方法：**
- 檢查 CAN 總線質量
- 暫時關閉補償功能測試
- 降低系統其他負載

### 執行時間過長（> 2000 us）

**可能原因：**
1. 啟用了重力/摩擦補償（增加約 200-400 us）
2. 軟體 PD 反饋計算
3. 線程調度問題

**解決方法：**
- 確認補償參數已正確優化
- 調整軟體 PD 增益係數
- 設置更高的線程優先級（需要 root 權限）
