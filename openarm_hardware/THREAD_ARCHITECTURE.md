# OpenArm Hardware Thread Architecture

## 完整數據流

```
Action Client (30 FPS)
    ↓
JointTrajectoryController (100Hz, 內建 action server)
    ├─ Trajectory interpolation
    ├─ time_from_start processing
    └─ write() commands
        ↓
v10_simple_hardware.cpp (Hardware Interface)
    ├─ hw_position_commands_[0-6]  → Arm Control Thread (1000Hz loop)
    │                                  └─ CAN → OpenArm motors
    │
    └─ hw_position_commands_[7-22] → LEAP Hand Control Thread (100Hz loop)
                                       └─ Serial → Dynamixel motors
```

## v10_simple_hardware.cpp 開啟多少 Thread?

**單臂系統**: 最多 **2 個 threads**
- Arm Control Thread (1000Hz)
- LEAP Hand Control Thread (100Hz) - 僅在 `has_leap_hand_` 時啟動

**雙臂系統**: 最多 **6 個 threads**
- Controller Manager Thread (100Hz) - ROS2 Control 框架自動管理
- Left Arm Instance:
  - Left Arm Control Thread (1000Hz)
- Right Arm Instance:
  - Right Arm Control Thread (1000Hz)
  - Right LEAP Hand Control Thread (100Hz)

## 三個關鍵問題解答

### 1. OpenArm read 和 write 是同一個 thread 嗎?

**否**，它們是**不同的 threads**:

- **`read()` 和 `write()`**: 由 **controller_manager thread** 在 100Hz 呼叫
- **`arm_control_loop()`**: 獨立的 **arm control thread** 在 1000Hz 執行

#### 資料流 (使用 mutex 同步)

```
Controller Thread (100Hz)          Arm Control Thread (1000Hz)
─────────────────────              ──────────────────────────
write() 
  └─ 寫入 arm_pos_cmd_buffer_  ──┐
     (透過 arm_command_mutex_)   │
                                 ├──> arm_control_loop() 讀取
                                 │      ├─ 讀取 CAN 馬達狀態
                                 │      ├─ 計算 gravity compensation
                                 │      ├─ 計算 friction compensation
                                 │      └─ CAN 發送 MIT 控制指令
                                 │
                                 │    arm_control_loop() 更新狀態
                                 │      └─ 寫入 arm_pos_state_buffer_
                                 │
read()                           │
  └─ 讀取 arm_pos_state_buffer_ <┘
     (透過 arm_state_mutex_)
```

**關鍵程式碼位置**:
- `write()`: Line 499-523 - 只做記憶體拷貝
- `read()`: Line 472-497 - 只做記憶體拷貝
- `arm_control_loop()`: Line 770-998 - 實際硬體通訊

### 2. Right 和 Left arm 是同一個 thread 嗎?

**否**，它們是**完全獨立的 threads**:

在 **bimanual 模式**下會啟動 **2 個獨立的 hardware interface instances**:

```yaml
# openarm_v10_bimanual_controllers.yaml
hardware:
  - name: left_openarm_v10
    type: openarm_hardware/OpenArm_v10HW
    # 有自己的 arm_control_thread_
    
  - name: right_openarm_v10
    type: openarm_hardware/OpenArm_v10HW
    # 有自己的 arm_control_thread_ 和 leap_control_thread_
```

每個 instance 各自管理:
- ✅ 獨立的 CAN bus 連接
- ✅ 獨立的 control loop thread (1000Hz)
- ✅ 獨立的 mutex (arm_command_mutex_, arm_state_mutex_)
- ✅ 獨立的 命令/狀態 buffers
- ✅ 獨立的 CSV debug 檔案

**關鍵程式碼位置**:
- Thread 啟動: Line 407 (`arm_control_thread_ = std::thread(...)`)
- Thread 停止: Line 432-435 (on_deactivate)

### 3. LEAP Hand read 和 write 是同一個 thread 嗎?

**否**，同樣是**不同的 threads**:

- **`read()` 和 `write()`**: 由 **controller_manager thread** 在 100Hz 呼叫
- **`leap_control_loop()`**: 獨立的 **LEAP control thread** 在 100Hz 執行

#### 資料流

```
Controller Thread (100Hz)          LEAP Control Thread (100Hz)
─────────────────────              ───────────────────────────
write()
  └─ 寫入 leap_pos_cmd_buffer_  ──┐
     (透過 leap_command_mutex_)   │
                                  ├──> leap_control_loop() 讀取
                                  │      ├─ URDF → LEAP 座標轉換
                                  │      ├─ Serial 發送 SyncWrite
                                  │      └─ Serial 接收 SyncRead
                                  │
                                  │    leap_control_loop() 更新狀態
                                  │      ├─ LEAP → URDF 座標轉換
                                  │      └─ 寫入 leap_pos_state_buffer_
                                  │
read()                            │
  └─ 讀取 leap_pos_state_buffer_ <┘
     (透過 leap_state_mutex_)
```

**關鍵程式碼位置**:
- `write()`: Line 515-523 - 只做記憶體拷貝
- `read()`: Line 486-495 - 只做記憶體拷貝
- `leap_control_loop()`: Line 999-1148 - 實際硬體通訊
- Thread 啟動: Line 413 (`leap_control_thread_ = std::thread(...)`)

## 完整架構總結

### Bimanual System = 6 個 Threads

```
┌─────────────────────────────────────────────────────────────┐
│            Controller Manager (100Hz)                       │
│  ┌──────────────────────┐  ┌──────────────────────┐        │
│  │ Left Controller      │  │ Right Controller     │        │
│  │ Thread (100Hz)       │  │ Thread (100Hz)       │        │
│  └──────────────────────┘  └──────────────────────┘        │
└─────────────────────────────────────────────────────────────┘
           │                              │
           │ read()/write()               │ read()/write()
           │ (mutex-protected)            │ (mutex-protected)
           ↓                              ↓
┌────────────────────────┐    ┌────────────────────────┐
│  Left Arm Instance     │    │  Right Arm Instance    │
├────────────────────────┤    ├────────────────────────┤
│ • Controller Thread    │    │ • Controller Thread    │
│   └─ read/write @100Hz │    │   └─ read/write @100Hz │
│                        │    │                        │
│ • Arm Control Thread   │    │ • Arm Control Thread   │
│   └─ CAN @1000Hz       │    │   └─ CAN @1000Hz       │
│     ├─ Gravity comp    │    │     ├─ Gravity comp    │
│     ├─ Friction comp   │    │     ├─ Friction comp   │
│     └─ MIT control     │    │     └─ MIT control     │
│                        │    │                        │
│                        │    │ • LEAP Control Thread  │
│                        │    │   └─ Serial @100Hz     │
│                        │    │     ├─ 16 Dynamixels   │
│                        │    │     └─ SyncRead/Write  │
└────────────────────────┘    └────────────────────────┘
         │                              │         │
         ↓                              ↓         ↓
    CAN Bus 1                      CAN Bus 0   Serial Port
    (Left Arm)                     (Right Arm)  (LEAP Hand)
```

### Thread 頻率與用途

| Thread | 頻率 | 用途 | 通訊方式 |
|--------|------|------|----------|
| Controller Thread | 100Hz | JointTrajectoryController 執行 | N/A |
| Arm Control Thread | 1000Hz (實際 ~500-800Hz) | OpenArm 馬達控制 + 補償 | CAN Bus |
| LEAP Control Thread | 100Hz | LEAP Hand 16 個馬達控制 | Serial (Dynamixel Protocol) |

### 關鍵設計原則

1. **分離控制與通訊**
   - `read()/write()` 只做記憶體拷貝 (快速、無阻塞)
   - 實際硬體通訊在獨立高頻 threads 執行

2. **Thread-Safe 資料交換**
   - 使用 `std::mutex` 保護共享 buffers
   - Command buffer: Controller → Hardware Thread
   - State buffer: Hardware Thread → Controller

3. **頻率解耦**
   - Controller 100Hz 與 Hardware Thread 1000Hz 獨立運作
   - 允許不同頻率的硬體共存 (Arm 1000Hz, LEAP 100Hz)

4. **雙臂獨立性**
   - 每個臂有獨立的 hardware interface instance
   - 完全隔離的資源 (CAN bus, threads, buffers)

## 程式碼關鍵位置

### Thread 管理
```cpp
// Line 407: 啟動 Arm Control Thread
arm_control_thread_ = std::thread(&OpenArm_v10HW::arm_control_loop, this);

// Line 413: 啟動 LEAP Hand Control Thread
leap_control_thread_ = std::thread(&OpenArm_v10HW::leap_control_loop, this);

// Line 432-443: 停止所有 threads (on_deactivate)
if (arm_thread_running_) {
  arm_thread_running_ = false;
  if (arm_control_thread_.joinable()) {
    arm_control_thread_.join();
  }
}
```

### Mutex 保護
```cpp
// Line 476-483: read() 讀取 arm 狀態
{
  std::lock_guard<std::mutex> lock(arm_state_mutex_);
  for (size_t i = 0; i < arm_size; ++i) {
    pos_states_[i] = arm_pos_state_buffer_[i];
  }
}

// Line 503-510: write() 寫入 arm 指令
{
  std::lock_guard<std::mutex> lock(arm_command_mutex_);
  for (size_t i = 0; i < arm_size; ++i) {
    arm_pos_cmd_buffer_[i] = pos_commands_[i];
  }
}
```

### 控制迴圈
```cpp
// Line 770: Arm Control Loop (1000Hz)
void OpenArm_v10HW::arm_control_loop() {
  const auto loop_period = microseconds(1000);  // 1ms
  while (arm_thread_running_) {
    // 1. 讀取 command buffer
    // 2. CAN 通訊讀取狀態
    // 3. 計算補償 (gravity + friction)
    // 4. CAN 通訊發送指令
    // 5. 更新 state buffer
  }
}

// Line 999: LEAP Control Loop (100Hz)
void OpenArm_v10HW::leap_control_loop() {
  const auto loop_period = milliseconds(10);  // 10ms
  while (leap_thread_running_) {
    // 1. 讀取 command buffer
    // 2. Serial SyncWrite 發送指令
    // 3. Serial SyncRead 讀取狀態
    // 4. 更新 state buffer
  }
}
```

## CSV Debug 輸出

每個 thread 都會產生獨立的 CSV 檔案:

```
<package_share_dir>/debug_csvs/YYYYMMDD/
├── debug_left_YYYYMMDD_HHMMSS.csv      # Left Arm (5Hz sampling)
├── debug_right_YYYYMMDD_HHMMSS.csv     # Right Arm (5Hz sampling)
└── debug_leap_hand_YYYYMMDD_HHMMSS.csv # LEAP Hand (100Hz sampling)
```

**Arm CSV 格式** (Line 918-920):
```
timestamp,joint_id,pos_cmd,vel_cmd,tau_cmd,pos_state,vel_state,tau_state,
pos_error,vel_error,gravity_comp,friction_comp,software_feedback,feedforward_tau,kp,kd
```

**LEAP Hand CSV 格式** (Line 1056-1057):
```
timestamp,motor_id,pos_cmd_urdf,pos_cmd_leap,pos_state_urdf,
pos_state_leap,pos_error_urdf,motor_name
```

## 效能特性

### Arm Control Thread (1000Hz target)
- **理論週期**: 1000 μs (1 ms)
- **實際週期**: ~1250-2000 μs (500-800 Hz)
- **瓶頸**: CAN 通訊延遲 + 重力/摩擦補償計算

### LEAP Control Thread (100Hz)
- **週期**: 10 ms
- **穩定性**: 高 (Serial 通訊較 CAN 穩定)
- **Sync I/O**: 16 個馬達使用 SyncRead/Write 批次處理

### Controller Thread (100Hz)
- **週期**: 10 ms
- **任務**: Trajectory interpolation, PID control
- **非阻塞**: `read()/write()` 只做記憶體操作 (<1 μs)

## 故障處理

### Thread 安全關閉
```cpp
// Line 424-443: on_deactivate()
1. Return to zero position
2. Stop arm_control_thread_ (set flag + join)
3. Stop leap_control_thread_ (set flag + join)
4. Disconnect LEAP Hand
5. Disable all motors
6. Close CSV files
```

### CAN 通訊錯誤
- Arm thread 會繼續運行但記錄錯誤
- 不會影響 LEAP Hand thread

### Serial 通訊錯誤
- LEAP thread 會繼續運行但記錄錯誤
- 不會影響 Arm thread

## 參考資料

- **JointTrajectoryController**: [ros2_controllers](https://github.com/ros-controls/ros2_controllers)
- **ROS2 Control**: [ros2_control](https://github.com/ros-controls/ros2_control)
- **Dynamixel SDK**: [DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK)
- **OpenArm SDK**: Internal CAN protocol implementation
