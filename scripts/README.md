# LEAP Hand 轉換工具

這個目錄提供 `tool.py`，用於 LEAP Hand **motor joint** 與 **ROS/URDF position** 的轉換。

## 轉換公式
- **joint → position**: $position = joint - \pi$
- **position → joint**: $joint = position + \pi$

## 使用方式

### 1) joint 轉 position
```bash
python3 scripts/tool.py --joints "3.14 3.14 3.14 3.14 3.14 3.14 3.14 3.14 3.14 3.14 3.14 3.14 3.14 3.14 3.14 3.14"
```

### 2) position 轉 joint
```bash
python3 scripts/tool.py --positions "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0"
```

### 3) 使用內建 open 範例（16 個 3.14）
```bash
python3 scripts/tool.py --example-open
```

## 參數
- `--offset`: 指定 offset（預設 $\pi$）
- `--precision`: 輸出小數位數（預設 3）

## 輸出格式
- 會同時輸出 Python list 與 JSON list
