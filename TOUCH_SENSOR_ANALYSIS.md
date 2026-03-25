# O6 Touch Sensor Data Structure Analysis

## Summary
Python SDK and C++ SDK use **completely different** implementations for O6 touch sensors.

## Python SDK (Working Implementation)

### Data Structure
- **Matrix Dimensions**: 10 rows × 4 columns per finger
- **Data Type**: numpy array, converted to list for publishing
- **Initial Value**: -1 (uninitialized)

### CAN Protocol (Direct Communication)
```python
# Send command to request touch data
self.send_frame(0xb1, [self.touch_code], sleep=0.002)  # Thumb
self.send_frame(0xb2, [self.touch_code], sleep=0.002)  # Index
self.send_frame(0xb3, [self.touch_code], sleep=0.002)  # Middle
self.send_frame(0xb4, [self.touch_code], sleep=0.002)  # Ring
self.send_frame(0xb5, [self.touch_code], sleep=0.002)  # Pinky
```

### Response Format
Each CAN frame returns 5 bytes:
```
[row_index, val1, val2, val3, val4]
```

**Row Index Mapping** (matrix_map):
| Byte Value | Row Index | Usage |
|------------|-----------|-------|
| 0          | 0         | ✓ O6 uses |
| 16         | 1         | ✓ O6 uses |
| 32         | 2         | ✓ O6 uses |
| 48         | 3         | ✓ O6 uses |
| 64         | 4         | ✓ O6 uses |
| 80         | 5         | ✓ O6 uses |
| 96         | 6         | ✓ O6 uses |
| 112        | 7         | ✓ O6 uses |
| 128        | 8         | ✓ O6 uses |
| 144        | 9         | ✓ O6 uses |
| 160        | 10        | ✗ Unused for O6 |
| 176        | 11        | ✗ Unused for O6 |

**O6 uses rows 0-9 (10 rows total)**

### Update Logic
```python
# When CAN frame 0xb1 response received:
d = list(response_data)  # [index, val1, val2, val3, val4]
if len(d) == 5:
    index = self.matrix_map.get(d[0])  # Get row number
    if index is not None:
        self.thumb_matrix[index] = d[1:]  # Update 4 columns
```

### API Method
```python
def get_thumb_matrix_touch(self, sleep_time=0.002):
    self.send_frame(0xb1, [self.touch_code], sleep=sleep_time)
    return self.thumb_matrix  # Returns 10×4 numpy array
```

### Published JSON Format
```json
{
  "stamp": {
    "secs": 1234567890,
    "nsecs": 123456789
  },
  "thumb_matrix": [[v1, v2, v3, v4], [v1, v2, v3, v4], ...],  // 10 rows
  "index_matrix": [[...], ...],
  "middle_matrix": [[...], ...],
  "ring_matrix": [[...], ...],
  "little_matrix": [[...], ...]
}
```

## C++ SDK (Current Implementation)

### API Method
```cpp
std::vector<std::vector<std::vector<float>>> LinkerHandApi::getForce()
```

### Return Structure
```
5 fingers × 12 rows × 6 columns
```

### Issues
1. **Wrong dimensions**: Returns 12×6 instead of 10×4
2. **All zeros**: O6 hands return all 0 values
3. **SDK warning**: "LinkerHandApi : Currently only supports L25 !"
4. **No O6 support**: getForce() is designed for L25 (high-end model)
5. **Black box**: Precompiled library, cannot see CAN implementation

### Test Result
```bash
$ ./test_o6_touch
LinkerHandApi : Currently only supports L25 !
Connected: 1
Firmware version: 1.2.4.0.2.0.1
Thumb matrix (12x6):
Row 0: 0 0 0 0 0 0
Row 1: 0 0 0 0 0 0
...
Row 11: 0 0 0 0 0 0
```

## Comparison Table

| Aspect | Python SDK | C++ SDK |
|--------|-----------|---------|
| **Matrix Size** | 10×4 | 12×6 |
| **Implementation** | Direct CAN (0xb1-0xb5) | Unknown (black box) |
| **O6 Support** | ✓ Yes (native) | ✗ No (L25 only) |
| **Data Source** | CAN frames | getForce() API |
| **Real Data** | ✓ Can receive actual values | ✗ All zeros for O6 |
| **Code Access** | ✓ Open source | ✗ Precompiled .so |

## Conclusion

**The C++ SDK's `getForce()` method is incompatible with O6 hands.**

### Options:

1. **Option A: Implement Direct CAN in C++** (Recommended)
   - Port Python's CAN protocol (0xb1-0xb5) to C++
   - Use socket CAN API directly
   - Publish 10×4 matrices matching Python format
   - Full compatibility with Python SDK

2. **Option B: Keep Current Implementation**
   - Accept 12×6 format with all zeros
   - Document incompatibility with Python SDK
   - Wait for C++ SDK update from vendor

3. **Option C: Remove Touch Publishers**
   - Since O6 hardware has no sensors anyway
   - Avoid publishing meaningless zero data
   - Re-add when hardware with sensors is available

## Next Steps

**If choosing Option A (Direct CAN Implementation):**

1. Study `linker_hand_o6_can.py` CAN send/receive logic
2. Implement C++ SocketCAN communication
3. Parse 5-byte CAN responses into 10×4 matrices
4. Update `o6_hand_hardware.cpp` to use CAN instead of getForce()
5. Test with Python SDK running simultaneously for comparison

**Required C++ Libraries:**
- `<linux/can.h>` - CAN frame structures
- `<sys/socket.h>` - Socket communication
- Standard template library for matrices

**Estimated Effort:**
- 2-3 hours for CAN implementation
- 1 hour for matrix handling
- 1 hour for testing and validation
