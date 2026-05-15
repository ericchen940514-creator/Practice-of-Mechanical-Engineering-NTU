# 架構對照：v0.7（ESP32 定高）→ v0.8（iNAV 接管定高+定點）

---

## 一、整體架構對比

### v0.7（現在）
```
Python（地面站）
  │  藍牙封包：thr, yaw, pitch, roll, vel_cmd, grip, arm, ah_val
  ▼
ESP32
  ├─ 解析 ah_val → 切換 alt_hold_active
  ├─ alt_hold_active=ON  → 自己跑 PID，算出油門，送 iBUS CH3
  ├─ alt_hold_active=OFF → 直接把 thr 送 iBUS CH3
  ├─ MSP2 → iNAV（rangefinder + optical flow，純感測器）
  └─ iBUS → iNAV CH1-6（roll, pitch, thr, yaw, arm, angle）

iNAV
  └─ 只做姿態穩定（Angle mode），油門直接用 iBUS CH3
```

### v0.8（目標）
```
Python（地面站）
  │  藍牙封包：thr, yaw, pitch, roll, grip, arm, nav_mode
  ▼
ESP32
  ├─ 不再做 PID，直接把 thr 送 iBUS CH3
  ├─ nav_mode 旗標 → 控制 CH7（POSHOLD）、CH8（ALTHOLD+SURFACE）
  ├─ MSP2 → iNAV（rangefinder + optical flow）
  └─ iBUS → iNAV CH1-8

iNAV
  ├─ 姿態穩定（Angle mode，CH6）
  ├─ 水平定點（NAV POSHOLD，CH7，光流）
  ├─ 垂直定高（NAV ALTHOLD，CH8）
  └─ 貼地高度（SURFACE，CH8，VL53L1X）
```

---

## 二、iBUS Channel 對照

| CH | v0.7 | v0.8 |
|----|------|------|
| 1  | Roll | Roll（不變）|
| 2  | Pitch | Pitch（不變）|
| 3  | Throttle（ESP32 PID 或手動）| Throttle（手動，iNAV ALTHOLD 時搖桿控速率）|
| 4  | Yaw | Yaw（不變）|
| 5  | Arm | Arm（不變）|
| 6  | 固定 2000（Angle）| 固定 2000（Angle）|
| 7  | 固定 1500（閒置）| **nav_mode=1 → 2000（POSHOLD ON）；nav_mode=0 → 1000** |
| 8  | 固定 1500（閒置）| **nav_mode=1 → 2000（ALTHOLD+SURFACE ON）；nav_mode=0 → 1000** |

---

## 三、藍牙封包對照

### v0.7 封包（10 bytes）
```
'S' | thr | yaw | pitch | roll | alt_byte | grip | arm | ah_val | checksum
```
- `alt_byte`：速度命令（128=0 cm/s，>128=上，<128=下）
- `ah_val`：0=手動，1=定高ON，2=積分重設

### v0.8 封包（9 bytes，拿掉 alt_byte）
```
'S' | thr | yaw | pitch | roll | grip | arm | nav_mode | checksum
```
- `thr`：油門（iNAV ALTHOLD 開著時，這個值控制爬升/下降速率）
- `nav_mode`：0=純手動 Angle，1=POSHOLD+ALTHOLD+SURFACE

---

## 四、ESP32 程式碼改動

### 拿掉的部分
```cpp
// ── 全部刪除 ──
float v_est, integral_vel, last_vel_error;
float prev_cm_filt, prev_cm_raw;
int   hover_estimate;
bool  alt_hold_active;
float received_vel_cmd;

void resetAltHoldController() { ... }   // 整個刪
void exitAltHoldToManual() { ... }      // 整個刪
void updateAltHold() { ... }            // 整個刪

// loop() 裡的 alt hold 區塊
if (alt_hold_active) {
    updateAltHold(filtered_mm);         // 刪
}

// BT 回傳的 T: P: F:2
SerialBT.print("T:...\n");             // 刪
SerialBT.print("P:...\n");             // 刪
SerialBT.print("F:2\n");              // 刪（sensor stale 改成只關 nav mode）
```

### 加上的部分
```cpp
// BT 封包解析：ah_val 改成 nav_mode
int nav_mode = buf[7];  // 0=手動，1=NAV全開

// CH7 / CH8 根據 nav_mode 切換
ibus_channels[6] = (nav_mode == 1) ? 2000 : 1000;  // POSHOLD
ibus_channels[7] = (nav_mode == 1) ? 2000 : 1000;  // ALTHOLD + SURFACE

// thr 永遠直送，不再判斷 alt_hold_active
ibus_channels[2] = map(thr_val, 0, 255, 1000, 2000);
```

### PID 參數區塊（全部刪除）
```cpp
// 刪掉這整個區塊
const float Kp_vel           = 0.80f;
const float Ki_vel           = 1.20f;
const float Kd_vel           = 0.0f;
const float MAX_VEL_CMD      = 60.0f;
const float MAX_INTEGRAL_VEL = 220.0f;
const float MAX_THR_OFFSET   = 220.0f;
const float ALPHA            = 0.35f;
const float GAMMA            = 0.20f;
const float MAX_ALT_CM       = 300.0f;
const int   MAX_THR_STEP     = 20;
```

---

## 五、Python 程式碼改動

### 拿掉的部分
```python
# 常數
ALT_VEL_SCALE = 60
THROTTLE_EXPO = 0.75   # 不一定要拿，但定高時 expo 意義改變

# 函式
def encode_vel(vel_cm_s): ...   # 整個刪

# state 欄位
'alt_hold_active': False
'syncing_throttle': False
'syncing_throttle_t': 0.0
'reref_pending': False
'prev_circle': 0
'prev_sq': 0  # 如果 □ 改功能的話

# 按鍵邏輯
# ○/H：切換定高 → 改成切換 nav_mode
# □/F：積分重設 → 拿掉

# 記錄
pid_logger.start() / pid_logger.stop() / pid_logger.record()
```

### 改成的部分
```python
# state 新增
'nav_mode': 0,   # 0=純手動，1=POSHOLD+ALTHOLD

# ○/H 按鍵：切換 nav_mode
if curr_circle and not state['prev_circle']:
    state['nav_mode'] = 1 - state['nav_mode']
    print("NAV ON" if state['nav_mode'] else "NAV OFF")

# 封包
def make_packet(thr, yaw, pitch, roll, grip, arm, nav_mode):
    payload = bytes([thr, yaw, pitch, roll, grip, arm, nav_mode])
    ...

# 油門：不再分 alt_hold_active，統一一套邏輯
final_throttle = max(0, min(255,
    state['base_throttle'] + int(round(-raw_throttle * JOYSTICK_SENSITIVITY))))
```

### 顯示改動
```python
# 拿掉
"速度: {vel:+.1f} cm/s  PID: {pid_conv}"

# 改成
"NAV: {'POSHOLD+ALT' if state['nav_mode'] else '手動'}"
"高度: {alt_str}"
```

---

## 六、iNAV Modes 設定

| 模式 | Channel | 範圍 | 說明 |
|------|---------|------|------|
| ANGLE | AUX2（CH6）| 1800-2100 | 永遠開，固定送 2000 |
| NAV POSHOLD | AUX3（CH7）| 1800-2100 | nav_mode=1 時開 |
| NAV ALTHOLD | AUX4（CH8）| 1800-2100 | nav_mode=1 時開 |
| SURFACE | AUX4（CH8）| 1800-2100 | 同 CH8，疊加在 ALTHOLD 上 |

---

## 七、iNAV CLI 需要設定的參數

```
# 已設定
set inav_default_alt_sensor = BARO
set inav_allow_dead_reckoning = ON

# 還需要確認
set nav_mc_hover_thr = XXXX     ← 實際懸停油門，換機架後重測
set inav_max_surface_altitude = 200   ← SURFACE 模式有效範圍 200cm
set nav_disarm_on_landing = ON  ← 已開，降落後自動上鎖

# 可能需要調整
set nav_mc_pos_z_p = 50         ← 垂直定高 P gain
set nav_mc_vel_z_p = 100        ← 垂直速度 P gain
set inav_w_z_surface_p = 3.5    ← SURFACE 模式權重
set opflow_scale = 10.5         ← 光流比例，需實飛校正
```

---

## 八、注意事項

1. **油門中點**：iNAV ALTHOLD 開著時，油門搖桿中間是懸停，推上爬升，推下下降。`nav_mc_althold_throttle = STICK` 已是這個模式。

2. **SURFACE 有效範圍**：`inav_max_surface_altitude = 200`（cm），超過 200cm 高度會退回 baro 定高。室內飛行應該沒問題。

3. **救機**：nav_mode=0 時 CH7/CH8 送 1000，iNAV 退回純 Angle，你自己控制油門。這個按鍵要練到反射動作。

4. **opflow_scale 校正方法**：在固定高度（如 50cm）懸停，推 pitch 讓無人機前進固定距離，看實際移動距離和 iNAV 估算是否一致，調整 opflow_scale 直到吻合。

5. **換機架後**：重新測 `nav_mc_hover_thr`，方法是手動飛到懸停，記錄穩定時的油門值（Python 顯示的 base_throttle 換算回 1000-2000 區間）。
