# iNAV 設定復現指南
**硬體：ESP32 感測器 Hub + PMW3901 光流 + VL53L1X 測距**
**iNAV 版本：7.x**

---

## 系統架構

```
VL53L1X (I2C) ─┐
               ├─► ESP32 ──UART1(MSP2)──► FC UART3   iNAV 定高 / 定位
PMW3901 (SPI) ─┘        ◄─UART1(MSP)──── FC UART3   馬力查詢回傳

Python PC ──BT──► ESP32 ──UART2(iBUS)──► FC UART1   遙控輸入
```

**FC 完全不直接接感測器**，所有感測器資料都由 ESP32 打包成 MSP2 frame 後傳給 FC：
- `MSP2_SENSOR_RANGEFINDER (0x1F01)`：VL53L1X 測距值
- `MSP2_SENSOR_OPTIC_FLOW  (0x1F02)`：PMW3901 光流 dx/dy

---

## 接線總覽

### ESP32 ↔ FC

| ESP32 腳位 | 方向 | FC 腳位 | 用途 |
|-----------|------|---------|------|
| GPIO17 (UART2 TX) | → | UART1 RX | iBUS 遙控訊號 |
| GPIO25 (UART1 TX) | → | UART3 RX | MSP2 感測器資料（測距 + 光流） |
| GPIO34 (UART1 RX) | ← | UART3 TX | MSP motor 查詢回傳 |

### ESP32 ↔ 感測器

| ESP32 腳位 | 感測器 | 用途 |
|-----------|--------|------|
| GPIO21 (SDA) / GPIO22 (SCL) | VL53L1X | I2C 測距 |
| GPIO23 (MOSI) / GPIO19 (MISO) / GPIO18 (SCK) / GPIO5 (CS) | PMW3901 | SPI 光流 |
| GPIO26 | PMW3901 NRESET | 硬體重置腳 |

---

## iNAV Configurator 設定步驟

### 1. Ports 分頁

| UART | MSP | Serial RX | Sensor Input | Baud |
|------|-----|-----------|--------------|------|
| UART1 | OFF | **ON** | OFF | 115200 |
| UART3 | **ON** | OFF | **MSP** | 115200 |

> UART3 必須同時勾 **MSP** 和 **Sensor Input = MSP**。
> MSP 讓 FC 能回應 motor 查詢；Sensor Input 讓 FC 讀入感測器資料。

---

### 2. Receiver 分頁

- Receiver Mode：`Serial (SPST / SerialRX)`
- Serial Receiver Provider：`iBUS`

---

### 3. Configuration 分頁 → Features

勾選：
- [x] `RANGEFINDER`
- [x] `ALTITUDE HOLD`
- [x] `OPTICAL FLOW`

取消勾選：
- [ ] `GPS`
- [ ] `GPS RESCUE`（若有）

---

### 4. Sensors 分頁

確認 Configurator 即時顯示：
- Sonar（Rangefinder）：有數值更新（來自 VL53L1X，單位 cm）
- Optic Flow：dx / dy 有數值更新（來自 PMW3901）

---

### 5. Modes 分頁（Flight Modes）

| Channel | 低位（1000） | 高位（2000） |
|---------|------------|------------|
| AUX1（CH5） | 上鎖 | **ARM** |
| AUX2（CH6） | — | **Angle**（ino 固定送 2000，常開） |
| AUX3（CH7） | — | **ALTHOLD** |
| AUX4（CH8） | — | **POSHOLD** |

ESP32 ino 在 `ah_val=1` 時同時把 CH7、CH8 送 2000，兩個都要設。

---

### 6. CLI 設定

在 Configurator → CLI 依序貼入以下指令：

```
# ── 導航基本 ──
# ON：FC 以 1500 為懸停零點
# Python 端定高中固定送 127（≈1500），搖桿偏移才是爬升/下降指令
# 手動模式油門仍為 base_throttle + 搖桿，兩者邏輯分開互不干擾
set nav_use_midrc_for_althold = ON

# ── 關閉 GPS，只用雷射 + 氣壓計 ──
set inav_use_gps_no_baro = OFF

# ── 大幅降低氣壓計權重，主要依賴雷射測距 ──
set inav_w_z_baro_p = 0.05

# ── 雷射測距有效範圍（單位 cm），依 VL53L1X Medium mode ──
set nav_surface_min_active_cm = 5
set nav_surface_max_active_cm = 280

# ── 光流比例尺（PMW3901 1 count ≈ 0.01 rad，與 ino 一致） ──
set opflow_scale = 1.0

save
```

> **`inav_w_z_baro_p`**：預設 0.35，降到 0.05 代表定高幾乎完全靠雷射，
> 氣壓計只在雷射超出範圍時當備援。飛行後若定高抖動再往上微調。

---

## 驗證清單

開機後在 Configurator 做以下確認再試飛：

- [ ] Ports 頁存檔後重開，UART3 仍同時顯示 MSP + Sensor Input
- [ ] Sensors 頁：Sonar 有 cm 數值，隨手遮感測器會變化
- [ ] Sensors 頁：Optic Flow dx/dy 有數值，移動無人機會變化
- [ ] Modes 頁：CH7 / CH8 對應 ALTHOLD / POSHOLD
- [ ] 解鎖 → 觸發定高 → Serial Monitor 上印 `Throttle sync: xxx (MSP)` 而非 `(timeout)`
  - 若仍印 `timeout`，確認 UART3 的 MSP checkbox 有勾到

---

## 調參備忘

| 參數 | 預設 | 本機設定 | 說明 |
|------|------|---------|------|
| `inav_w_z_baro_p` | 0.35 | **0.05** | 氣壓計 Z 軸權重，降低讓雷射主導 |
| `opflow_scale` | 1.0 | 1.0（待測） | 光流計數轉換比例，飛行後比對位移微調 |
| `nav_surface_max_active_cm` | 200 | **280** | VL53L1X Medium mode ~3m |
| `ALPHA`（ino EMA） | — | 0.35 | 雷射 EMA 濾波係數，可在 ino 調整 |

---

## 常見問題

**Q：定高晃動嚴重**
→ 先確認 Sonar 數值穩定。若穩定，嘗試把 `inav_w_z_baro_p` 再降到 0.02，或調整 ino 的 `ALPHA`（降低 EMA 濾波）。

**Q：POSHOLD 持續漂移**
→ 調整 `opflow_scale`：實際位移比預期大 → 調小；比預期小 → 調大。

**Q：Throttle sync 一直印 timeout**
→ UART3 的 MSP checkbox 沒有勾，回 Ports 頁確認。

**Q：重開機後感測器沒數值**
→ 確認 ESP32 有正常啟動（BT 藍牙 `ESP32_Drone_Hub` 看得到），檢查 I2C / SPI 接線。
