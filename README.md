# 無人機地面控制系統

本系統以 PlayStation 手把透過藍牙控制無人機飛行，並搭載 VL53L0X 雷射測距感測器實現定高飛行，以及夾爪機構進行空中作業。

> 📖 **第一次組裝？** 請先閱讀 [SETUP_GUIDE.md](SETUP_GUIDE.md)，從 Arduino Nano 開箱到整機整合測試均有詳細說明。

> 📊 **零件規格試算表**：[drone parts data](https://docs.google.com/spreadsheets/d/1BVyKEbavxxj2lV3zZNN0sOg7pvSjEQ_-sjfodHf1z0o/)

---

## 系統架構

```
[PS 手把] --藍牙--> [電腦 main_althold_v3_velmode.py] --藍牙(COM4)--> [Arduino] --IBUS--> [飛控 F405]
                                                       <--高度/PID回傳--
                                                                        |
                                                                        +--PWM--> [夾爪伺服馬達 D6]
                                                                        |
                                                                        +--I2C--> [VL53L0X 測距感測器]
```

| 元件 | 說明 |
|------|------|
| PS 手把 | 透過藍牙與電腦配對，作為輸入裝置 |
| 電腦 (`main_althold_v3_velmode.py`) | 讀取手把輸入，計算速度命令，打包成 10-byte 封包透過藍牙傳給 Arduino；接收高度回傳 |
| Arduino (`sketch_althold_v6_velmode.ino`) | 接收指令，轉換 IBUS 送給飛控；執行速度 PID 定高；回傳高度與 PID 數據 |
| VL53L0X | 雷射測距感測器，提供即時高度資料給 Arduino PID 控制器 |
| 飛控 F405 | 接收 IBUS 訊號控制四軸馬達飛行 |

---

## 硬體需求

- PlayStation 手把（藍牙），或使用鍵盤替代
- Arduino Nano（HC-05 藍牙模組接 D10 RX / D11 TX，含降壓電路）
- VL53L0X 雷射測距感測器（I2C 接 A4/A5）
- 夾爪伺服馬達（接 Arduino D6）
- F405 飛控板
- Windows 電腦（已安裝 Python 3）

---

## 環境安裝

```bash
py -3.11 -m pip install --upgrade pip setuptools wheel
py -3.11 -m pip install pygame pyserial keyboard
```

如果 `py -3.11 -m pip` 無法執行，先重新安裝 Python（建議使用 python.org 的 Windows x64 installer，不要只用 Microsoft Store 版本），並勾選 `Add python.exe to PATH`。

將 `sketch_althold_v6_velmode.ino` 燒錄到 Arduino（需先安裝 **VL53L0X by Pololu** 函式庫）。

---

## 啟動前設定

COM 埠可用 `--port` 參數指定（預設 COM4）：

```bash
python main_althold_v3_velmode.py --port COM6
```

或修改 `main_althold_v3_velmode.py` 最上方的預設值：

```python
parser.add_argument('--port', default='COM4', ...)
```

> 若不確定埠號，請至裝置管理員查詢「藍牙串列連接埠（傳出）」。

---

## 啟動方式

```bash
python main_althold_v3_velmode.py
```

程式啟動時**自動偵測手把**：
- 偵測到手把 → 手把模式
- 未偵測到手把 → 鍵盤模式（全域輸入，不需點選視窗）

啟動時若藍牙尚未連線，程式會每 3 秒自動重試，最多等待 60 秒。

---

## 控制按鍵說明

### 飛行控制（手把搖桿）

| 搖桿 | 功能 |
|------|------|
| 左搖桿 上/下 | 油門 Throttle |
| 左搖桿 左/右 | 偏航 Yaw |
| 右搖桿 上/下 | 俯仰 Pitch |
| 右搖桿 左/右 | 翻滾 Roll |

### 飛行控制（手把按鍵）

| 按鍵 | 手動模式 | 速度定高模式 |
|------|----------|-------------|
| 方向鍵 上 / 下 | 基準油門 ± 步進 | （無效）|
| 方向鍵 左 / 右 | 步進量 縮小/增大 | （無效）|
| **○（Circle）** | 切換定高（開啟）| 切換定高（關閉）|
| **□（Square）** | 搖桿校準歸零 | 積分重設（重新收斂懸停油門）|
| **△（Triangle）** | **解鎖 / 上鎖** | **解鎖 / 上鎖** |
| **Options** 長按 3 秒 | 安全關閉程式 | 安全關閉程式 |
| **4 + 6 同時按** | 🚨 緊急停機 | 🚨 緊急停機 |

### 飛行控制（鍵盤）

| 按鍵 | 手動模式 | 速度定高模式 |
|------|----------|-------------|
| `W` / `S` | 油門增減 | 爬升 / 下降速率 |
| `A` / `D` | 偏航 | 偏航 |
| `↑` / `↓` | 俯仰 | 俯仰 |
| `←` / `→` | 翻滾 | 翻滾 |
| `Tab` / `Shift` | 基準油門 上/下 | （無效）|
| `C` / `Z` | 步進量 增大/縮小 | — |
| `F` | — | 積分重設 |
| `H` | 切換定高（開啟）| 切換定高（關閉）|
| `R` | 解鎖 / 上鎖 | 解鎖 / 上鎖 |
| `X` 長按 3 秒 | 安全退出 | 安全退出 |

### 夾爪控制

| 手把 | 鍵盤 | 功能 |
|------|------|------|
| **L1** | `Q` | 夾爪閉合 |
| **R1** | `E` | 夾爪張開 |

---

## 終端機狀態顯示說明

```
[連線中] 解鎖 | 定高:80cm PID:142 | 當前:79.3cm | 基準:142 | 夾爪:127 | Y:127 P:127 R:127
```

| 欄位 | 說明 |
|------|------|
| 連線中 / 斷線中 | 藍牙目前連線狀態 |
| 解鎖 / 上鎖 | 飛控解鎖狀態 |
| 手動:xxx / 定高:xxxcm | 當前模式與油門（定高時顯示目標高度）|
| PID:xxx | 定高模式下 Arduino PID 實際輸出油門（0~255）|
| 當前:xxxcm | VL53L0X 回傳的即時高度 |
| 基準 | 當前基準油門（0~255）|
| 夾爪 | 夾爪開合程度（0 全閉 ~ 255 全開）|
| Y / P / R | 偏航 / 俯仰 / 翻滾（0~255，128 為中立）|

---

## 標準飛行流程

1. 確認 Arduino 已上電並與電腦藍牙配對
2. PS 手把透過藍牙與電腦配對
3. 執行 `python main_althold_v3_velmode.py`
4. 等待終端機顯示連線成功
5. **搖桿置中**，按下 **□** 進行搖桿校準（可選）
6. 確認油門歸零後，按 **△**（或 `R`）解鎖
7. 慢慢推油門起飛至穩定懸停高度
8. 按 **○**（或 `H`）切入速度模式定高；左搖桿置中 = 懸停，推上/下 = 爬升/下降
9. 按 **○** 退出定高，手動降落後油門歸零按 **△** 上鎖
10. 長按 **Options 3 秒**（或 `X`）安全關閉程式

> ⚠️ **安全提醒**：解鎖前油門值必須 ≤ 5，否則無法解鎖。  
> 🚨 **緊急狀況**：手把同時按下按鍵 4+6 可立即停機。

---

## 偵錯工具

### 測試手把按鍵編號

```bash
python tests/test_joy.py
```

按下手把按鍵後終端機會顯示對應編號，再填入 `main_althold_v3_velmode.py` 的按鍵定義區。

### 測試藍牙通訊

```bash
python tests/test_bt.py
```

### 監看 IBUS 封包

```bash
python tests/test_ibus.py
```

正常輸出範例：
```
📦 成功攔截 IBUS! | 翻滾:1500 | 俯仰:1500 | 油門:1000 | 偏航:1500 | 夾爪:1000
```

---

## 進階參數調整

開啟 `main_althold_v3_velmode.py` 最上方：

```python
JOYSTICK_SENSITIVITY = 60   # 手動模式搖桿推滿時的額外油門推力
TILT_SENS    = 0.3          # 俯仰 / 翻滾靈敏度（0.0 ~ 1.0）
YAW_SENS     = 0.25         # 偏航旋轉靈敏度（0.0 ~ 1.0）
STEP_SPEED   = 5            # 夾爪開合速度
ALT_VEL_SCALE = 60          # 速度模式搖桿全推對應最大速率（cm/s）
ALT_MAX_CM   = 300          # 高度上限（cm，超過時 Arduino 強制往下）
```

PID 係數在 `sketch_althold_v6_velmode.ino`：

```cpp
const float Kp_vel           = 0.80f;
const float Ki_vel           = 1.20f;
const float Kd_vel           = 0.0f;
const float MAX_ALT_CM       = 300.0f;  // 超過此高度強制 vel_cmd ≤ −5 cm/s
const float ALPHA            = 0.35f;   // 感測器 EMA 濾波
const float GAMMA            = 0.20f;   // 速度估計 EMA
```

---

## 通訊協議說明（開發參考）

### 電腦 → Arduino（藍牙，9600 baud）

每 40ms 發送一次，共 10 bytes：

```
[ 'S' | Throttle | Yaw | Pitch | Roll | TargetAlt | Gripper | Arm | AltHold | XOR ]
```

所有數值範圍皆為 0~255。AltHold：1 = 定高開啟，0 = 手動。XOR 為前 8 bytes 的 XOR checksum。

### Arduino → 電腦（藍牙，9600 baud）

| 格式 | 說明 | 發送時機 |
|------|------|----------|
| `D:xxxx\n` | 目前高度（mm） | 每 200ms，感測器有效時 |
| `P:xxxx\n` | PID 實際油門（IBUS 1000~2000） | 定高模式中，每 200ms |
| `T:xxxx\n` | 定高結束時最後 PID 油門 | 關閉定高時一次 |

### Arduino → 飛控（IBUS，115200 baud）

標準 32-byte IBUS 封包，每 20ms 發送一次：

| 通道 | 功能 | 範圍 |
|------|------|------|
| CH1 | 翻滾 Roll | 1000 ~ 2000 |
| CH2 | 俯仰 Pitch | 1000 ~ 2000 |
| CH3 | 油門 Throttle（手動）/ PID 輸出（定高）| 1000 ~ 2000 |
| CH4 | 偏航 Yaw | 1000 ~ 2000 |
| CH5 | 解鎖 Arm | 1000 / 2000 |
| CH6 | 飛行模式（固定自穩）| 2000 |

---

## 檔案說明

| 檔案 | 說明 |
|------|------|
| `main_althold_v3_velmode.py` | 主控制程式（手把 / 鍵盤自動切換，速度模式定高）|
| `pid_logger.py` | PID 數據記錄模組（可選，記錄至 `vel_logs/`）|
| `plot_pid.py` | PID 數據視覺化工具 |
| `sketch_althold_v6_velmode/sketch_althold_v6_velmode.ino` | Arduino 韌體（IBUS 轉換 + 速度 PID + 高度回傳）|
| `tests/test_bt.py` | 藍牙通訊測試工具 |
| `tests/test_joy.py` | 手把按鍵編號測試工具 |
| `tests/test_ibus.py` | IBUS 封包監看工具（debug 用）|
| `vel_logs/` | 速度模式 PID 數據記錄（CSV，自動產生）|
| `SETUP_GUIDE.md` | 從零開始的完整組裝與設定指南 |
| `PID_TUNING.md` | 速度模式 PID 調參說明與診斷指南 |
| `velmode_guide.md` | 速度模式操作指南與架構說明 |
| `archive/` | 舊版程式備份（含舊定高 v2/v3/v5 韌體）|
