# 無人機地面控制系統（ESP32 版）

本系統以 PlayStation 手把透過藍牙控制無人機飛行，搭載 **VL53L1X 雷射測距**實現速度模式定高、**PMW3901 光流**做水平漂移記錄/補正，並以夾爪機構進行空中作業。控制核心為 **ESP32**（內建藍牙），飛控韌體使用 **Betaflight**（IBUS 接收 RC）。

> 📖 **第一次組裝？** 請先閱讀 [SETUP_GUIDE.md](SETUP_GUIDE.md)，從 ESP32 開箱到整機整合測試、供電 brownout 排查均有詳細說明。
>
> 📊 **零件規格試算表**：[drone parts data](https://docs.google.com/spreadsheets/d/1BVyKEbavxxj2lV3zZNN0sOg7pvSjEQ_-sjfodHf1z0o/)

---

## 系統架構

```
[PS 手把] --藍牙--> [電腦 main_esp32_velmode.py] --藍牙(SPP)--> [ESP32] --IBUS--> [Betaflight 飛控 F405] --> 四軸馬達
                                                 <--高度/光流/PID回傳--      |
                                                                           +--PWM--> [夾爪伺服馬達 GPIO13]
                                                                           +--I2C--> [VL53L1X 測距感測器]
                                                                           +--SPI--> [PMW3901 光流感測器]
```

| 元件 | 說明 |
|------|------|
| PS 手把 | 透過藍牙與電腦配對，作為輸入裝置（無手把可用鍵盤）|
| 電腦 (`main_esp32_velmode.py`) | 讀手把輸入 → 計算速度命令 → 打包 10-byte 封包經藍牙傳給 ESP32；接收高度/光流/PID 遙測並即時繪圖 |
| ESP32 (`ino_for_esp32/ino_for_esp32.ino`) | 收指令轉 IBUS 給飛控；讀 VL53L1X 跑速度 PID 定高；讀 PMW3901 光流；控制夾爪；藍牙回傳遙測 |
| VL53L1X | I2C 雷射測距，提供高度給 ESP32 PID |
| PMW3901 | SPI 光流，提供水平位移（dx/dy）給漂移記錄/補正 |
| Betaflight 飛控 F405 | 收 IBUS 訊號，以自穩（ANGLE）模式驅動四軸馬達 |

> 定高與水平補正皆由 **ESP32 自行運算**，不使用 Betaflight 的 ALTHOLD/POSHOLD。

---

## 硬體需求

- PlayStation 手把（藍牙），或用鍵盤替代
- **ESP32 開發板**（需藍牙 Classic/SPP，例如 ESP32-WROOM-32 DevKit）
- VL53L1X 雷射測距感測器（I2C，SDA=GPIO21 / SCL=GPIO22，VCC 3.3V）
- PMW3901 光流感測器（SPI，MOSI=23 / MISO=19 / SCK=18 / CS=5 / RST=26，VCC 3.3V）
- 夾爪伺服馬達（PWM，GPIO13）
- F405 飛控板（Betaflight）
- 降壓模組（飛控 BAT → 5V 供 ESP32，建議 ≥2A，**見供電注意事項**）
- Windows 電腦（已安裝 Python 3.11）

> ⚠️ **供電注意**：ESP32 與馬達共用同一顆電池時，裝槳催高馬力可能造成電壓驟降 → ESP32 brownout 重啟 → 藍牙斷線。詳見 [SETUP_GUIDE.md 第 31 章](SETUP_GUIDE.md#31--esp32-供電與-brownout-斷線排查)。

---

## 環境安裝

```bash
py -3.11 -m pip install --upgrade pip setuptools wheel
py -3.11 -m pip install pyserial pygame keyboard matplotlib
```

如果 `py -3.11 -m pip` 無法執行，請重裝 Python（建議 python.org 的 Windows x64 installer，勾選 `Add python.exe to PATH`，不要只用 Microsoft Store 版本）。

韌體燒錄：用 Arduino IDE 開啟 `ino_for_esp32/ino_for_esp32.ino`，先安裝 ESP32 開發板套件與函式庫（**VL53L1X by Pololu**、**Bitcraze PMW3901**、**ESP32Servo**），詳見 SETUP_GUIDE.md 第 2~9 章。

---

## 啟動前設定

COM 埠用 `--port` 指定（預設 COM9），須為 ESP32 藍牙的「**傳出（Outgoing）**」埠：

```bash
python main_esp32_velmode.py --port COM9
```

或改 `main_esp32_velmode.py` 上方預設值：

```python
parser.add_argument('--port', default='COM9', ...)
```

> 至裝置管理員 → 連接埠 查「標準串列藍牙連結」，**選傳出那個**。連到傳入埠會出現「能寫但收不到回傳」的假連線。

---

## 啟動方式

```bash
python main_esp32_velmode.py
```

程式啟動**自動偵測手把**：有手把 → 手把模式；沒有 → 鍵盤模式（全域輸入，不需點選視窗）。
啟動時若藍牙未連線，會每 3 秒自動重試，最多等 60 秒。視窗左側為狀態面板，右側為即時圖表（光流 dx/dy、量級、高度、累積路徑、FC 平均馬力）。

---

## 控制按鍵說明

### 飛行控制（手把搖桿）

| 搖桿 | 功能 |
|------|------|
| 左搖桿 上/下 | 油門（手動）/ 爬升·下降速率（定高）|
| 左搖桿 左/右 | 偏航 Yaw |
| 右搖桿 上/下 | 俯仰 Pitch |
| 右搖桿 左/右 | 翻滾 Roll |

### 飛行控制（手把按鍵）

| 按鍵 | 手動模式 | 速度定高模式 |
|------|----------|-------------|
| 方向鍵 上 / 下 | 基準油門 ± 步進 | （無效）|
| 方向鍵 左 / 右 | 步進量 縮小/增大 | （無效）|
| **○（Circle）** | 切換定高（開啟）| 切換定高（關閉）|
| **□（Square）** | 搖桿校準歸零 | 搖桿校準歸零 |
| **△（Triangle）** | **解鎖 / 上鎖** | **解鎖 / 上鎖** |
| **按鍵 15** | 右搖桿偏移補正（水平 trim）| 右搖桿偏移補正（水平 trim）|
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
| `H` | 切換定高（開啟）| 切換定高（關閉）|
| `R` | 解鎖 / 上鎖 | 解鎖 / 上鎖 |
| `X` 長按 3 秒 | 安全退出 | 安全退出 |

### 夾爪控制

| 手把 | 鍵盤 | 功能 |
|------|------|------|
| **L1** | `Q` | 夾爪夾合（0°）|
| **R1** | `E` | 夾爪張開（70°）|

---

## 終端機狀態顯示說明

```
[連線中] 解鎖 | 定點定高/FC油門:142 | 高:79.3 | 基:142 | 爪:  2 | Y:127 P:127 R:127 | ...
```

| 欄位 | 說明 |
|------|------|
| 連線中 / 斷線中 | 藍牙連線狀態（以有無遙測判斷真實連線）|
| 解鎖 / 上鎖 | 飛控解鎖狀態 |
| 手:xxx / 定點定高 | 當前模式與油門（定高時顯示 FC 平均馬力換算）|
| 高:xxx | VL53L1X 回傳的即時高度（cm）|
| 基 | 當前基準油門（0~255）|
| 爪 | 夾爪指令（0 保持 / 1 夾 / 2 開）|
| Y / P / R | 偏航 / 俯仰 / 翻滾（0~255，128 中立）|

---

## 標準飛行流程

1. 接電池給整機供電，等 ESP32 藍牙與電腦連上（`ESP32_Drone_Hub`）
2. PS 手把與電腦配對
3. 執行 `python main_esp32_velmode.py --port COM9`
4. 等終端機顯示連線成功、狀態面板高度正常
5. **搖桿置中**，按 **□** 搖桿校準（可選）
6. 確認油門歸零後，按 **△**（或 `R`）解鎖
7. 慢慢推油門起飛至穩定懸停高度
8. 按 **○**（或 `H`）切入速度定高；左搖桿置中=懸停，推上/下=爬升/下降
9. 按 **○** 退出定高，手動降落後油門歸零按 **△** 上鎖
10. 長按 **Options 3 秒**（或 `X`）安全關閉程式

> ⚠️ **安全提醒**：解鎖前油門必須歸零；飛行前先確認供電不 brownout。
> 🚨 **緊急狀況**：手把同時按 4+6 立即停機。

---

## 偵錯工具

### 測試藍牙通訊（ESP32 診斷）

```bash
python tests/test_esp32_bt.py
```

會經 USB 確認 ESP32 是否在跑韌體、經藍牙確認是否回傳遙測，並區分「假連線/埠填錯/ESP32 沒跑」等情況。

### 測試手把按鍵編號

```bash
python tests/test_joy.py
```

按鍵後顯示對應編號，再填入 `main_esp32_velmode.py` 的按鍵定義區。

### 光流方向校正

```bash
python tests/test_flow_direction.py --port COM9
```

地面平移機體，自動判讀 `FLOW_ROLL_SIGN` / `FLOW_PITCH_SIGN` 該維持還是改號（只讀遙測、不送控制，建議拆槳）。

### 監看 IBUS 封包

```bash
python tests/test_ibus.py
```

---

## 進階參數調整

`main_esp32_velmode.py` 上方：

```python
JOYSTICK_SENSITIVITY = 60   # 手動模式搖桿推滿時的額外油門推力
TILT_SENS    = 0.3          # 俯仰 / 翻滾靈敏度
YAW_SENS     = 0.25         # 偏航靈敏度
ALT_VEL_SCALE = 60          # 定高搖桿全推對應最大速率（cm/s）
FLOW_ROLL_SIGN  = +1        # 光流 roll 補正符號（用 test_flow_direction 校正）
FLOW_PITCH_SIGN = -1        # 光流 pitch 補正符號
```

PID 與感測器濾波係數在 `ino_for_esp32/ino_for_esp32.ino`：

```cpp
const float Kp_vel  = 0.80f;
const float Ki_vel  = 1.20f;
const float Kd_vel  = 0.0f;
const float MAX_ALT_CM_PID = 250.0f;  // 超過此高度不再積分定高
const float ALPHA   = 0.35f;          // 高度 EMA 濾波
const float GAMMA   = 0.20f;          // 速度估計 EMA
```

> 光流自動補償（P+I）目前在程式中**停用**（已註解），僅手動右搖桿 trim（按鍵 15）生效。

---

## 通訊協議說明（開發參考）

### 電腦 → ESP32（藍牙，10 bytes）

```
[ 'S' | Throttle | Yaw | Pitch | Roll | AltVel | Gripper | Arm | AltHold | XOR ]
```

各值 0~255。`AltVel`：128=0 cm/s，每 1 = 1 cm/s。`Gripper`：1=夾 / 2=開 / 0=保持。`AltHold`：1=定高、0=手動。`XOR`=前 8 bytes 的 XOR checksum。

### ESP32 → 電腦（藍牙遙測）

| 格式 | 說明 | 時機 |
|------|------|------|
| `D:xxxx\n` | 高度（mm）| 每 200ms |
| `OF:dx,dy\n` | 光流增量 | 每 40ms（25Hz）|
| `OF:REINIT[_OK/_FAIL]\n` | 光流 watchdog | 觸發時 |
| `THR:xxxx\n` | 定高中 ESP32 PID 油門（IBUS）| 每 200ms |
| `T:xxxx\n` | 退出定高時最後油門 | 退出時一次 |
| `F:1` / `F:0` | 高度 Freeze 進入/恢復 | 觸發時 |

### ESP32 → 飛控（IBUS，115200，每 20ms）

標準 32-byte IBUS 封包：

| 通道 | 功能 | 範圍 |
|------|------|------|
| CH1 | 翻滾 Roll | 1000~2000 |
| CH2 | 俯仰 Pitch | 1000~2000 |
| CH3 | 油門 Throttle（手動）/ PID 輸出（定高）| 1000~2000 |
| CH4 | 偏航 Yaw | 1000~2000 |
| CH5 | 解鎖 Arm | 1000 / 2000 |
| CH6 | 飛行模式 ANGLE（固定）| 2000 |
| CH7 / CH8 | 固定（不使用飛控定高/定點）| 1000 |

---

## 檔案說明

| 檔案 | 說明 |
|------|------|
| `main_esp32_velmode.py` | 主控制程式（手把/鍵盤自動切換，速度模式定高，即時圖表）|
| `ino_for_esp32/ino_for_esp32.ino` | ESP32 韌體（IBUS 轉換 + 速度 PID 定高 + 光流 + 夾爪 + 遙測）|
| `pid_logger.py` / `vel_logs/` | PID 數據記錄（CSV，自動產生）|
| `flow_logger.py` / `flow_logs/` | 光流數據記錄（CSV，自動產生）|
| `plot_pid.py` | PID 數據視覺化 |
| `plot_flow.py` | 光流數據視覺化 |
| `flow_live.py` | 光流即時監看（獨立工具）|
| `tests/test_esp32_bt.py` | ESP32 藍牙收訊診斷 |
| `tests/test_joy.py` | 手把按鍵編號測試 |
| `tests/test_flow_direction.py` | 光流補償方向校正 |
| `tests/test_ibus.py` | IBUS 封包監看（debug）|
| `SETUP_GUIDE.md` | 從零開始的完整組裝與設定指南（ESP32 版）|
| `PID_TUNING.md` | 速度模式 PID 調參說明 |
| `velmode_guide.md` | 速度模式操作指南與架構說明 |
| `archive/` | 舊版程式備份（Arduino Nano 時期）|

> 舊的 Arduino Nano + HC-05 + VL53L0X 版本（`main_althold_v3_velmode.py`、`sketch_althold_v6_velmode/`）已停用，保留於倉庫供參考。
