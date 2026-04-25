# 速度模式 PID 調整指南（v0.6.0）

## 架構：單環速度 PID（sketch_althold_v6_velmode）

v0.6.0 改為**純速度模式**：Python 直接送速度命令，Arduino 只用速度 PID 追蹤。沒有位置外環，搖桿值就是目標速度。

```
Python 搖桿 → vel_cmd (cm/s) → [速度 PID] → 油門偏移 → hover_estimate + offset → IBUS
```

---

## 目前參數

```cpp
// 速度 PID（sketch_althold_v6_velmode.ino）
const float Kp_vel           = 0.80f;
const float Ki_vel           = 1.20f;
const float Kd_vel           = 0.0f;    // 速度模式不開 D，避免搖桿跳動暴衝

// 限制
const float MAX_VEL_CMD      = 60.0f;   // 速度命令上限 (cm/s)
const float MAX_INTEGRAL_VEL = 220.0f;
const float MAX_THR_OFFSET   = 220.0f;  // 總油門偏移上限 (IBUS)
const int   MAX_THR_STEP     = 20;      // 油門速率限制 (IBUS/20ms)

// 濾波
const float ALPHA            = 0.35f;   // 高度 EMA
const float GAMMA            = 0.20f;   // 速度 EMA

// 保護
const float MAX_ALT_CM       = 300.0f;  // 超過此高度強制 vel_cmd ≤ −5 cm/s
// IBUS 輸出範圍：1100 ～ 1750
```

Python 端（`main_althold_v3_velmode.py`）：

```python
ALT_VEL_SCALE = 60   # 搖桿全推 = 60 cm/s
```

---

## 各參數說明

| 參數 | 作用 | 過大 | 過小 |
|------|------|------|------|
| `Kp_vel` | 速度誤差 → 即時油門推力 | 高頻振盪 | 追速慢，跟不上命令 |
| `Ki_vel` | 補償懸停油門，長期維持速度 | 懸停時緩慢漂移（積分過衝） | 積分找不到懸停點，高度持續下沉 |
| `Kd_vel` | 速度微分阻尼（目前為 0） | 放大感測器雜訊，油門高頻跳動 | — |
| `GAMMA` | 速度估計平滑度 | 速度估計抖動，Kp 直接振盪 | 速度估計過遲鈍，PID 反應慢 |
| `ALPHA` | 高度 EMA 平滑度 | 雜訊傳入速度估計 | 高度讀值滯後，速度估計失真 |
| `MAX_THR_STEP` | 每 20ms 最大油門變化量 | 響應太猛 | 追不上速度命令，反應遲頓 |

---

## Ki_vel 的特殊角色

速度模式沒有目標高度做比較，**Ki 積分項就是懸停油門的自動尋找器**：

- 啟動時 `integral_vel = 0`，`hover_estimate` 是初始猜測值
- 若無人機緩慢下沉 → 速度誤差持續為正 → `integral_vel` 累積 → 油門升高 → 收斂至懸停
- `Ki_vel = 1.20` 比定高版（0.40）強，目的是讓積分快速找到懸停點

---

## 抗積分飽和（Anti-windup）

```cpp
if (fabs(tentative_offset) < MAX_THR_OFFSET * 0.85f) {
    integral_vel += vel_error * dt;        // 正常累積
} else if (vel_error * integral_vel > 0) { // 飽和且誤差同向 → 洩漏
    integral_vel *= 0.92f;                 // 每拍縮 8%，防積分爆炸
}
```

---

## hover_estimate 初始化邏輯

```cpp
hover_estimate = (manual_throttle >= 1250) ? manual_throttle : 1500;
```

- 手動油門合理（IBUS ≥ 1250，對應 base_throttle ≥ 63）→ 用那個值，收斂最快
- 手動油門太低（地面空轉，base_throttle ≈ 0）→ 用 1500 作 fallback

---

## 診斷對照表

| 現象 | 診斷 | 建議調整 |
|------|------|---------|
| 啟動後緩慢持續下沉，最終落地 | Ki 太小，積分追不上 | 加大 `Ki_vel`（1.20 → 1.50） |
| 啟動後油門飆高暴衝 | hover_estimate 初始值偏高，或 Kp 過大 | 確認啟動前手動油門合理；降低 `Kp_vel` |
| 懸停時高度緩慢週期性漂移（數秒） | 積分過衝後洩漏不夠快 | 降低 `Ki_vel`；或將洩漏係數 0.92 → 0.95 |
| 推搖桿有反應但追速慢 | Kp 偏低 | 加大 `Kp_vel`（0.80 → 1.00） |
| 高頻油門跳動（聽得到） | 速度估計雜訊大 | 降低 `GAMMA`（0.20 → 0.12）；確認 `Kd_vel=0` |
| PID 顯示卡在 ~25（IBUS 1100） | hover_estimate=1000，積分起點太低 | 啟動前先把手動油門調到 IBUS ≥ 1250 |

---

## 調參建議順序

1. **先確認懸停**：放開搖桿能穩定懸停（靠積分收斂），確認 Ki 夠大
2. **再調速度響應**：推搖桿爬升/下降夠快且不震盪，調整 Kp
3. **最後調平滑度**：若油門跳動，先降低 GAMMA，再考慮開 Kd（小值）

---

## 修改位置

- Arduino PID 參數：`sketch_althold_v6_velmode/sketch_althold_v6_velmode.ino`，第 47–59 行
- Python 速度上限：`main_althold_v3_velmode.py`，`ALT_VEL_SCALE`

每次修改 Arduino 參數後需重新燒錄。

---

## 調參歷史摘要

| 版本 | 架構 | 問題 |
|------|------|------|
| v3 | 單環位置 PID | D 項符號錯誤；rate limiter 造成等效三階系統過阻尼 |
| v5 | 串級 PID（位置 P + 速度 PID） | 飛上平台時位置外環讀到平台誤判高度，暴衝 |
| **v6 現行** | **純速度 PID** | 無位置外環，不受平台誤讀影響；放開搖桿靠積分懸停 |
