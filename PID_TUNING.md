# 定高 PID 調整指南（v0.4.20）

## 現行架構：串級 PID（sketch_althold_v5_cascade）

v0.4.20 起改用**串級（Cascade）PID**：外環控制位置、內環控制速度。

```
目標高度 → [外環 P] → 目標速度 → [內環 PID] → 油門偏移 → base_throttle + offset → IBUS
```

---

## 目前參數

```cpp
// 外環（位置）
const float Kp_pos           = 0.8f;    // 位置誤差 → 目標速度（cm/s per cm）

// 內環（速度）
const float Kp_vel           = 0.60f;
const float Ki_vel           = 0.40f;
const float Kd_vel           = 0.25f;

// 限制
const float MAX_VEL_CMD      = 25.0f;   // 外環速度命令上限 (cm/s)
const float MAX_INTEGRAL_VEL = 85.0f;   // 內環積分上限
const float MAX_THR_OFFSET   = 185.0f;  // 總油門偏移上限 (IBUS)
const int   MAX_THR_STEP     = 10;      // 油門速率限制 (IBUS/20ms)

// 濾波
const float ALPHA            = 0.35f;   // 位置 EMA（比 v3 的 0.15 更快）
const float GAMMA            = 0.20f;   // 速度 EMA

// 保護
const float MAX_ALT_CM       = 90.0f;   // 超過此高度 target 夾到 90cm
const int   TAKEOFF_LIFTOFF_CM = 20;    // 離地偵測閾值（EMA 濾波後）
const int   TAKEOFF_MAX_IBUS   = 1627;  // 起飛爬坡上限
```

---

## 各參數直覺說明

| 參數 | 作用 | 過大 | 過小 |
|------|------|------|------|
| `Kp_pos` | 位置誤差轉速度命令 | 命令速度太猛，overshoots | 反應遲鈍 |
| `Kp_vel` | 速度誤差轉油門 | 高頻震盪 | 速度跟不上命令 |
| `Ki_vel` | 補償 hover_estimate 誤差 | 積分過衝 | 穩態誤差（飄離目標） |
| `Kd_vel` | 速度微分阻尼 | 放大感測器雜訊 | 速度超調後收不住 |
| `ALPHA` | 位置 EMA 平滑度 | 0.35 已偏快，再大雜訊增加 | 延遲增大，PID 反應慢 |
| `GAMMA` | 速度 EMA 平滑度 | 速度估計抖動 | 速度估計過度平滑，內環失真 |

---

## 抗積分飽和（Saturation Anti-windup）

v5 採用工業級**洩漏式 anti-windup**：

```cpp
if (fabs(tentative_offset) < MAX_THR_OFFSET * 0.85f) {
    integral_vel += vel_error * dt;        // 正常累積
} else if (vel_error * integral_vel > 0) { // 飽和且誤差同向 → 洩漏
    integral_vel *= 0.92f;
}
```

- 未飽和：正常累積
- 飽和且積分在「加劇」方向：每拍縮減 8%，防止積分爆炸後造成大過衝

---

## 起飛流程

1. 解鎖無人機
2. 按 ○ 切入定高（Python 傳入目標高度，預設 25cm）
3. Arduino 自動緩坡爬升（每 20ms +2 IBUS，上限 1627 IBUS）
4. EMA 高度超過 **20cm** 後快照爬坡油門為 `hover_estimate`，切換為串級 PID
5. PID 繼續爬升至目標高度

> `TAKEOFF_LIFTOFF_CM = 20`（比 v3 的 15 更晚切換，讓 hover_estimate 更接近真實懸停點）

---

## 調參建議順序

1. **Kp_pos 從 0.6 開始**：太小 → 追蹤慢；太大 → 速度命令跳動
2. **Kp_vel**：決定速度環的「硬度」，0.5–0.8 是安全範圍
3. **Ki_vel**：若目標設 25cm 但穩定在 18cm，表示 hover_estimate 偏低，加大 Ki_vel
4. **Kd_vel**：若速度超調（overshooting target velocity），加大；若雜訊大，縮小

---

## 診斷對照表

| 現象 | 診斷 | 建議調整 |
|------|------|---------|
| 穩定飛在目標以下 5–15cm | hover_estimate 偏低，Ki 補不足 | 加大 `Ki_vel`（0.40 → 0.55） |
| 到目標後繼續衝過去（overshoot） | `Kp_pos` 命令速度太大 | 降低 `Kp_pos` 或 `MAX_VEL_CMD` |
| 高度震盪且越來越大 | `Kp_vel` 過大或速度環不穩 | 降低 `Kp_vel`，加大 `Kd_vel` |
| 高度震盪但不發散 | 阻尼不足 | 加大 `Kd_vel` |
| 目標改變後追得很慢 | `Kp_pos` 或 `MAX_VEL_CMD` 太小 | 加大兩者之一 |
| 油門高頻跳動 | 速度估計雜訊大 | 降低 `GAMMA`（速度更平滑）或降低 `Kd_vel` |
| 起飛後立刻掉回地面 | `hover_estimate` 捕捉太低（爬坡太短） | 調高 `TAKEOFF_LIFTOFF_CM` 或降低 `ALPHA` |

---

## 修改位置

`sketch_althold_v5_cascade/sketch_althold_v5_cascade.ino`，第 56–71 行的係數區塊。

每次修改後需重新燒錄。

---

## 調參歷史摘要

| 版本 | 架構 | 問題 |
|------|------|------|
| v3 初版 | 單環 PID（P+I+D on 位置） | D 項符號錯誤（上升時加油而非煞車） |
| v3 修正 | 單環 PID，D 符號修正 | Rate limiter 造成 D 項無法即時響應，等效 3 階系統，過阻尼無法達成 |
| v3 移除 rate limiter | 單環 PID，MAX_CORRECTION=80 | hover_estimate 不足時（25cm→50cm hover 差 50 IBUS），MAX_CORRECTION 不夠用 |
| v3 最終 | 單環 PID，MAX_CORRECTION=150 | 在 60cm 附近發現不對稱振盪發散（地效過渡帶） |
| **v5 現行** | **串級 PID（位置 P + 速度 PID）** | rate limiter 重新加回但衝擊較小（內環速度回授提供真實阻尼） |
