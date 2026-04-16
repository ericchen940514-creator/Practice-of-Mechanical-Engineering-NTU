"""
plot_pid.py — 定高 PID 數據視覺化
用法：
  python plot_pid.py               # 自動載入最新一筆 CSV
  python plot_pid.py pid_logs/xxx.csv  # 指定檔案
"""
import sys
import os
import glob
import csv
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

# ── 載入檔案 ──
if len(sys.argv) >= 2:
    fpath = sys.argv[1]
else:
    files = sorted(glob.glob('pid_logs/althold_*.csv'))
    if not files:
        print("❌ 找不到 pid_logs/ 下的 CSV 檔案，請先飛行並開啟定高模式。")
        sys.exit(1)
    fpath = files[-1]
    print(f"📂 自動載入最新檔案：{fpath}")

# ── 讀取資料 ──
times, cur_alt, tgt_alt, pid_255 = [], [], [], []
with open(fpath, newline='', encoding='utf-8') as f:
    for row in csv.DictReader(f):
        try:
            times.append(float(row['time_s']))
            cur_alt.append(float(row['current_alt_cm']))
            tgt_alt.append(float(row['target_alt_cm']))
            pid_255.append(float(row['pid_thr_0_255']) if row['pid_thr_0_255'] else None)
        except (ValueError, KeyError):
            pass

if not times:
    print("❌ CSV 無有效資料。")
    sys.exit(1)

# ── 計算統計 ──
target = tgt_alt[0]
stable_start = max(0, int(len(times) * 0.6))   # 後 40% 視為穩態
steady_vals  = [c for c in cur_alt[stable_start:] if c is not None]
steady_err   = round(sum(steady_vals) / len(steady_vals) - target, 2) if steady_vals else 0
overshoot    = round(max((c - target for c in cur_alt), default=0), 2)
undershoot   = round(min((target - c for c in cur_alt), default=0), 2)

# ── 繪圖 ──
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 7), sharex=True,
                                gridspec_kw={'height_ratios': [2, 1]})
fig.suptitle(f'定高 PID 分析  —  {os.path.basename(fpath)}', fontsize=13)

# 上圖：高度
ax1.plot(times, cur_alt, color='#2196F3', linewidth=1.5, label='當前高度')
ax1.axhline(target, color='#F44336', linewidth=1.2, linestyle='--', label=f'目標高度 {target} cm')
ax1.fill_between(times, cur_alt, target,
                 where=[c > target for c in cur_alt], alpha=0.15, color='#F44336', label='超射')
ax1.fill_between(times, cur_alt, target,
                 where=[c < target for c in cur_alt], alpha=0.15, color='#2196F3', label='欠射')
ax1.set_ylabel('高度 (cm)')
ax1.legend(loc='upper right', fontsize=9)
ax1.yaxis.set_minor_locator(ticker.AutoMinorLocator())
ax1.grid(True, which='major', linestyle='-', alpha=0.3)
ax1.grid(True, which='minor', linestyle=':', alpha=0.2)

stats_text = (f'穩態誤差: {steady_err:+.2f} cm    '
              f'最大超射: {overshoot:.2f} cm    '
              f'最大欠射: {undershoot:.2f} cm')
ax1.text(0.01, 0.03, stats_text, transform=ax1.transAxes,
         fontsize=9, color='#555', verticalalignment='bottom')

# 下圖：PID 油門
valid_t   = [t for t, p in zip(times, pid_255) if p is not None]
valid_pid = [p for p in pid_255 if p is not None]
ax2.plot(valid_t, valid_pid, color='#4CAF50', linewidth=1.5, label='PID 油門 (0~255)')
ax2.set_ylabel('油門')
ax2.set_xlabel('時間 (s)')
ax2.legend(loc='upper right', fontsize=9)
ax2.yaxis.set_minor_locator(ticker.AutoMinorLocator())
ax2.grid(True, which='major', linestyle='-', alpha=0.3)
ax2.grid(True, which='minor', linestyle=':', alpha=0.2)

plt.tight_layout()
plt.show()
