"""
plot_pid.py — 定高 PID 數據視覺化（互動式）
用法：
  python plot_pid.py               # 載入 pid_logs/ 所有 CSV，互動切換
  python plot_pid.py pid_logs/xxx.csv [yyy.csv ...]  # 指定檔案
鍵盤快捷鍵：
  ← / →  上一筆 / 下一筆
  數字鍵  直接跳到第 N 筆（1~9）
"""
import sys
import os
import re
import glob
import csv
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import matplotlib
from matplotlib.widgets import Button

matplotlib.rcParams['font.family'] = ['Microsoft JhengHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False

# ── 收集檔案清單 ──
if len(sys.argv) >= 2:
    files = sys.argv[1:]
else:
    files = sorted(glob.glob('pid_logs/althold_*.csv') + glob.glob('vel_logs/velmode_*.csv'))

if not files:
    print("❌ 找不到 pid_logs/ 或 vel_logs/ 下的 CSV 檔案，請先飛行並開啟定高模式。")
    sys.exit(1)

print(f"📂 共找到 {len(files)} 筆紀錄")

# ── 讀取單一 CSV ──
def load_csv(fpath):
    times, cur_alt, tgt_alt, pid_255 = [], [], [], []
    pid_params = '不詳'
    with open(fpath, newline='', encoding='utf-8') as f:
        content = f.read()
    for line in content.splitlines():
        if line.startswith('#'):
            m = re.search(r'Kp=[\d.?]+\s+Ki=[\d.?]+\s+Kd=[\d.?]+', line)
            if m:
                pid_params = m.group(0)
    import io
    csv_lines = [l for l in content.splitlines() if not l.startswith('#')]
    for row in csv.DictReader(io.StringIO('\n'.join(csv_lines))):
        try:
            times.append(float(row['time_s']))
            cur_alt.append(float(row['current_alt_cm']))
            tgt_alt.append(float(row['target_alt_cm']))
            pid_255.append(float(row['pid_thr_0_255']) if row['pid_thr_0_255'] else None)
        except (ValueError, KeyError):
            pass
    return times, cur_alt, tgt_alt, pid_255, pid_params

# ── 繪圖狀態 ──
idx = [len(files) - 1]  # 預設顯示最新一筆

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 7), sharex=True,
                                gridspec_kw={'height_ratios': [2, 1]})
fig.subplots_adjust(bottom=0.13)

ax_prev = fig.add_axes([0.35, 0.02, 0.1, 0.05])
ax_next = fig.add_axes([0.55, 0.02, 0.1, 0.05])
btn_prev = Button(ax_prev, '◀ 上一筆')
btn_next = Button(ax_next, '下一筆 ▶')

def draw(i):
    fpath = files[i]
    times, cur_alt, tgt_alt, pid_255, pid_params = load_csv(fpath)
    is_velmode = os.path.basename(fpath).startswith('velmode_')

    if not times:
        fig.suptitle(f'❌ 無有效資料 — {os.path.basename(fpath)}', fontsize=13)
        return

    ax1.cla()
    ax2.cla()

    mode_label = '速度模式' if is_velmode else '定高模式'
    fig.suptitle(
        f'{mode_label} PID 分析  [{i+1}/{len(files)}]  —  {os.path.basename(fpath)}\n{pid_params}',
        fontsize=12
    )

    ax1.plot(times, cur_alt, color='#2196F3', linewidth=1.5, label='當前高度')
    if not is_velmode:
        ax1.plot(times, tgt_alt, color='#F44336', linewidth=1.2, linestyle='--', label='目標高度')
        ax1.fill_between(times, cur_alt, tgt_alt,
                         where=[c > t for c, t in zip(cur_alt, tgt_alt)],
                         alpha=0.15, color='#F44336', label='超射')
        ax1.fill_between(times, cur_alt, tgt_alt,
                         where=[c < t for c, t in zip(cur_alt, tgt_alt)],
                         alpha=0.15, color='#2196F3', label='欠射')
        stable_start = max(0, int(len(times) * 0.6))
        steady_vals = [c - t for c, t in zip(cur_alt[stable_start:], tgt_alt[stable_start:])
                       if c is not None]
        steady_err  = round(sum(steady_vals) / len(steady_vals), 2) if steady_vals else 0
        overshoot   = round(max((c - t for c, t in zip(cur_alt, tgt_alt)), default=0), 2)
        undershoot  = round(min((t - c for c, t in zip(cur_alt, tgt_alt)), default=0), 2)
        stats_text = (f'穩態誤差: {steady_err:+.2f} cm    '
                      f'最大超射: {overshoot:.2f} cm    '
                      f'最大欠射: {undershoot:.2f} cm')
        ax1.text(0.01, 0.03, stats_text, transform=ax1.transAxes,
                 fontsize=9, color='#555', verticalalignment='bottom')
    ax1.set_ylabel('高度 (cm)')
    ax1.legend(loc='upper right', fontsize=9)
    ax1.yaxis.set_minor_locator(ticker.AutoMinorLocator())
    ax1.grid(True, which='major', linestyle='-', alpha=0.3)
    ax1.grid(True, which='minor', linestyle=':', alpha=0.2)

    valid_t   = [t for t, p in zip(times, pid_255) if p is not None]
    valid_pid = [p for p in pid_255 if p is not None]
    ax2.plot(valid_t, valid_pid, color='#4CAF50', linewidth=1.5, label='PID 油門 (0~255)')
    ax2.set_ylabel('油門')
    ax2.set_xlabel('時間 (s)')
    ax2.legend(loc='upper right', fontsize=9)
    ax2.yaxis.set_minor_locator(ticker.AutoMinorLocator())
    ax2.grid(True, which='major', linestyle='-', alpha=0.3)
    ax2.grid(True, which='minor', linestyle=':', alpha=0.2)

    # 更新按鈕狀態提示
    btn_prev.label.set_text(f'◀ 上一筆' if i > 0 else '（已是第一筆）')
    btn_next.label.set_text(f'下一筆 ▶' if i < len(files) - 1 else '（已是最後筆）')

    fig.canvas.draw_idle()

def on_prev(_):
    if idx[0] > 0:
        idx[0] -= 1
        draw(idx[0])

def on_next(_):
    if idx[0] < len(files) - 1:
        idx[0] += 1
        draw(idx[0])

def on_key(event):
    if event.key == 'left':
        on_prev(None)
    elif event.key == 'right':
        on_next(None)
    elif event.key.isdigit():
        n = int(event.key)
        if 1 <= n <= len(files):
            idx[0] = n - 1
            draw(idx[0])

btn_prev.on_clicked(on_prev)
btn_next.on_clicked(on_next)
fig.canvas.mpl_connect('key_press_event', on_key)

draw(idx[0])
plt.show()
