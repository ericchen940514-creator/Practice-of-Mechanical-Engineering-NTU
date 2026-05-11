"""
plot_flow.py — 光流路徑 + 高度視覺化（互動式）
用法：
  python plot_flow.py               # 載入 flow_logs/ 所有 CSV，互動切換
  python plot_flow.py flow_logs/xxx.csv [yyy.csv ...]  # 指定檔案
鍵盤快捷鍵：
  ← / →  上一筆 / 下一筆
  數字鍵  直接跳到第 N 筆（1~9）

座標說明：
  x / y 為光流累積位移（cm），使用當下高度換算
  1 count ≈ 0.01 rad；real_cm = count × 0.01 × alt_cm
  iNAV 的 opflow_scale 會影響實際比例，可在此腳本調 COUNTS_TO_RAD
"""
import sys
import os
import glob
import csv
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import matplotlib
from matplotlib.widgets import Button
import math

matplotlib.rcParams['font.family'] = ['Microsoft JhengHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False

COUNTS_TO_RAD = 0.01  # 與 ino 一致，可調整

# ── 收集檔案清單 ──
if len(sys.argv) >= 2:
    files = sys.argv[1:]
else:
    files = sorted(glob.glob('flow_logs/flow_*.csv'))

if not files:
    print("❌ 找不到 flow_logs/ 下的 CSV 檔案，請先飛行並解鎖。")
    sys.exit(1)

print(f"📂 共找到 {len(files)} 筆紀錄")

# ── 讀取單一 CSV ──
def load_csv(fpath):
    times, dx_list, dy_list, alt_list = [], [], [], []
    with open(fpath, newline='', encoding='utf-8') as f:
        for row in csv.DictReader(f):
            try:
                times.append(float(row['time_s']))
                dx_list.append(int(row['dx']))
                dy_list.append(int(row['dy']))
                alt_mm = float(row['alt_mm']) if row.get('alt_mm') else -1
                alt_list.append(alt_mm)
            except (ValueError, KeyError):
                pass
    return times, dx_list, dy_list, alt_list

def build_path(dx_list, dy_list, alt_list):
    """累積光流位移並換算為 cm，使用每點當下的高度。"""
    x_cm, y_cm = [0.0], [0.0]
    cx = cy = 0.0
    for dx, dy, alt_mm in zip(dx_list, dy_list, alt_list):
        alt_cm = (alt_mm / 10.0) if alt_mm > 0 else 50.0  # 無高度資料時用 50cm 估算
        scale = COUNTS_TO_RAD * alt_cm
        cx += dx * scale
        cy += dy * scale
        x_cm.append(cx)
        y_cm.append(cy)
    return x_cm, y_cm

# ── 繪圖狀態 ──
idx = [len(files) - 1]

fig = plt.figure(figsize=(13, 8))
fig.subplots_adjust(bottom=0.13, left=0.07, right=0.97, top=0.91, wspace=0.35)

ax_path = fig.add_subplot(1, 2, 1)
ax_alt  = fig.add_subplot(2, 2, 2)
ax_flow = fig.add_subplot(2, 2, 4)

ax_prev = fig.add_axes([0.35, 0.02, 0.1, 0.05])
ax_next = fig.add_axes([0.55, 0.02, 0.1, 0.05])
btn_prev = Button(ax_prev, '◀ 上一筆')
btn_next = Button(ax_next, '下一筆 ▶')

def draw(i):
    fpath = files[i]
    times, dx_list, dy_list, alt_list = load_csv(fpath)

    ax_path.cla()
    ax_alt.cla()
    ax_flow.cla()

    fig.suptitle(
        f'光流路徑分析  [{i+1}/{len(files)}]  —  {os.path.basename(fpath)}',
        fontsize=12
    )

    if not times:
        ax_path.text(0.5, 0.5, '無有效資料', transform=ax_path.transAxes,
                     ha='center', va='center')
        fig.canvas.draw_idle()
        return

    x_cm, y_cm = build_path(dx_list, dy_list, alt_list)

    # 顏色漸層：按時間上色
    n = len(x_cm)
    colors = [plt.cm.plasma(k / max(n - 1, 1)) for k in range(n)]

    # ── 左：2D 路徑 ──
    for k in range(n - 1):
        ax_path.plot(x_cm[k:k+2], y_cm[k:k+2], color=colors[k], linewidth=1.8)

    ax_path.plot(x_cm[0],  y_cm[0],  'go', markersize=8, label='起點', zorder=5)
    ax_path.plot(x_cm[-1], y_cm[-1], 'rs', markersize=8, label='終點', zorder=5)

    total_dist = sum(
        math.hypot(x_cm[k+1]-x_cm[k], y_cm[k+1]-y_cm[k]) for k in range(n-1)
    )
    ax_path.set_xlabel('X 位移 (cm)')
    ax_path.set_ylabel('Y 位移 (cm)')
    ax_path.set_title(f'平面路徑（總位移 {total_dist:.1f} cm）')
    ax_path.set_aspect('equal', adjustable='datalim')
    ax_path.legend(fontsize=9)
    ax_path.grid(True, linestyle=':', alpha=0.4)

    sm = plt.cm.ScalarMappable(cmap='plasma',
                                norm=plt.Normalize(vmin=times[0], vmax=times[-1]))
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax_path, fraction=0.046, pad=0.04)
    cbar.set_label('時間 (s)', fontsize=8)

    # ── 右上：高度 vs 時間 ──
    valid_t   = [t for t, a in zip(times, alt_list) if a > 0]
    valid_alt = [a / 10.0 for a in alt_list if a > 0]
    if valid_t:
        ax_alt.plot(valid_t, valid_alt, color='#2196F3', linewidth=1.5)
        ax_alt.set_ylabel('高度 (cm)')
        ax_alt.set_title('高度')
        ax_alt.grid(True, linestyle=':', alpha=0.4)
        ax_alt.yaxis.set_minor_locator(ticker.AutoMinorLocator())
    else:
        ax_alt.text(0.5, 0.5, '無高度資料', transform=ax_alt.transAxes,
                    ha='center', va='center', color='gray')

    # ── 右下：光流量級 vs 時間 ──
    flow_mag = [math.hypot(dx, dy) for dx, dy in zip(dx_list, dy_list)]
    ax_flow.plot(times, flow_mag, color='#FF9800', linewidth=1.2)
    ax_flow.set_xlabel('時間 (s)')
    ax_flow.set_ylabel('光流量級 (counts)')
    ax_flow.set_title('光流強度')
    ax_flow.grid(True, linestyle=':', alpha=0.4)

    btn_prev.label.set_text('◀ 上一筆' if i > 0 else '（已是第一筆）')
    btn_next.label.set_text('下一筆 ▶' if i < len(files) - 1 else '（已是最後筆）')

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
