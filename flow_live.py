"""
flow_live.py — 光流訊號即時視覺化
與 flow_logger 平行運作：record() 把資料丟進跨行程佇列，
子行程開 matplotlib 視窗即時繪圖，不阻塞主控制迴圈。

四個面板：
  ① dx / dy 時間序列（最近 WINDOW_S 秒）
  ② 光流量級 |flow|（最近 WINDOW_S 秒）
  ③ 高度 alt_cm（最近 WINDOW_S 秒）
  ④ 累積平面路徑（cm，以即時高度換算）
"""
import multiprocessing as mp
import time
import math
from collections import deque

WINDOW_S       = 10.0   # 時間序列視窗（秒）
PATH_MAX_PTS   = 2000   # 路徑最多保留點數
COUNTS_TO_RAD  = 0.01   # 與 plot_flow.py 一致
REDRAW_HZ      = 20

_proc  = None
_queue = None


def _viewer(q: 'mp.Queue') -> None:
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    matplotlib.rcParams['font.family']     = ['Microsoft JhengHei', 'DejaVu Sans']
    matplotlib.rcParams['axes.unicode_minus'] = False

    ts   = deque()
    dxs  = deque()
    dys  = deque()
    mags = deque()
    alts_t = deque()
    alts   = deque()

    px = deque(maxlen=PATH_MAX_PTS)
    py = deque(maxlen=PATH_MAX_PTS)
    px.append(0.0); py.append(0.0)
    cx = cy = 0.0
    t0 = None

    fig, axes = plt.subplots(2, 2, figsize=(11, 7))
    fig.canvas.manager.set_window_title('光流即時訊號')
    ax_dxy, ax_mag = axes[0]
    ax_alt, ax_path = axes[1]

    ln_dx, = ax_dxy.plot([], [], color='#E53935', linewidth=1.2, label='dx')
    ln_dy, = ax_dxy.plot([], [], color='#1E88E5', linewidth=1.2, label='dy')
    ax_dxy.set_title('光流原始增量 dx / dy (counts)')
    ax_dxy.set_xlabel('時間 (s)')
    ax_dxy.axhline(0, color='gray', linewidth=0.5)
    ax_dxy.grid(True, linestyle=':', alpha=0.4)
    ax_dxy.legend(loc='upper left', fontsize=9)

    ln_mag, = ax_mag.plot([], [], color='#FF9800', linewidth=1.2)
    ax_mag.set_title('光流量級 |flow|')
    ax_mag.set_xlabel('時間 (s)')
    ax_mag.set_ylabel('counts')
    ax_mag.grid(True, linestyle=':', alpha=0.4)

    ln_alt, = ax_alt.plot([], [], color='#2196F3', linewidth=1.4)
    ax_alt.set_title('高度')
    ax_alt.set_xlabel('時間 (s)')
    ax_alt.set_ylabel('alt (cm)')
    ax_alt.grid(True, linestyle=':', alpha=0.4)

    ln_path, = ax_path.plot([], [], color='#7B1FA2', linewidth=1.2)
    pt_start = ax_path.plot([0], [0], 'go', markersize=7, label='起點')[0]
    pt_now,  = ax_path.plot([0], [0], 'rs', markersize=7, label='當前')
    ax_path.set_title('累積路徑 (cm)')
    ax_path.set_xlabel('X (cm)')
    ax_path.set_ylabel('Y (cm)')
    ax_path.set_aspect('equal', adjustable='datalim')
    ax_path.grid(True, linestyle=':', alpha=0.4)
    ax_path.legend(loc='upper left', fontsize=9)

    fig.tight_layout()

    def drain():
        nonlocal cx, cy, t0
        got = False
        try:
            while True:
                msg = q.get_nowait()
                if msg is None:
                    plt.close(fig)
                    return False
                kind = msg[0]
                if kind == 'reset':
                    ts.clear(); dxs.clear(); dys.clear(); mags.clear()
                    alts_t.clear(); alts.clear()
                    px.clear(); py.clear(); px.append(0.0); py.append(0.0)
                    cx = cy = 0.0
                    t0 = None
                    got = True
                elif kind == 'of':
                    _, t_abs, dx, dy, alt_mm = msg
                    if t0 is None:
                        t0 = t_abs
                    t_rel = t_abs - t0
                    ts.append(t_rel); dxs.append(dx); dys.append(dy)
                    mags.append(math.hypot(dx, dy))
                    alt_cm = (alt_mm / 10.0) if alt_mm > 0 else None
                    if alt_cm is not None:
                        alts_t.append(t_rel); alts.append(alt_cm)
                    scale = COUNTS_TO_RAD * (alt_cm if alt_cm is not None else 50.0)
                    cx += dx * scale
                    cy += dy * scale
                    px.append(cx); py.append(cy)
                    got = True
        except Exception:
            pass

        if ts:
            cutoff = ts[-1] - WINDOW_S
            while ts and ts[0] < cutoff:
                ts.popleft(); dxs.popleft(); dys.popleft(); mags.popleft()
            while alts_t and alts_t[0] < cutoff:
                alts_t.popleft(); alts.popleft()
        return got

    def redraw():
        ln_dx.set_data(ts, dxs)
        ln_dy.set_data(ts, dys)
        ln_mag.set_data(ts, mags)
        ln_alt.set_data(alts_t, alts)
        ln_path.set_data(px, py)
        if px:
            pt_now.set_data([px[-1]], [py[-1]])

        if ts:
            x_hi = ts[-1]
            x_lo = max(0.0, x_hi - WINDOW_S)
            for ax in (ax_dxy, ax_mag, ax_alt):
                ax.set_xlim(x_lo, x_hi if x_hi > x_lo else x_lo + 1e-3)
        for ax, ys in ((ax_dxy, list(dxs) + list(dys)),
                       (ax_mag, list(mags)),
                       (ax_alt, list(alts))):
            if ys:
                lo, hi = min(ys), max(ys)
                if lo == hi:
                    lo -= 1; hi += 1
                pad = (hi - lo) * 0.1
                ax.set_ylim(lo - pad, hi + pad)
        if px:
            ax_path.relim(); ax_path.autoscale_view()

    period = 1.0 / REDRAW_HZ
    plt.show(block=False)
    try:
        while plt.fignum_exists(fig.number):
            alive = drain()
            if alive is False:
                break
            redraw()
            try:
                fig.canvas.draw_idle()
                fig.canvas.flush_events()
            except Exception:
                break
            time.sleep(period)
    finally:
        try:
            plt.close('all')
        except Exception:
            pass


def start() -> None:
    """開啟即時繪圖視窗（若已開啟則重置路徑）。"""
    global _proc, _queue
    if _proc is not None and _proc.is_alive():
        try:
            _queue.put_nowait(('reset',))
        except Exception:
            pass
        return
    _queue = mp.Queue(maxsize=4096)
    _proc  = mp.Process(target=_viewer, args=(_queue,), daemon=True)
    _proc.start()
    print("\n📈 光流即時視窗已開啟。")


def stop() -> None:
    global _proc, _queue
    if _proc is None:
        return
    try:
        _queue.put_nowait(None)
    except Exception:
        pass
    try:
        _proc.join(timeout=1.0)
    except Exception:
        pass
    _proc  = None
    _queue = None


def record(dx: int, dy: int, alt_mm: float) -> None:
    if _queue is None:
        return
    try:
        _queue.put_nowait(('of', time.time(), int(dx), int(dy), float(alt_mm)))
    except Exception:
        pass  # 滿了就丟掉，不影響控制迴圈
