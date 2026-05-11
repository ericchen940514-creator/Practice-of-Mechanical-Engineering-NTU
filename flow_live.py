"""
flow_live.py — 光流訊號即時視覺化

兩種用途：
  1. 由 main_esp32_velmode.py 以 subprocess 啟動，透過 stdin 接收資料行
  2. 直接執行 `python flow_live.py` 自行測試（從 stdin 餵資料）

資料行格式（每行一筆，以換行結束）：
    of <t_abs> <dx> <dy> <alt_mm>
    reset
"""
import sys
import os
import time
import math
from collections import deque

WINDOW_S      = 10.0
PATH_MAX_PTS  = 2000
COUNTS_TO_RAD = 0.01
REDRAW_HZ     = 20


# ──────────────────────────────────────────────────────────────
# 子行程：當 __main__ 執行時開視窗
# ──────────────────────────────────────────────────────────────
def _run_viewer() -> None:
    import threading
    import matplotlib
    try:
        matplotlib.use('TkAgg')
    except Exception:
        pass
    import matplotlib.pyplot as plt
    matplotlib.rcParams['font.family']        = ['Microsoft JhengHei', 'DejaVu Sans']
    matplotlib.rcParams['axes.unicode_minus'] = False
    print(f"[flow_live] 子行程啟動，backend = {matplotlib.get_backend()}",
          flush=True)

    ts   = deque()
    dxs  = deque()
    dys  = deque()
    mags = deque()
    alts_t = deque()
    alts   = deque()
    px = deque(maxlen=PATH_MAX_PTS)
    py = deque(maxlen=PATH_MAX_PTS)
    px.append(0.0); py.append(0.0)

    state = {'cx': 0.0, 'cy': 0.0, 't0': None, 'closed': False}

    fig, axes = plt.subplots(2, 2, figsize=(11, 7))
    try:
        fig.canvas.manager.set_window_title('光流即時訊號')
    except Exception:
        pass
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

    ln_path, = ax_path.plot([0], [0], color='#7B1FA2', linewidth=1.2)
    ax_path.plot([0], [0], 'go', markersize=7, label='起點')
    pt_now,  = ax_path.plot([0], [0], 'rs', markersize=7, label='當前')
    ax_path.set_title('累積路徑 (cm)')
    ax_path.set_xlabel('X (cm)')
    ax_path.set_ylabel('Y (cm)')
    ax_path.set_aspect('equal', adjustable='datalim')
    ax_path.grid(True, linestyle=':', alpha=0.4)
    ax_path.legend(loc='upper left', fontsize=9)

    fig.tight_layout()

    lock = threading.Lock()

    def reader_thread():
        for raw in sys.stdin:
            line = raw.strip()
            if not line:
                continue
            parts = line.split()
            with lock:
                if parts[0] == 'reset':
                    ts.clear(); dxs.clear(); dys.clear(); mags.clear()
                    alts_t.clear(); alts.clear()
                    px.clear(); py.clear(); px.append(0.0); py.append(0.0)
                    state['cx'] = state['cy'] = 0.0
                    state['t0'] = None
                elif parts[0] == 'of' and len(parts) == 5:
                    try:
                        t_abs  = float(parts[1])
                        dx     = int(parts[2])
                        dy     = int(parts[3])
                        alt_mm = float(parts[4])
                    except ValueError:
                        continue
                    if state['t0'] is None:
                        state['t0'] = t_abs
                    t_rel = t_abs - state['t0']
                    ts.append(t_rel); dxs.append(dx); dys.append(dy)
                    mags.append(math.hypot(dx, dy))
                    alt_cm = (alt_mm / 10.0) if alt_mm > 0 else None
                    if alt_cm is not None:
                        alts_t.append(t_rel); alts.append(alt_cm)
                    scale = COUNTS_TO_RAD * (alt_cm if alt_cm is not None else 50.0)
                    state['cx'] += dx * scale
                    state['cy'] += dy * scale
                    px.append(state['cx']); py.append(state['cy'])
        state['closed'] = True

    threading.Thread(target=reader_thread, daemon=True).start()

    def redraw():
        with lock:
            if ts:
                cutoff = ts[-1] - WINDOW_S
                while ts and ts[0] < cutoff:
                    ts.popleft(); dxs.popleft(); dys.popleft(); mags.popleft()
                while alts_t and alts_t[0] < cutoff:
                    alts_t.popleft(); alts.popleft()
            ln_dx.set_data(list(ts), list(dxs))
            ln_dy.set_data(list(ts), list(dys))
            ln_mag.set_data(list(ts), list(mags))
            ln_alt.set_data(list(alts_t), list(alts))
            ln_path.set_data(list(px), list(py))
            if px:
                pt_now.set_data([px[-1]], [py[-1]])
            has_ts = bool(ts)
            xs_dxy = list(dxs) + list(dys)
            ys_mag = list(mags)
            ys_alt = list(alts)
            x_hi   = ts[-1] if has_ts else 0.0
        if has_ts:
            x_lo = max(0.0, x_hi - WINDOW_S)
            for ax in (ax_dxy, ax_mag, ax_alt):
                ax.set_xlim(x_lo, x_hi if x_hi > x_lo else x_lo + 1e-3)
        for ax, ys in ((ax_dxy, xs_dxy), (ax_mag, ys_mag), (ax_alt, ys_alt)):
            if ys:
                lo, hi = min(ys), max(ys)
                if lo == hi:
                    lo -= 1; hi += 1
                pad = (hi - lo) * 0.1
                ax.set_ylim(lo - pad, hi + pad)
        ax_path.relim(); ax_path.autoscale_view()

    plt.show(block=False)
    print("[flow_live] 視窗已顯示，等待資料中...", flush=True)
    period = 1.0 / REDRAW_HZ
    try:
        while plt.fignum_exists(fig.number):
            redraw()
            try:
                fig.canvas.draw_idle()
                fig.canvas.flush_events()
            except Exception:
                break
            if state['closed']:
                # stdin 已關閉但保留視窗，使用者可手動關
                pass
            time.sleep(period)
    finally:
        try:
            plt.close('all')
        except Exception:
            pass
        print("[flow_live] 視窗已關閉。", flush=True)


# ──────────────────────────────────────────────────────────────
# 主程式呼叫端 API（在 main_esp32_velmode.py 內 import 後使用）
# ──────────────────────────────────────────────────────────────
import subprocess
import threading

_proc      = None
_stdin     = None
_proc_lock = threading.Lock()


def _alive() -> bool:
    return _proc is not None and _proc.poll() is None


def start() -> None:
    """啟動子行程開即時視窗。已開啟則只發 reset。"""
    global _proc, _stdin
    with _proc_lock:
        if _alive():
            try:
                _stdin.write('reset\n'); _stdin.flush()
            except Exception:
                pass
            return
        script = os.path.abspath(__file__)
        try:
            _proc = subprocess.Popen(
                [sys.executable, '-u', script],
                stdin=subprocess.PIPE,
                text=True,
                bufsize=1,
            )
            _stdin = _proc.stdin
            print(f"\n📈 光流即時視窗啟動中（PID {_proc.pid}）...")
        except Exception as e:
            print(f"\n❌ flow_live 啟動失敗：{e}")
            _proc = None
            _stdin = None


def stop() -> None:
    global _proc, _stdin
    with _proc_lock:
        if not _alive():
            _proc = None
            _stdin = None
            return
        try:
            _stdin.close()
        except Exception:
            pass
        try:
            _proc.wait(timeout=1.0)
        except Exception:
            try:
                _proc.terminate()
            except Exception:
                pass
        _proc  = None
        _stdin = None


def record(dx: int, dy: int, alt_mm: float) -> None:
    if not _alive() or _stdin is None:
        return
    try:
        _stdin.write(f"of {time.time():.3f} {int(dx)} {int(dy)} {float(alt_mm):.1f}\n")
        _stdin.flush()
    except Exception:
        pass


if __name__ == '__main__':
    _run_viewer()
