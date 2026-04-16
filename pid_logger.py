"""
pid_logger.py — 定高 PID 數據記錄模組（測試用）
每次定高開啟時自動建立 CSV，關閉時儲存。
"""
import os
import csv
import time
import threading

_lock      = threading.Lock()
_file      = None
_writer    = None
_start_t   = 0.0


def start(target_alt_cm: float) -> None:
    """定高啟動時呼叫，建立新的 CSV 檔案。"""
    global _file, _writer, _start_t
    os.makedirs('pid_logs', exist_ok=True)
    fname = os.path.join('pid_logs', f"althold_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    with _lock:
        _file    = open(fname, 'w', newline='', encoding='utf-8')
        _writer  = csv.writer(_file)
        _writer.writerow(['time_s', 'current_alt_cm', 'target_alt_cm', 'pid_thr_ibus', 'pid_thr_0_255'])
        _start_t = time.time()
    print(f"\n📝 開始記錄 PID 數據：{fname}（目標高度 {target_alt_cm} cm）")


def stop() -> None:
    """定高關閉或程式結束時呼叫，存檔。"""
    global _file, _writer
    with _lock:
        if _file:
            _file.close()
            _file   = None
            _writer = None
            print("\n📝 PID 數據記錄已儲存。")


def record(current_alt_cm: float, target_alt_cm: float, pid_throttle_ibus: int) -> None:
    """每次收到高度回報時呼叫（僅定高中有效）。"""
    with _lock:
        if _writer is None:
            return
        pid_ibus = pid_throttle_ibus if pid_throttle_ibus >= 0 else ''
        pid_255  = int((pid_throttle_ibus - 1000) / 1000.0 * 255) if pid_throttle_ibus >= 0 else ''
        _writer.writerow([
            round(time.time() - _start_t, 3),
            current_alt_cm,
            target_alt_cm,
            pid_ibus,
            pid_255,
        ])
