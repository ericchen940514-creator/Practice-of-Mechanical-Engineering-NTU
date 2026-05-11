"""
flow_logger.py — 光流路徑數據記錄模組
解鎖時自動建立 CSV，上鎖或程式結束時儲存。
CSV 欄位：time_s, dx, dy, alt_mm
"""
import os
import csv
import time
import threading

_lock     = threading.Lock()
_file     = None
_writer   = None
_start_t  = 0.0
_log_dir  = 'flow_logs'
_prefix   = 'flow'


def configure(log_dir: str = 'flow_logs', prefix: str = 'flow') -> None:
    global _log_dir, _prefix
    _log_dir = log_dir
    _prefix  = prefix


def start() -> None:
    global _file, _writer, _start_t
    os.makedirs(_log_dir, exist_ok=True)
    fname = os.path.join(_log_dir, f"{_prefix}_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    with _lock:
        _file    = open(fname, 'w', newline='', encoding='utf-8')
        _writer  = csv.writer(_file)
        _writer.writerow(['time_s', 'dx', 'dy', 'alt_mm'])
        _start_t = time.time()
    print(f"\n📍 開始記錄光流路徑：{fname}")


def stop() -> None:
    global _file, _writer
    with _lock:
        if _file:
            _file.close()
            _file   = None
            _writer = None
            print("\n📍 光流路徑記錄已儲存。")


def record(dx: int, dy: int, alt_mm: float) -> None:
    with _lock:
        if _writer is None:
            return
        _writer.writerow([
            round(time.time() - _start_t, 3),
            dx,
            dy,
            int(alt_mm) if alt_mm >= 0 else '',
        ])
