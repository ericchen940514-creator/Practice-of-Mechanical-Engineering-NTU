"""
pid_logger.py — 定高 PID 數據記錄模組（測試用）
每次定高開啟時自動建立 CSV，關閉時儲存。
"""
import os
import re
import csv
import time
import threading

_lock      = threading.Lock()
_file      = None
_writer    = None
_start_t   = 0.0
_log_dir   = 'pid_logs'
_prefix    = 'althold'


def _read_pid_from_ino() -> str:
    """從 Arduino sketch 讀取 Kp/Ki/Kd，回傳格式化字串。"""
    ino_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            'sketch_althold_v6_velmode', 'sketch_althold_v6_velmode.ino')
    params = {}
    try:
        with open(ino_path, encoding='utf-8') as f:
            for line in f:
                for p in ('Kp_vel', 'Ki_vel', 'Kd_vel'):
                    m = re.search(rf'const float {p}\s*=\s*([\d.]+)', line)
                    if m:
                        params[p] = m.group(1)
    except Exception:
        pass
    if params:
        return f"Kp={params.get('Kp_vel','?')} Ki={params.get('Ki_vel','?')} Kd={params.get('Kd_vel','?')}"
    return "Kp=? Ki=? Kd=?"


def configure(log_dir: str = 'pid_logs', prefix: str = 'althold') -> None:
    """在 start() 之前呼叫，設定儲存資料夾與檔名前綴。"""
    global _log_dir, _prefix
    _log_dir = log_dir
    _prefix  = prefix


def start(target_alt_cm: float) -> None:
    """定高啟動時呼叫，建立新的 CSV 檔案。"""
    global _file, _writer, _start_t
    os.makedirs(_log_dir, exist_ok=True)
    fname = os.path.join(_log_dir, f"{_prefix}_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    pid_str = _read_pid_from_ino()
    with _lock:
        _file = open(fname, 'w', newline='', encoding='utf-8')
        _file.write(f'# {pid_str}\n')
        _writer  = csv.writer(_file)
        _writer.writerow(['time_s', 'current_alt_cm', 'target_alt_cm', 'vel_cmd_cm_s', 'pid_thr_ibus', 'pid_thr_0_255'])
        _start_t = time.time()
    print(f"\n📝 開始記錄 PID 數據：{fname}（目標高度 {target_alt_cm} cm，{pid_str}）")


def stop() -> None:
    """定高關閉或程式結束時呼叫，存檔。"""
    global _file, _writer
    with _lock:
        if _file:
            _file.close()
            _file   = None
            _writer = None
            print("\n📝 PID 數據記錄已儲存。")


def record(current_alt_cm: float, target_alt_cm: float, pid_throttle_ibus: int,
           vel_cmd_cm_s: float = 0.0) -> None:
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
            round(vel_cmd_cm_s, 2),
            pid_ibus,
            pid_255,
        ])
