#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
光流補償方向測試工具
====================

目的：在「不解鎖、不送控制封包」的前提下，地面手動平移機體，
      快速判斷光流定點補正的「軸向」與「符號」對不對。

原理（為什麼只看 dx/dy 正負就夠）：
  main_esp32_velmode.py 的補正公式是
      flow_p_roll  = -FLOW_ROLL_SIGN  * (norm_dx) * FLOW_P_GAIN
      flow_p_pitch = -FLOW_PITCH_SIGN * (norm_dy) * FLOW_P_GAIN
      （norm = mean_d * alt_cm * 0.01，alt 恆正、gain 恆正）
  → 補正「符號」 = -SIGN * sign(累積 dx/dy)，與高度、增益無關。
  → 想要負回饋（把機體推回原位）：
        往「右」平移時，roll 補正必須是「往左推桿」
        往「前」平移時，pitch 補正必須是「往後推桿」

桿位方向約定（標準 Mode 2 右搖桿，與 README 一致）：
  roll  正 = 往右；負 = 往左
  pitch 正 = 往前；負 = 往後
  （若你的實機某軸桿位是反的，該軸結論跟著反；可直接看即時數字自行判斷）

安全：
  - 本程式只「讀」遙測，不送任何封包，ESP32 不會收到 arm 指令。
  - 仍強烈建議「拆掉螺旋槳」再做，避免任何意外。
  - PMW3901 要對著有紋理的地面，高度抓飛行時的常用高度（例如 30~60cm）。

用法：
  python test_flow_direction.py --port COM4
"""

import argparse
import sys
import time
import threading
from collections import deque

try:
    import serial
except ImportError:
    print("缺少 pyserial：pip install pyserial")
    sys.exit(1)

# ── 與 main_esp32_velmode.py 同步的常數（只用於顯示，不影響方向判斷）──
FLOW_ROLL_SIGN     = +1      # ← 把你 main 程式現在的值貼進來
FLOW_PITCH_SIGN    = -1      # ← 把你 main 程式現在的值貼進來
FLOW_P_GAIN        = 0.10
_FLOW_COUNTS_TO_RAD = 0.01

# ── 參數 ──
parser = argparse.ArgumentParser(description="光流補償方向測試")
parser.add_argument("--port", default="COM9", help="藍牙 COM 埠（預設 COM9）")
parser.add_argument("--baud", type=int, default=9600, help="鮑率（預設 9600，與 main 相同）")
parser.add_argument("--capture", type=float, default=4.0, help="每次擷取秒數（預設 4 秒）")
args = parser.parse_args()

# ── 共享狀態 ──
_lock        = threading.Lock()
_latest_dx   = 0          # 最近一筆 OF 的瞬時值（顯示用）
_latest_dy   = 0
_accum_dx    = 0          # 開機以來的累積總和（擷取靠快照差值）
_accum_dy    = 0
_alt_cm      = 0.0
_of_count    = 0          # 收到幾筆 OF（判斷有沒有資料）
_recent      = deque(maxlen=25)
_running     = True


def _reader(ser):
    """以 in_waiting 輪詢 + 手動換行切割，避免 Windows 藍牙 readline 漏接。"""
    global _latest_dx, _latest_dy, _accum_dx, _accum_dy, _alt_cm, _of_count
    buf = b""
    while _running:
        try:
            n = ser.in_waiting
            if n:
                buf += ser.read(n)
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    line = raw.decode("utf-8", "ignore").strip()
                    if not line:
                        continue
                    if line.startswith("OF:"):
                        payload = line[3:]
                        if payload.startswith("REINIT"):
                            print(f"\n[PMW3901 watchdog] {payload}")
                            continue
                        parts = payload.split(",")
                        if len(parts) == 2:
                            try:
                                dx, dy = int(parts[0]), int(parts[1])
                            except ValueError:
                                continue
                            with _lock:
                                _latest_dx, _latest_dy = dx, dy
                                _accum_dx += dx
                                _accum_dy += dy
                                _of_count += 1
                                _recent.append((dx, dy))
                    elif line.startswith("D:"):
                        try:
                            with _lock:
                                _alt_cm = int(line[2:]) / 10.0
                        except ValueError:
                            pass
            else:
                time.sleep(0.005)
        except (OSError, serial.SerialException):
            print("\n⚠️ 串列讀取錯誤，連線可能中斷。")
            time.sleep(0.2)


def snapshot():
    with _lock:
        return _accum_dx, _accum_dy, _alt_cm, _of_count


def live_monitor():
    """自由監看：隨意移動機體，觀察 dx/dy 與即時補正方向。"""
    print("\n── 即時監看（Ctrl+C 返回選單）──")
    print("把機體往各方向平移，觀察 dx/dy 怎麼變，先建立直覺。\n")
    try:
        while True:
            with _lock:
                dx, dy, alt = _latest_dx, _latest_dy, _alt_cm
                cnt = _of_count
            # 即時補正（用瞬時值，僅示意方向）
            alt_eff = max(alt, 20.0)
            ndx = dx * alt_eff * _FLOW_COUNTS_TO_RAD
            ndy = dy * alt_eff * _FLOW_COUNTS_TO_RAD
            roll_corr  = -FLOW_ROLL_SIGN  * ndx * FLOW_P_GAIN
            pitch_corr = -FLOW_PITCH_SIGN * ndy * FLOW_P_GAIN
            r_dir = "→右" if roll_corr  > 0.001 else ("←左" if roll_corr  < -0.001 else " · ")
            p_dir = "↑前" if pitch_corr > 0.001 else ("↓後" if pitch_corr < -0.001 else " · ")
            data = "OK " if cnt else "無資料"
            sys.stdout.write(
                f"\r[{data}] alt={alt:5.1f}cm  dx={dx:+5d} dy={dy:+5d} | "
                f"補正 roll={roll_corr:+.3f}{r_dir}  pitch={pitch_corr:+.3f}{p_dir}   "
            )
            sys.stdout.flush()
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n")


def capture_slide(label):
    """固定秒數擷取一次平移，回傳期間累積的 (dx, dy)。"""
    input(f"\n準備好後按 Enter，接著用約 {args.capture:.0f} 秒，"
          f"把機體平穩地往「{label}」平移 20~30cm…")
    d0x, d0y, _, c0 = snapshot()
    t_end = time.time() + args.capture
    peak = 0
    while time.time() < t_end:
        with _lock:
            dx, dy = _latest_dx, _latest_dy
        remain = t_end - time.time()
        cur_x, cur_y, alt, _ = snapshot()
        sx, sy = cur_x - d0x, cur_y - d0y
        peak = max(peak, abs(sx), abs(sy))
        sys.stdout.write(
            f"\r  擷取中… {remain:3.1f}s  瞬時 dx={dx:+5d} dy={dy:+5d}  "
            f"累積 Σdx={sx:+6d} Σdy={sy:+6d}   "
        )
        sys.stdout.flush()
        time.sleep(0.04)
    d1x, d1y, alt, c1 = snapshot()
    sum_dx, sum_dy = d1x - d0x, d1y - d0y
    got = c1 - c0
    print()
    return sum_dx, sum_dy, got, peak


def verdict_axis(slide_label, sum_dx, sum_dy, axis, sign_name, sign_val,
                 corr_should_be_negative):
    """
    axis: 'dx'(roll) 或 'dy'(roll/pitch 對應的感測器軸)
    corr_should_be_negative: 這次平移方向下，正確補正應為負（往左 / 往後）
    """
    d_main = sum_dx if axis == "dx" else sum_dy
    d_other = sum_dy if axis == "dx" else sum_dx
    other_axis = "dy" if axis == "dx" else "dx"

    print(f"\n  ── {slide_label} 的判讀 ──")
    print(f"  Σdx={sum_dx:+d}   Σdy={sum_dy:+d}")

    # 1) 軸向檢查：這個方向應該主要由 {axis} 反應
    if abs(d_main) < 30:
        print("  ⚠️ 主軸位移太小（<30 counts），可能沒移動到 / 高度太低 / 地面沒紋理。"
              "請拉高一點、移大力一點重做。")
        return None
    if abs(d_other) > abs(d_main):
        print(f"  ⚠️ 軸向疑似錯置：這個方向 {other_axis} 反應({d_other:+d}) 比 "
              f"{axis} ({d_main:+d}) 還大。")
        print(f"     代表 PMW3901 可能裝歪/轉了 90°，或 dx/dy 該對調。"
              f"先把感測器擺正（X 軸對機頭右、Y 軸對機頭前）再測符號。")
        return None

    # 2) 符號檢查：補正符號 = -SIGN * sign(d_main)
    corr_sign = -sign_val * (1 if d_main > 0 else -1)   # +1=正補正, -1=負補正
    corr_is_negative = corr_sign < 0
    want = "負（往左/往後推桿，抵消平移）" if corr_should_be_negative else "正"
    actual = "負" if corr_is_negative else "正"

    print(f"  {axis} 主反應 = {d_main:+d}（方向明確 ✓）")
    print(f"  目前 {sign_name} = {sign_val:+d}  →  補正符號為「{actual}」，"
          f"應為「{want.split('（')[0]}」")

    if corr_is_negative == corr_should_be_negative:
        print(f"  ✅ 方向正確：{sign_name} = {sign_val:+d} 不用改（負回饋，會把機體推回去）。")
        return sign_val
    else:
        new_val = -sign_val
        print(f"  ❌ 方向相反！現在是正回饋，一定會越飄越開。")
        print(f"     請把 main_esp32_velmode.py 改成：{sign_name} = {new_val:+d}")
        return new_val


def guided_test():
    print("\n══════════ 引導式方向測試 ══════════")
    print("固定機頭朝向（和飛行時一致），地面平移即可。建議拆槳、對著有紋理地面。")

    # 確認有資料
    _, _, alt0, c0 = snapshot()
    time.sleep(0.6)
    _, _, alt1, c1 = snapshot()
    if c1 - c0 == 0:
        print("\n⚠️ 收不到 OF 遙測。檢查：BT 是否連上、PMW3901 是否 init OK、port 對不對。")
        return
    print(f"✓ 有收到光流資料，目前高度約 {alt1:.0f}cm\n")

    # ROLL（左右）：往「右」平移 → 正確補正應為「負（往左）」，主軸 dx
    sx, sy, got, peak = capture_slide("右")
    roll_fix = verdict_axis("往右平移", sx, sy, axis="dx",
                            sign_name="FLOW_ROLL_SIGN", sign_val=FLOW_ROLL_SIGN,
                            corr_should_be_negative=True)

    # PITCH（前後）：往「前」平移 → 正確補正應為「負（往後）」，主軸 dy
    sx, sy, got, peak = capture_slide("前")
    pitch_fix = verdict_axis("往前平移", sx, sy, axis="dy",
                             sign_name="FLOW_PITCH_SIGN", sign_val=FLOW_PITCH_SIGN,
                             corr_should_be_negative=True)

    print("\n══════════ 總結 ══════════")
    if roll_fix is not None:
        print(f"FLOW_ROLL_SIGN  建議 = {roll_fix:+d}"
              + ("（維持）" if roll_fix == FLOW_ROLL_SIGN else "（需修改！）"))
    if pitch_fix is not None:
        print(f"FLOW_PITCH_SIGN 建議 = {pitch_fix:+d}"
              + ("（維持）" if pitch_fix == FLOW_PITCH_SIGN else "（需修改！）"))
    print("改完記得重跑 main，並先在低空、隨時準備切手動的情況下驗證。\n")


def connect():
    print(f"連線 {args.port} @ {args.baud}…")
    for attempt in range(20):
        try:
            s = serial.Serial(args.port, args.baud, timeout=0.05, write_timeout=0.1)
            print(f"✅ 已連線 {args.port}")
            return s
        except serial.SerialException as e:
            print(f"  重試 {attempt + 1}/20… ({e})")
            time.sleep(3)
    print("❌ 連線失敗，確認 ESP32 已配對且 COM 埠正確。")
    sys.exit(1)


def main():
    global _running
    ser = connect()
    th = threading.Thread(target=_reader, args=(ser,), daemon=True)
    th.start()
    time.sleep(1.0)

    try:
        while True:
            print("\n┌──────────── 光流方向測試 ────────────┐")
            print("│ 1) 即時監看（先建立 dx/dy 直覺）      │")
            print("│ 2) 引導式方向測試（自動判讀符號/軸向）│")
            print("│ q) 離開                               │")
            print("└───────────────────────────────────────┘")
            choice = input("選擇： ").strip().lower()
            if choice == "1":
                live_monitor()
            elif choice == "2":
                guided_test()
            elif choice == "q":
                break
            else:
                print("無效選擇。")
    except KeyboardInterrupt:
        pass
    finally:
        _running = False
        time.sleep(0.1)
        try:
            ser.close()
        except Exception:
            pass
        print("已關閉。")


if __name__ == "__main__":
    main()
