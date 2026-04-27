"""
ESP32 藍牙收訊診斷工具
執行流程：
  Step 1 - USB 診斷：重置 ESP32，確認它有沒有在跑 sketch
  Step 2 - BT 測試：送封包，等 ESP32 藍牙回傳遙測
  Step 3 - 印出診斷結論

封包格式：S + 8 bytes payload + 1 XOR checksum（共 10 bytes）
ESP32 BT 名稱：ESP32_Drone_Hub
"""

import serial
import threading
import time
import sys

# ==================== 設定 ====================
BT_PORT  = 'COM13'   # ← ESP32 藍牙 COM 埠（裝置管理員查）
USB_PORT = 'COM10'   # ← ESP32 USB-Serial COM 埠（可設 None 跳過）

BT_BAUD  = 9600
USB_BAUD = 115200

USB_LISTEN_SECS = 5    # Step 1 監聽 USB 多少秒
SEND_INTERVAL   = 0.3  # Step 2 送封包間隔（秒）
# ===============================================


def make_packet(thr=128, yaw=127, pitch=127, roll=127,
                alt=128, grip=127, arm=0, ah=0):
    payload = bytes([thr, yaw, pitch, roll, alt, grip, arm, ah])
    chk = 0
    for b in payload:
        chk ^= b
    return b'S' + payload + bytes([chk])


def drain_lines(port, secs, label):
    """讀 port 最多 secs 秒，回傳所有收到的行。"""
    lines = []
    buf = b''
    deadline = time.time() + secs
    while time.time() < deadline:
        try:
            if port.in_waiting:
                buf += port.read(port.in_waiting)
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    text = line.decode(errors='ignore').strip()
                    if text:
                        lines.append(text)
                        print(f"  [{label}] {text}")
        except Exception:
            pass
        time.sleep(0.02)
    return lines


# ============================================================
# Step 1：USB 診斷
# ============================================================
print("=" * 60)
print("STEP 1／USB 診斷")
print("=" * 60)

usb = None
if USB_PORT:
    print(f"連線 ESP32 USB {USB_PORT} @ {USB_BAUD} baud ...")
    try:
        usb = serial.Serial(USB_PORT, USB_BAUD, timeout=0.05)
        print("  開啟成功")

        # 用 DTR 重置 ESP32，觸發重新印出 startup 訊息
        print("  發送 DTR reset，等 ESP32 重啟 ...")
        usb.setDTR(False)
        time.sleep(0.2)
        usb.setDTR(True)
        time.sleep(1.5)          # 等 ESP32 開機完成
        usb.reset_input_buffer()

        print(f"  監聽 USB {USB_LISTEN_SECS} 秒（應看到 VL53L0X / Bluetooth Started 等訊息）：")
        usb_lines = drain_lines(usb, USB_LISTEN_SECS, "USB")

        if not usb_lines:
            print("  !! 沒收到任何 USB 訊息 -> ESP32 可能沒在跑這個 sketch，或 COM 埠錯誤")
        else:
            bt_started = any("Bluetooth" in l for l in usb_lines)
            vl_ok      = any("VL53L0X Init OK" in l for l in usb_lines)
            vl_fail    = any("VL53L0X Init Failed" in l for l in usb_lines)
            print()
            print(f"  藍牙初始化：{'OK' if bt_started else '未看到'}")
            print(f"  VL53L0X   ：{'OK' if vl_ok else ('Init Failed' if vl_fail else '未看到')}")

    except Exception as e:
        print(f"  USB 連線失敗: {e}")
        usb = None
else:
    print("  USB_PORT = None，跳過 USB 診斷")

print()

# ============================================================
# Step 2：BT 測試
# ============================================================
print("=" * 60)
print("STEP 2／藍牙封包測試")
print("=" * 60)

print(f"連線藍牙 {BT_PORT} @ {BT_BAUD} baud ...")
try:
    bt = serial.Serial(BT_PORT, BT_BAUD, timeout=0.05)
    print("  開啟成功（注意：port 開啟 ≠ BT 已配對，需要等回傳確認）")
except Exception as e:
    print(f"  藍牙連線失敗: {e}")
    if usb:
        usb.close()
    sys.exit(1)

# 兩種封包交替送：
#   PKT_NORM  - 正常中立封包（ah=0）
#   PKT_AH    - 嘗試啟動 alt hold（ah=1）
#     → ESP32 會從 USB 印 "Alt Hold request rejected" 或 "Alt Hold ON"
#     → 這是沒有 VL53L0X 時確認 BT 有收到的最直接方法
PKT_NORM = make_packet(ah=0)
PKT_AH   = make_packet(ah=1)
print(f"  正常封包（hex）: {PKT_NORM.hex(' ').upper()}")
print(f"  AH封包 （hex）: {PKT_AH.hex(' ').upper()}")
print("  策略：每 5 包送一次 ah=1，看 USB 有沒有印出 'Alt Hold' 字樣")
print()

# 背景讀 BT 回傳
bt_confirmed = False
bt_raw_any   = False
bt_lock      = threading.Lock()
bt_lines_buf = []
usb_lines_buf = []


def read_bt():
    global bt_confirmed, bt_raw_any
    buf = b''
    while True:
        try:
            n = bt.in_waiting
            if n:
                chunk = bt.read(n)
                with bt_lock:
                    bt_raw_any = True
                buf += chunk
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    text = line.decode(errors='ignore').strip()
                    if text:
                        with bt_lock:
                            bt_lines_buf.append(text)
                            if text.startswith(('D:', 'T:', 'P:', 'F:')):
                                bt_confirmed = True
        except Exception:
            pass
        time.sleep(0.02)


def read_usb_bg():
    if not usb:
        return
    buf = b''
    while True:
        try:
            if usb.in_waiting:
                buf += usb.read(usb.in_waiting)
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    text = line.decode(errors='ignore').strip()
                    if text:
                        with bt_lock:
                            usb_lines_buf.append(text)
        except Exception:
            pass
        time.sleep(0.02)


threading.Thread(target=read_bt,     daemon=True).start()
threading.Thread(target=read_usb_bg, daemon=True).start()

print("送封包中，按 Ctrl+C 停止\n")
print(f"  {'#':>5}  {'BT送出':>6}  狀態")
print("  " + "-" * 50)

sent = 0
try:
    while True:
        # 每 5 包送一次 ah=1，其餘送正常封包
        pkt = PKT_AH if (sent % 5 == 4) else PKT_NORM
        bt.write(pkt)
        sent += 1

        with bt_lock:
            conf    = bt_confirmed
            raw_any = bt_raw_any
            new_bt  = bt_lines_buf[:]
            new_usb = usb_lines_buf[:]
            bt_lines_buf.clear()
            usb_lines_buf.clear()

        for l in new_bt:
            print(f"\n  [BT <<] {l}")
        for l in new_usb:
            print(f"\n  [USB<<] {l}")

        if conf:
            status = "ESP32 有回傳遙測"
        elif raw_any:
            status = "有收到原始 bytes（解析中...）"
        else:
            status = "等待回傳..."

        print(f"  #{sent:>5}  {status:<30}", end="\r", flush=True)
        time.sleep(SEND_INTERVAL)

except KeyboardInterrupt:
    pass

print(f"\n\n結束，共送出 {sent} 個封包。")

# ============================================================
# Step 3：結論
# ============================================================
print()
print("=" * 60)
print("STEP 3／診斷結論")
print("=" * 60)

with bt_lock:
    conf    = bt_confirmed
    raw_any = bt_raw_any

if conf:
    print("  ESP32 有收到藍牙封包，並回傳遙測資料。")
elif raw_any:
    print("  BT 有收到原始 bytes，但不是標準遙測格式。")
    print("  可能原因：BT COM 埠是「傳入」而非「傳出」，或 sketch 版本不同。")
else:
    print("  BT 完全無回傳。可能原因：")
    print("    1. Windows 藍牙配對了但 BT COM 埠未真正連線 -> 重新配對")
    print("    2. BT_PORT 填錯（試另一個 COM 埠）")
    print("    3. ESP32 沒有在跑此 sketch（看 Step 1 USB 訊息）")
    print("    4. VL53L0X 未接，D: 遙測不會送出（這不影響 ESP32 收封包，")
    print("       但你可以在 USB 看 Alt Hold ON/OFF 之類的 Serial.println）")
    if usb and not any("Bluetooth" in l for l in (usb_lines_buf or [])):
        print()
        print("  提示：Step 1 USB 若無輸出，先用 Arduino IDE Serial Monitor")
        print("        連 COM10@115200 確認 ESP32 sketch 是否正常執行。")

bt.close()
if usb:
    usb.close()
