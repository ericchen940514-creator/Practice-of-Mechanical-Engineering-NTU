import serial
import time
import threading

# ==========================================
# 藍牙收訊確認工具（不需要手把）
# 送封包給 Arduino，同時從 USB 線確認 Arduino 有沒有收到
# ==========================================
BT_PORT  = 'COM4'   # ← 藍牙傳出 COM 埠（裝置管理員查）
USB_PORT = 'COM3'   # ← Arduino USB COM 埠

print(f"連線藍牙 {BT_PORT}...")
try:
    bt = serial.Serial(BT_PORT, 9600, timeout=1)
    print("✅ 藍牙連線成功")
except Exception as e:
    print(f"❌ 藍牙連線失敗: {e}"); exit()

print(f"連線 Arduino USB {USB_PORT}...")
try:
    usb = serial.Serial(USB_PORT, 115200, timeout=0.1)
    usb.reset_input_buffer()
    print("✅ Arduino USB 連線成功\n")
except Exception as e:
    print(f"❌ Arduino USB 連線失敗: {e}"); exit()

# 送固定封包：油門=128（對應 IBUS 約 1502），其他置中，未解鎖
PACKET = b'S' + bytes([128, 127, 127, 127, 127, 0])

print("開始測試，按 Ctrl+C 停止\n")

received = False

def read_ibus():
    """背景讀 IBUS，確認油門通道有沒有變成 ~1502"""
    global received
    while True:
        if usb.in_waiting > 0:
            if usb.read(1) == b'\x20':
                if usb.read(1) == b'\x40':
                    packet = usb.read(30)
                    if len(packet) == 30:
                        throttle = packet[4] | (packet[5] << 8)
                        # 預設值是 1000，收到我們送的 128 後應變成約 1502
                        if throttle > 1100:
                            if not received:
                                print(f"\n✅ Arduino 有收到藍牙訊號！(IBUS 油門通道: {throttle})")
                                received = True
                        else:
                            received = False
                        usb.reset_input_buffer()
        time.sleep(0.005)

t = threading.Thread(target=read_ibus, daemon=True)
t.start()

count = 0
try:
    while True:
        bt.write(PACKET)
        count += 1
        status = "✅ 已確認收到" if received else "⏳ 等待確認..."
        print(f"已送出第 {count} 個封包 | {status}", end="\r")
        time.sleep(1)
except KeyboardInterrupt:
    print(f"\n結束，共送出 {count} 個封包。")
finally:
    bt.close()
    usb.close()
