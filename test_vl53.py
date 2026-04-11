import serial
import time

# ==========================================
# VL53L0X 雷射測距感測器測試
# 需先燒錄 sketch_vl53_test.ino 到 Arduino
# ==========================================
COM_PORT  = 'COM4'
BAUD_RATE = 9600

print(f"嘗試連線到 {COM_PORT}...")
try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=2)
    print("✅ 連線成功！開始接收距離數值（Ctrl+C 結束）\n")
except Exception as e:
    print(f"❌ 連線失敗: {e}")
    exit()

time.sleep(2)  # 等 Arduino 重置
ser.reset_input_buffer()

try:
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            continue

        if line == "TIMEOUT":
            print("⚠️  感測器逾時（超出範圍或接線問題）")
        elif line.startswith("D:"):
            try:
                mm = int(line[2:])
                cm = mm / 10.0
                bar = '█' * min(int(cm / 2), 50)  # 每 2cm 一格，最多 50 格
                status = "✅" if 30 <= mm <= 1200 else "⚠️ 超出有效範圍"
                print(f"{status}  {mm:5d} mm  ({cm:5.1f} cm)  |{bar}", end="\r")
            except ValueError:
                pass
        else:
            print(f"[未知訊息] {line}")

except KeyboardInterrupt:
    print("\n\n🛑 測試結束。")
finally:
    ser.close()
    print("✅ 已關閉連線。")
