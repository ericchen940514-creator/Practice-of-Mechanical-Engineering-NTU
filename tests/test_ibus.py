import serial
import time

USB_PORT = 'COM3' 
BAUD_RATE = 115200 

print(f"準備攔截 Arduino 的 IBUS 封包 ({USB_PORT})...")

try:
    ser = serial.Serial(USB_PORT, BAUD_RATE, timeout=0.1)
    
    # 啟動時先清空 USB 裡面的開機垃圾亂碼
    ser.reset_input_buffer() 
    
    while True:
        # 一個 Byte 一個 Byte 找，絕對不會錯位
        if ser.in_waiting > 0:
            # 尋找 IBUS 的第一碼：長度 0x20 (32)
            if ser.read(1) == b'\x20':
                # 找到第一碼後，檢查第二碼：指令 0x40 (64)
                if ser.read(1) == b'\x40':
                    # 確認是 IBUS 開頭！直接把剩下的 30 個 Bytes 抓下來
                    packet = ser.read(30)
                    
                    if len(packet) == 30:
                        # 注意：因為前兩碼已經被我們讀掉了，所以陣列從 0 開始算是對應原本的第 3 個 Byte
                        ch1_roll     = packet[0] | (packet[1] << 8)
                        ch2_pitch    = packet[2] | (packet[3] << 8)
                        ch3_throttle = packet[4] | (packet[5] << 8)
                        ch4_yaw      = packet[6] | (packet[7] << 8)
                        ch5_gripper  = packet[8] | (packet[9] << 8)
                        
                        # 漂亮地印出來 (\r 會讓同一行覆蓋更新，不會洗版)
                        print(f"📦 成功攔截 IBUS! | 翻滾:{ch1_roll:4d} | 俯仰:{ch2_pitch:4d} | 油門:{ch3_throttle:4d} | 偏航:{ch4_yaw:4d} | 夾爪(AUX1):{ch5_gripper:4d}", end='\r')
                        
                        # 讀完一包後清空緩衝，準備接下一包
                        ser.reset_input_buffer()
                        
        # 縮短等待時間，配合 115200 的極速
        time.sleep(0.002)

except Exception as e:
    print(f"錯誤: {e}")
finally:
    if 'ser' in locals():
        ser.close()