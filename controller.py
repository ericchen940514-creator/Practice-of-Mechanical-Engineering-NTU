import pygame
import serial
import time

# ==========================================
# 🎛️ 試飛調音台 (專屬手感參數設定區)
# ==========================================
COM_PORT = 'COM4'      
BAUD_RATE = 9600

JOYSTICK_SENSITIVITY = 60   
TILT_SENS = 0.6  
YAW_SENS  = 0.5  
STEP_SPEED = 5
# ==========================================

# --- 按鍵編號定義 ---
BTN_SQUARE = 2   
BTN_TRI    = 3   
BTN_L1     = 9   
BTN_R1     = 10   
BTN_UP, BTN_DOWN, BTN_LEFT, BTN_RIGHT = 11, 12, 13, 14
BTN_OPTIONS = 6  # 🛑 Options 鍵 (用來長按關機)

print(f"嘗試連線到 {COM_PORT}...")
try:
    bt_serial = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    print("✅ 成功連線到無人機藍牙！")
except Exception as e:
    print(f"❌ 連線失敗: {e}"); exit()

pygame.init(); pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

# --- 系統變數初始化 ---
base_throttle = 0           
throttle_step = 5           
gripper_val = 127           
arm_state = 0               
offset_throttle, offset_yaw, offset_pitch, offset_roll = 0.0, 0.0, 0.0, 0.0
prev_up, prev_down, prev_left, prev_right = 0, 0, 0, 0
prev_btn_tri, prev_btn_sq = 0, 0

# ⏱️ 長按關機計時變數
exit_press_time = 0
is_exiting = False

# 🖥️ 顯示節流 (每 200ms 刷新一次，不卡 VSCode)
last_print_time = 0
PRINT_INTERVAL = 0.2

try:
    print("\n🚀 [單人任務模式] 終端機控制系統已上線！")
    
    while True:
        pygame.event.pump() 

        # 💀 [Options 鍵] 長按 3 秒安全下線邏輯
        if joystick.get_button(BTN_OPTIONS) == 1:
            if not is_exiting:
                is_exiting = True
                exit_press_time = time.time()
                print("\n⚠️ [警告] 偵測到關機指令！請繼續按住 3 秒鐘...")
            elif time.time() - exit_press_time >= 3.0:
                print("\n💀 [系統關機] 已達到 3 秒，正在切斷地面站連線...")
                break
        else:
            if is_exiting:
                is_exiting = False
                print("\n✅ [恢復] 已放開按鍵，關機指令取消。")

        # 🎯 [□ 鍵] 搖桿校準
        curr_sq = joystick.get_button(BTN_SQUARE)
        if curr_sq == 1 and prev_btn_sq == 0:
            offset_throttle = joystick.get_axis(1)
            offset_yaw      = joystick.get_axis(0)
            offset_pitch    = joystick.get_axis(3)
            offset_roll     = joystick.get_axis(2)
        prev_btn_sq = curr_sq

        # ⚙️ [方向鍵] 油門基準
        curr_up, curr_down = joystick.get_button(BTN_UP), joystick.get_button(BTN_DOWN)
        curr_left, curr_right = joystick.get_button(BTN_LEFT), joystick.get_button(BTN_RIGHT)
        if curr_right == 1 and prev_right == 0: throttle_step = min(20, throttle_step + 1)
        if curr_left == 1 and prev_left == 0:   throttle_step = max(1, throttle_step - 1)
        if curr_up == 1 and prev_up == 0:       base_throttle = min(255, base_throttle + throttle_step)
        if curr_down == 1 and prev_down == 0:   base_throttle = max(0, base_throttle - throttle_step)
        prev_up, prev_down, prev_left, prev_right = curr_up, curr_down, curr_left, curr_right

        raw_throttle = joystick.get_axis(1) - offset_throttle
        raw_yaw      = joystick.get_axis(0) - offset_yaw
        raw_pitch    = joystick.get_axis(3) - offset_pitch
        raw_roll     = joystick.get_axis(2) - offset_roll

        final_throttle = max(0, min(255, base_throttle + int(-raw_throttle * JOYSTICK_SENSITIVITY)))
        yaw_val   = int((max(-1.0, min(1.0, raw_yaw * YAW_SENS)) + 1.0) / 2.0 * 255)
        pitch_val = int((max(-1.0, min(1.0, -raw_pitch * TILT_SENS)) + 1.0) / 2.0 * 255)
        roll_val  = int((max(-1.0, min(1.0, raw_roll * TILT_SENS)) + 1.0) / 2.0 * 255)

        # 🦾 [L1/R1] 夾爪控制
        if joystick.get_button(BTN_L1) == 1: gripper_val -= STEP_SPEED
        elif joystick.get_button(BTN_R1) == 1: gripper_val += STEP_SPEED
        gripper_val = max(0, min(255, gripper_val))

        # ⚠️ [△ 鍵] 解鎖/上鎖
        curr_tri = joystick.get_button(BTN_TRI)
        if curr_tri == 1 and prev_btn_tri == 0:
            if arm_state == 0 and final_throttle <= 5:
                arm_state = 255
            else:
                arm_state = 0
        prev_btn_tri = curr_tri

        # 📦 打包發送
        data_packet = b'S' + bytes([final_throttle, yaw_val, pitch_val, roll_val, gripper_val, arm_state])
        bt_serial.write(data_packet)

        # 🖥️ 介面即時顯示 (節流版，不炸 VSCode)
        now = time.time()
        if not is_exiting and now - last_print_time >= PRINT_INTERVAL:
            arm_str = "解鎖" if arm_state == 255 else "上鎖"
            print(f"狀態:{arm_str} | 總油門:{final_throttle:3d} | 夾爪:{gripper_val:3d} | Y:{yaw_val:3d} P:{pitch_val:3d} R:{roll_val:3d}", end="\r")
            last_print_time = now

        time.sleep(0.04)

except KeyboardInterrupt:
    print("\n🛑 任務中止 (接收到鍵盤強制中斷)。")
finally:
    print("\n🔌 正在關閉藍牙通訊與手把硬體資源...")
    if 'bt_serial' in locals() and bt_serial.is_open:
        bt_serial.close()
    pygame.quit()
    print("✅ 地面控制系統已完全安全關閉。")