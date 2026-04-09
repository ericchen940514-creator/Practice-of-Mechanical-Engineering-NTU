import pygame
import serial
import time

# ==========================================
# 鍵盤控制模式（無手把備用）
# ==========================================
COM_PORT  = 'COM4'   # ← 改成你的藍牙傳出 COM 埠
BAUD_RATE = 9600

JOYSTICK_SENSITIVITY = 60   # W/S 按住時的油門增減量
TILT_SENS = 0.6             # 俯仰 / 翻滾靈敏度
YAW_SENS  = 0.5             # 偏航靈敏度
STEP_SPEED = 5              # 夾爪開合速度

# 按鍵對應說明：
# W / S          → 左搖桿 上/下（油門增減）
# A / D          → 左搖桿 左/右（偏航）
# ↑ / ↓          → 右搖桿 上/下（俯仰）
# ← / →          → 右搖桿 左/右（翻滾）
# Tab / Shift    → 基準油門 上/下
# C / Z          → 每次調整的步進量 增大/縮小
# Q / E          → 夾爪 閉合/張開
# R              → 解鎖 / 上鎖（油門須歸零才能解鎖）
# X 長按 3 秒    → 安全退出
# ==========================================

print(f"嘗試連線到 {COM_PORT}...")
try:
    bt_serial = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    print("✅ 成功連線到無人機藍牙！")
except Exception as e:
    print(f"❌ 連線失敗: {e}"); exit()

pygame.init()
screen = pygame.display.set_mode((360, 80))
pygame.display.set_caption("鍵盤控制模式 ← 請點選此視窗再操作")

# --- 系統變數初始化 ---
base_throttle = 0
throttle_step = 5
gripper_val   = 127
arm_state     = 0

prev_tab = prev_shift = prev_z = prev_c = prev_r = False
exit_press_time = 0
is_exiting = False
last_print_time = 0

print("\n⌨️  [鍵盤控制模式] 已上線！請點選彈出的小視窗後再操作。")
print("W/S=油門  A/D=偏航  ↑↓=俯仰  ←→=翻滾")
print("Tab=油門↑  Shift=油門↓  C=步進↑  Z=步進↓")
print("Q=夾爪閉  E=夾爪開  R=解鎖/上鎖  X長按3秒=退出\n")

try:
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt

        keys = pygame.key.get_pressed()

        # 💀 X 長按 3 秒安全退出
        if keys[pygame.K_x]:
            if not is_exiting:
                is_exiting = True
                exit_press_time = time.time()
                print("\n⚠️  偵測到退出指令，請繼續按住 X 鍵 3 秒...")
            elif time.time() - exit_press_time >= 3.0:
                print("\n💀 已達 3 秒，正在退出...")
                break
        else:
            if is_exiting:
                is_exiting = False
                print("\n✅ 已放開 X 鍵，退出指令取消。")

        # ⚙️ Tab / Shift：基準油門（邊緣觸發）
        curr_tab   = keys[pygame.K_TAB]
        curr_shift = keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]
        curr_c     = keys[pygame.K_c]
        curr_z     = keys[pygame.K_z]
        curr_r     = keys[pygame.K_r]

        if curr_tab   and not prev_tab:   base_throttle = min(255, base_throttle + throttle_step)
        if curr_shift and not prev_shift: base_throttle = max(0,   base_throttle - throttle_step)
        if curr_c     and not prev_c:     throttle_step = min(20,  throttle_step + 1)
        if curr_z     and not prev_z:     throttle_step = max(1,   throttle_step - 1)

        prev_tab, prev_shift, prev_c, prev_z = curr_tab, curr_shift, curr_c, curr_z

        # 🕹️ 左搖桿（W/S = 油門, A/D = 偏航）
        raw_throttle = (-1.0 if keys[pygame.K_w] else 0.0) + (1.0 if keys[pygame.K_s] else 0.0)
        raw_yaw      = (-1.0 if keys[pygame.K_a] else 0.0) + (1.0 if keys[pygame.K_d] else 0.0)
        raw_throttle = max(-1.0, min(1.0, raw_throttle))
        raw_yaw      = max(-1.0, min(1.0, raw_yaw))

        # 🕹️ 右搖桿（↑↓ = 俯仰, ←→ = 翻滾）
        raw_pitch = (-1.0 if keys[pygame.K_UP]    else 0.0) + (1.0 if keys[pygame.K_DOWN]  else 0.0)
        raw_roll  = (-1.0 if keys[pygame.K_LEFT]   else 0.0) + (1.0 if keys[pygame.K_RIGHT] else 0.0)
        raw_pitch = max(-1.0, min(1.0, raw_pitch))
        raw_roll  = max(-1.0, min(1.0, raw_roll))

        # 計算最終 6 通道數值（0~255）
        final_throttle = max(0, min(255, base_throttle + int(-raw_throttle * JOYSTICK_SENSITIVITY)))
        yaw_val   = int((max(-1.0, min(1.0, raw_yaw   * YAW_SENS))  + 1.0) / 2.0 * 255)
        pitch_val = int((max(-1.0, min(1.0, -raw_pitch * TILT_SENS)) + 1.0) / 2.0 * 255)
        roll_val  = int((max(-1.0, min(1.0, raw_roll   * TILT_SENS)) + 1.0) / 2.0 * 255)

        # ⚠️ R 鍵：解鎖 / 上鎖（邊緣觸發，油門須歸零才能解鎖）
        if curr_r and not prev_r:
            if arm_state == 0 and final_throttle <= 5:
                arm_state = 255
            else:
                arm_state = 0
        prev_r = curr_r

        # 🦾 Q/E：夾爪開合
        if keys[pygame.K_q]: gripper_val = max(0,   gripper_val - STEP_SPEED)
        if keys[pygame.K_e]: gripper_val = min(255, gripper_val + STEP_SPEED)

        # 📦 打包發送
        data_packet = b'S' + bytes([final_throttle, yaw_val, pitch_val, roll_val, gripper_val, arm_state])
        bt_serial.write(data_packet)

        # 🖥️ 即時狀態顯示
        now = time.time()
        if not is_exiting and now - last_print_time >= 0.2:
            arm_str = "解鎖" if arm_state == 255 else "上鎖"
            print(f"狀態:{arm_str} | 基準:{base_throttle:3d}(步進:{throttle_step:2d}) | 總油門:{final_throttle:3d} | 夾爪:{gripper_val:3d} | Y:{yaw_val:3d} P:{pitch_val:3d} R:{roll_val:3d}", end="\r")
            last_print_time = now

        time.sleep(0.04)

except KeyboardInterrupt:
    print("\n🛑 任務中止（Ctrl+C）。")
finally:
    print("\n🔌 正在關閉連線...")
    if 'bt_serial' in locals() and bt_serial.is_open:
        bt_serial.close()
    pygame.quit()
    print("✅ 已安全關閉。")
