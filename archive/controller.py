import pygame
import serial
import time
import threading

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
BTN_ESTOP_A = 4  # 🚨 緊急停機組合鍵（4 + 6 同時按）
BTN_ESTOP_B = 6

def connect_serial():
    """嘗試連線，失敗回傳 None"""
    try:
        s = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
        print(f"✅ 成功連線到 {COM_PORT}")
        return s
    except Exception as e:
        print(f"❌ 連線失敗: {e}")
        return None

def send_packet(ser, packet):
    """送封包，清 RX buffer，失敗回傳 False"""
    try:
        ser.reset_input_buffer()
        ser.write(packet)
        return True
    except Exception:
        return False

def wait_for_connection(joystick, timeout=60):
    """嘗試連線，最多等 timeout 秒。Options 長按 3 秒或緊急停機（4+6）可立即中止。"""
    deadline = time.time() + timeout
    options_press_time = None
    next_retry = 0  # 立即嘗試第一次

    print(f"\n🔄 等待藍牙連線（最多 {timeout} 秒）...")
    print("   Options 長按 3 秒 或 緊急停機（4+6）可中止\n")

    while time.time() < deadline:
        pygame.event.pump()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return None

        if joystick is not None:
            # 緊急停機（4+6 同時）→ 立即中止
            if joystick.get_button(BTN_ESTOP_A) and joystick.get_button(BTN_ESTOP_B):
                print("\n🚨 緊急停機觸發，中止等待。")
                return None

            # Options 長按 3 秒中止
            if joystick.get_button(BTN_OPTIONS):
                if options_press_time is None:
                    options_press_time = time.time()
                    print("\n⚠️  偵測到中止指令，繼續按住 Options 3 秒...")
                elif time.time() - options_press_time >= 3.0:
                    print("\n🛑 已中止等待。")
                    return None
            else:
                options_press_time = None

        # 每 3 秒嘗試連線一次
        if time.time() >= next_retry:
            ser = connect_serial()
            if ser is not None:
                return ser
            remaining = int(deadline - time.time())
            print(f"   ⏳ 剩餘 {remaining} 秒，3 秒後重試...", end="\r")
            next_retry = time.time() + 3.0

        time.sleep(0.05)

    print(f"\n❌ 超過 {timeout} 秒仍無法連線，結束程式。")
    return None

# --- 初始化 pygame + 手把 ---
pygame.init()
pygame.joystick.init()

try:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"✅ 手把已連線：{joystick.get_name()}")
except Exception as e:
    print(f"❌ 找不到手把: {e}")
    pygame.quit()
    exit()

# --- 初始連線（最多等待 60 秒）---
print(f"嘗試連線到 {COM_PORT}...")
bt_serial = wait_for_connection(joystick, timeout=60)
if bt_serial is None:
    pygame.quit()
    exit()

# --- 背景重連 ---
_serial_lock  = threading.Lock()
_reconnecting = False

def _do_reconnect():
    global bt_serial, _reconnecting, reconnect_cooldown
    new_ser = connect_serial()
    with _serial_lock:
        bt_serial = new_ser
        _reconnecting = False
        reconnect_cooldown = time.time() + 3.0

# --- 系統變數初始化 ---
base_throttle = 0
throttle_step = 5
gripper_val = 127
arm_state = 0
offset_throttle, offset_yaw, offset_pitch, offset_roll = 0.0, 0.0, 0.0, 0.0
prev_up, prev_down, prev_left, prev_right = 0, 0, 0, 0
prev_btn_tri, prev_btn_sq = 0, 0

exit_press_time = 0
is_exiting = False
last_print_time = 0
reconnect_cooldown = 0
PRINT_INTERVAL = 0.2

clock = pygame.time.Clock()

try:
    print("\n🚀 [單人任務模式] 終端機控制系統已上線！")

    while True:
        pygame.event.pump()

        # 🚨 緊急停機（按鈕 4 + 6 同時按，立即斷電）
        if joystick.get_button(BTN_ESTOP_A) and joystick.get_button(BTN_ESTOP_B):
            print("\n🚨 [緊急停機] 觸發！立即送出停機指令...")
            estop_packet = b'S' + bytes([0, 127, 127, 127, gripper_val, 0])
            if bt_serial is not None and bt_serial.is_open:
                for _ in range(5):  # 連送 5 次確保收到
                    send_packet(bt_serial, estop_packet)
            print("💀 已送出停機指令，強制結束程式。")
            raise KeyboardInterrupt

        # 💀 [Options 鍵] 長按 3 秒安全下線
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

        # 📦 打包發送（清空積壓封包，避免延遲堆疊）
        data_packet = b'S' + bytes([final_throttle, yaw_val, pitch_val, roll_val, gripper_val, arm_state])

        with _serial_lock:
            _bt = bt_serial

        if _bt is None or not _bt.is_open:
            now = time.time()
            if not _reconnecting and now >= reconnect_cooldown:
                _reconnecting = True
                print("\n🔄 藍牙斷線，背景嘗試重連...", end="\r")
                threading.Thread(target=_do_reconnect, daemon=True).start()
        else:
            ok = send_packet(_bt, data_packet)
            if not ok:
                print("\n⚠️  串口寫入失敗，藍牙可能已斷線")
                try:
                    _bt.close()
                except Exception:
                    pass
                with _serial_lock:
                    bt_serial = None
                arm_state = 0  # 斷線時強制上鎖

        # 🖥️ 介面即時顯示
        now = time.time()
        if not is_exiting and now - last_print_time >= PRINT_INTERVAL:
            conn_str = "連線中" if (bt_serial and bt_serial.is_open) else "斷線中"
            arm_str  = "解鎖" if arm_state == 255 else "上鎖"
            print(f"[{conn_str}] 狀態:{arm_str} | 基準:{base_throttle:3d}(步進:{throttle_step:2d}) | 總油門:{final_throttle:3d} | 夾爪:{gripper_val:3d} | Y:{yaw_val:3d} P:{pitch_val:3d} R:{roll_val:3d}", end="\r")
            last_print_time = now

        clock.tick(25)  # 精確鎖定 25 Hz

except KeyboardInterrupt:
    print("\n🛑 任務中止 (接收到鍵盤強制中斷)。")
finally:
    print("\n🔌 正在關閉藍牙通訊與手把硬體資源...")
    if bt_serial is not None and bt_serial.is_open:
        bt_serial.close()
    pygame.quit()
    print("✅ 地面控制系統已完全安全關閉。")
