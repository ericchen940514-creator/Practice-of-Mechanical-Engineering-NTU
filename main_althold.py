import pygame
import serial
import time
import sys
import argparse
import threading

# ==========================================
# 啟動參數
# 用法：python main_althold.py --port COM5
# ==========================================
parser = argparse.ArgumentParser(description='無人機地面控制站（定高版）')
parser.add_argument('--port', default='COM4', help='藍牙 COM 埠（預設: COM4）')
args = parser.parse_args()

COM_PORT  = args.port
BAUD_RATE = 9600

JOYSTICK_SENSITIVITY = 60
TILT_SENS  = 0.6
YAW_SENS   = 0.5
STEP_SPEED = 5
DEAD_ZONE  = 0.05

ALT_STEP_DEFAULT = 10   # D-pad 每按一次調整幾 cm
ALT_MAX_CM       = 120  # VL53L0X 長距模式，定高實用上限

# 封包格式（8 bytes）：
#   [S] [throttle 0~255] [yaw] [pitch] [roll] [target_alt 0~120cm] [arm_val] [ah_val]
# throttle 永遠送當前真實油門，Arduino 以此為 PID base_throttle
# target_alt 取代原本 gripper 欄位，同時知道目標高度
# gripper 伺服馬達由 Arduino 本地控制，不走此封包

# --- 手把按鍵編號 ---
BTN_CIRCLE  = 1   # ○：切換定高開/關
BTN_SQUARE  = 2
BTN_TRI     = 3
BTN_L1      = 9
BTN_R1      = 10
BTN_UP, BTN_DOWN, BTN_LEFT, BTN_RIGHT = 11, 12, 13, 14
BTN_OPTIONS = 6
BTN_ESTOP_A = 4
BTN_ESTOP_B = 6


# ==========================================
# 藍牙工具
# ==========================================

def connect_serial():
    try:
        s = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
        print(f"✅ 成功連線到 {COM_PORT}")
        return s
    except Exception as e:
        print(f"❌ 連線失敗: {e}")
        return None

def send_packet(ser, packet):
    try:
        ser.reset_input_buffer()
        ser.write(packet)
        return True
    except Exception:
        return False

def safe_disconnect(ser, gripper_val=127):
    """送出停機封包後再關閉連線"""
    shutdown = b'S' + bytes([0, 127, 127, 127, 0, gripper_val, 0, 0])
    for _ in range(3):
        try:
            ser.write(shutdown)
            time.sleep(0.02)
        except Exception:
            pass
    try:
        ser.close()
    except Exception:
        pass

def apply_dead_zone(value):
    return 0.0 if abs(value) < DEAD_ZONE else value


# ==========================================
# 連線等待
# ==========================================

def wait_for_connection(mode, joystick=None, timeout=60):
    deadline   = time.time() + timeout
    next_retry = 0
    hold_start = None

    print(f"\n🔄 等待藍牙連線（最多 {timeout} 秒）[{COM_PORT}]...")
    if mode == 'gamepad':
        print("   Options 長按 3 秒 或 緊急停機（4+6）可中止")
    else:
        print("   X 長按 3 秒可中止等待")

    while time.time() < deadline:
        pygame.event.pump()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return None

        if mode == 'gamepad' and joystick:
            if joystick.get_button(BTN_ESTOP_A) and joystick.get_button(BTN_ESTOP_B):
                print("\n🚨 緊急停機觸發，中止等待。")
                return None
            held = joystick.get_button(BTN_OPTIONS)
        else:
            keys = pygame.key.get_pressed()
            held = keys[pygame.K_x]

        if held:
            if hold_start is None:
                hold_start = time.time()
                print("\n⚠️  偵測到中止指令，繼續按住 3 秒...")
            elif time.time() - hold_start >= 3.0:
                print("\n🛑 已中止等待。")
                return None
        else:
            hold_start = None

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


# ==========================================
# 輸入讀取
# ==========================================

def read_gamepad(joystick, state):
    """
    回傳 (channels, should_exit, estop)
    channels 包含 throttle/yaw/pitch/roll，
    以及 ah_val（定高開關）與 t_val（定高時為目標高度 cm）
    """
    # 緊急停機
    if joystick.get_button(BTN_ESTOP_A) and joystick.get_button(BTN_ESTOP_B):
        return None, False, True

    # 退出偵測（Options 長按 3 秒）
    should_exit = False
    if joystick.get_button(BTN_OPTIONS):
        if not state['is_exiting']:
            state['is_exiting'] = True
            state['exit_press_time'] = time.time()
            print("\n⚠️  偵測到退出指令，繼續按住 Options 3 秒...")
        elif time.time() - state['exit_press_time'] >= 3.0:
            should_exit = True
    else:
        if state['is_exiting']:
            state['is_exiting'] = False
            print("\n✅ 已放開按鍵，退出指令取消。")

    # ○ 鍵：切換定高開/關（邊緣觸發）
    curr_circle = joystick.get_button(BTN_CIRCLE)
    if curr_circle and not state['prev_circle']:
        state['alt_hold_active'] = not state['alt_hold_active']
        if state['alt_hold_active']:
            print(f"\n🔒 定高啟動！目標高度：{state['target_alt']} cm")
            print("   D-pad 上/下 調整目標高度  |  D-pad 左/右 調整步進量")
        else:
            print("\n🔓 定高關閉，回手動模式。")
    state['prev_circle'] = curr_circle

    # □ 鍵：搖桿校準（僅手動模式）
    curr_sq = joystick.get_button(BTN_SQUARE)
    if curr_sq and not state['prev_sq'] and not state['alt_hold_active']:
        state['offset'] = (
            joystick.get_axis(1),
            joystick.get_axis(0),
            joystick.get_axis(3),
            joystick.get_axis(2),
        )
        print("\n⚙️  搖桿校準完成。")
    state['prev_sq'] = curr_sq

    # 方向鍵：定高模式 → 調整目標高度 / 手動模式 → 調整基準油門
    curr_up    = joystick.get_button(BTN_UP)
    curr_down  = joystick.get_button(BTN_DOWN)
    curr_left  = joystick.get_button(BTN_LEFT)
    curr_right = joystick.get_button(BTN_RIGHT)

    if state['alt_hold_active']:
        # 定高模式：D-pad 上/下 調整目標高度
        if curr_up   and not state['prev_up']:
            state['target_alt'] = min(ALT_MAX_CM, state['target_alt'] + state['alt_step'])
            print(f"\n📡 目標高度 → {state['target_alt']} cm")
        if curr_down and not state['prev_down']:
            state['target_alt'] = max(5, state['target_alt'] - state['alt_step'])
            print(f"\n📡 目標高度 → {state['target_alt']} cm")
        # D-pad 左/右 調整高度步進量
        if curr_right and not state['prev_right']:
            state['alt_step'] = min(30, state['alt_step'] + 5)
            print(f"\n📐 高度步進 → {state['alt_step']} cm")
        if curr_left  and not state['prev_left']:
            state['alt_step'] = max(5, state['alt_step'] - 5)
            print(f"\n📐 高度步進 → {state['alt_step']} cm")
    else:
        # 手動模式：D-pad 上/下 調整基準油門（原邏輯）
        if curr_up    and not state['prev_up']:    state['base_throttle'] = min(255, state['base_throttle'] + state['throttle_step'])
        if curr_down  and not state['prev_down']:  state['base_throttle'] = max(0,   state['base_throttle'] - state['throttle_step'])
        if curr_right and not state['prev_right']: state['throttle_step'] = min(20,  state['throttle_step'] + 1)
        if curr_left  and not state['prev_left']:  state['throttle_step'] = max(1,   state['throttle_step'] - 1)

    state['prev_up'], state['prev_down'] = curr_up, curr_down
    state['prev_left'], state['prev_right'] = curr_left, curr_right

    # 搖桿軸（加死區與校準偏移）
    oy, ox, op, or_ = state['offset']
    raw_throttle = apply_dead_zone(joystick.get_axis(1) - oy)
    raw_yaw      = apply_dead_zone(joystick.get_axis(0) - ox)
    raw_pitch    = apply_dead_zone(joystick.get_axis(3) - op)
    raw_roll     = apply_dead_zone(joystick.get_axis(2) - or_)

    final_throttle = max(0, min(255, state['base_throttle'] + int(-raw_throttle * JOYSTICK_SENSITIVITY)))

    # △ 鍵：解鎖 / 上鎖
    curr_tri = joystick.get_button(BTN_TRI)
    if curr_tri and not state['prev_tri']:
        if state['arm_state'] == 0 and final_throttle <= 5:
            state['arm_state'] = 255
        else:
            state['arm_state'] = 0
    state['prev_tri'] = curr_tri

    # L1 / R1：夾爪（Arduino 直接驅動伺服馬達，不經過飛控）
    if joystick.get_button(BTN_L1): state['gripper_val'] = max(0,   state['gripper_val'] - STEP_SPEED)
    if joystick.get_button(BTN_R1): state['gripper_val'] = min(255, state['gripper_val'] + STEP_SPEED)

    # ── 封包內容決定 ──
    target_alt_byte = int(state['target_alt']) if state['alt_hold_active'] else 0
    ah_val          = 1 if state['alt_hold_active'] else 0

    channels = {
        'throttle':   final_throttle,
        'target_alt': target_alt_byte,
        'yaw':   int((max(-1.0, min(1.0, raw_yaw   * YAW_SENS))  + 1.0) / 2.0 * 255),
        'pitch': int((max(-1.0, min(1.0, -raw_pitch * TILT_SENS)) + 1.0) / 2.0 * 255),
        'roll':  int((max(-1.0, min(1.0, raw_roll   * TILT_SENS)) + 1.0) / 2.0 * 255),
        'ah_val': ah_val,
    }
    return channels, should_exit, False


def read_keyboard(keys, state):
    """
    鍵盤模式：H 鍵切換定高，Tab/Shift 在定高時調整目標高度
    """
    should_exit = False
    if keys[pygame.K_x]:
        if not state['is_exiting']:
            state['is_exiting'] = True
            state['exit_press_time'] = time.time()
            print("\n⚠️  偵測到退出指令，繼續按住 X 鍵 3 秒...")
        elif time.time() - state['exit_press_time'] >= 3.0:
            should_exit = True
    else:
        if state['is_exiting']:
            state['is_exiting'] = False
            print("\n✅ 已放開 X 鍵，退出指令取消。")

    # H 鍵：切換定高（邊緣觸發）
    curr_h = keys[pygame.K_h]
    if curr_h and not state['prev_h']:
        state['alt_hold_active'] = not state['alt_hold_active']
        if state['alt_hold_active']:
            print(f"\n🔒 定高啟動！目標高度：{state['target_alt']} cm")
            print("   Tab=高度↑  Shift=高度↓")
        else:
            print("\n🔓 定高關閉，回手動模式。")
    state['prev_h'] = curr_h

    curr_tab   = keys[pygame.K_TAB]
    curr_shift = keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]
    curr_c     = keys[pygame.K_c]
    curr_z     = keys[pygame.K_z]

    if state['alt_hold_active']:
        # 定高模式：Tab/Shift 調整目標高度
        if curr_tab   and not state['prev_tab']:
            state['target_alt'] = min(ALT_MAX_CM, state['target_alt'] + state['alt_step'])
            print(f"\n📡 目標高度 → {state['target_alt']} cm")
        if curr_shift and not state['prev_shift']:
            state['target_alt'] = max(5, state['target_alt'] - state['alt_step'])
            print(f"\n📡 目標高度 → {state['target_alt']} cm")
    else:
        # 手動模式：Tab/Shift 調整基準油門
        if curr_tab   and not state['prev_tab']:   state['base_throttle'] = min(255, state['base_throttle'] + state['throttle_step'])
        if curr_shift and not state['prev_shift']: state['base_throttle'] = max(0,   state['base_throttle'] - state['throttle_step'])
        if curr_c     and not state['prev_c']:     state['throttle_step'] = min(20,  state['throttle_step'] + 1)
        if curr_z     and not state['prev_z']:     state['throttle_step'] = max(1,   state['throttle_step'] - 1)

    state['prev_tab'], state['prev_shift'] = curr_tab, curr_shift
    state['prev_c'],   state['prev_z']     = curr_c,   curr_z

    raw_throttle = max(-1.0, min(1.0, (-1.0 if keys[pygame.K_w] else 0.0) + (1.0 if keys[pygame.K_s] else 0.0)))
    raw_yaw      = max(-1.0, min(1.0, (-1.0 if keys[pygame.K_a] else 0.0) + (1.0 if keys[pygame.K_d] else 0.0)))
    raw_pitch    = max(-1.0, min(1.0, (-1.0 if keys[pygame.K_UP]   else 0.0) + (1.0 if keys[pygame.K_DOWN]  else 0.0)))
    raw_roll     = max(-1.0, min(1.0, (-1.0 if keys[pygame.K_LEFT]  else 0.0) + (1.0 if keys[pygame.K_RIGHT] else 0.0)))

    final_throttle = max(0, min(255, state['base_throttle'] + int(-raw_throttle * JOYSTICK_SENSITIVITY)))

    curr_r = keys[pygame.K_r]
    if curr_r and not state['prev_r']:
        if state['arm_state'] == 0 and final_throttle <= 5:
            state['arm_state'] = 255
        else:
            state['arm_state'] = 0
    state['prev_r'] = curr_r

    # Q / E：夾爪（Arduino 直接驅動伺服馬達，不經過飛控）
    if keys[pygame.K_q]: state['gripper_val'] = max(0,   state['gripper_val'] - STEP_SPEED)
    if keys[pygame.K_e]: state['gripper_val'] = min(255, state['gripper_val'] + STEP_SPEED)

    target_alt_byte = int(state['target_alt']) if state['alt_hold_active'] else 0
    ah_val          = 1 if state['alt_hold_active'] else 0

    channels = {
        'throttle':   final_throttle,
        'target_alt': target_alt_byte,
        'yaw':   int((max(-1.0, min(1.0, raw_yaw   * YAW_SENS))  + 1.0) / 2.0 * 255),
        'pitch': int((max(-1.0, min(1.0, -raw_pitch * TILT_SENS)) + 1.0) / 2.0 * 255),
        'roll':  int((max(-1.0, min(1.0, raw_roll   * TILT_SENS)) + 1.0) / 2.0 * 255),
        'ah_val': ah_val,
    }
    return channels, should_exit


# ==========================================
# pygame 視窗狀態顯示
# ==========================================

def draw_status(screen, font, state, mode, channels, connected):
    screen.fill((20, 20, 20))

    conn_color = (80, 220, 80) if connected else (220, 80, 80)
    ah_color   = (80, 180, 255) if state['alt_hold_active'] else (160, 160, 160)

    if state['alt_hold_active']:
        throttle_str = f"目標高度: {state['target_alt']:3d} cm (步進:{state['alt_step']:2d}cm)"
    else:
        throttle_str = f"油門: {channels['throttle']:3d}  基準: {state['base_throttle']:3d}(步進:{state['throttle_step']:2d})"

    lines = [
        (f"模式: {'手把' if mode == 'gamepad' else '鍵盤'}   "
         f"連線: {'連線中' if connected else '斷線'}   "
         f"狀態: {'解鎖' if state['arm_state'] == 255 else '上鎖'}",
         conn_color),
        (f"定高: {'開啟 🔒' if state['alt_hold_active'] else '關閉'}   " + throttle_str,
         ah_color),
        (f"夾爪: {state['gripper_val']:3d}  (L1/R1)   Y: {channels['yaw']:3d}  P: {channels['pitch']:3d}  R: {channels['roll']:3d}   COM: {COM_PORT}",
         (180, 180, 180)),
        ("○/H=定高切換  D-pad=高度/油門  Options/X長按3秒=退出  4+6=緊急停機",
         (110, 110, 110)),
    ]

    for i, (text, color) in enumerate(lines):
        surf = font.render(text, True, color)
        screen.blit(surf, (12, 10 + i * 28))

    pygame.display.flip()


# ==========================================
# 主程式
# ==========================================

pygame.init()
pygame.joystick.init()

joystick = None
try:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    mode = 'gamepad'
    print(f"✅ 手把已連線：{joystick.get_name()}  → 手把模式")
except Exception:
    mode = 'keyboard'
    print("ℹ️  未偵測到手把  → 鍵盤模式")

screen = pygame.display.set_mode((600, 130))
pygame.display.set_caption(f"無人機控制站（定高版）[{COM_PORT}] — {'手把' if mode == 'gamepad' else '鍵盤'}模式")
font  = pygame.font.SysFont("Microsoft JhengHei", 18)
clock = pygame.time.Clock()

print(f"\n嘗試連線到 {COM_PORT}...")
bt_serial = wait_for_connection(mode, joystick, timeout=60)
if bt_serial is None:
    pygame.quit()
    sys.exit()

state = {
    'base_throttle': 0,
    'throttle_step': 5,
    'gripper_val': 127,
    'arm_state': 0,
    'is_exiting': False,
    'exit_press_time': 0,
    # 定高
    'alt_hold_active': False,
    'target_alt': 50,           # 預設目標高度 50 cm
    'alt_step': ALT_STEP_DEFAULT,
    # 手把
    'offset': (0.0, 0.0, 0.0, 0.0),
    'prev_sq': 0, 'prev_tri': 0, 'prev_circle': 0,
    'prev_up': 0, 'prev_down': 0, 'prev_left': 0, 'prev_right': 0,
    # 鍵盤
    'prev_tab': False, 'prev_shift': False,
    'prev_c': False, 'prev_z': False, 'prev_r': False, 'prev_h': False,
}
channels = {'throttle': 0, 'target_alt': 0, 'yaw': 127, 'pitch': 127, 'roll': 127, 'ah_val': 0}
reconnect_cooldown = 0
last_print_time    = 0

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

if mode == 'gamepad':
    print("\n🎮 手把模式已上線！（定高版）")
    print("△=解鎖/上鎖  ○=定高切換  □=搖桿校準")
    print("定高開啟時：D-pad上下=目標高度±  D-pad左右=步進量±")
    print("手動模式時：D-pad上下=基準油門±  Options長按3秒=退出  4+6=緊急停機\n")
else:
    print("\n⌨️  鍵盤模式已上線！（定高版）")
    print("W/S=油門  A/D=偏航  ↑↓=俯仰  ←→=翻滾  R=解鎖/上鎖  H=定高切換")
    print("定高開啟時：Tab=高度↑  Shift=高度↓")
    print("手動模式時：Tab=油門↑  Shift=油門↓  C/Z=步進±  X長按3秒=退出\n")

try:
    while True:
        pygame.event.pump()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt

        if mode == 'gamepad':
            channels, should_exit, estop = read_gamepad(joystick, state)
            if estop:
                print("\n🚨 [緊急停機] 觸發！立即送出停機指令...")
                if bt_serial and bt_serial.is_open:
                    estop_pkt = b'S' + bytes([0, 127, 127, 127, 0, state['gripper_val'], 0, 0])
                    for _ in range(5):
                        send_packet(bt_serial, estop_pkt)
                raise KeyboardInterrupt
        else:
            keys = pygame.key.get_pressed()
            channels, should_exit = read_keyboard(keys, state)

        if should_exit:
            break

        # 打包（9 bytes）
        # Gripper 由 Arduino 直接驅動伺服馬達（D6），不經過 IBUS 飛控
        data_packet = b'S' + bytes([
            channels['throttle'],       # 當前真實油門（Arduino 以此為 PID base_throttle）
            channels['yaw'],
            channels['pitch'],
            channels['roll'],
            channels['target_alt'],     # 定高時=目標高度cm；手動時=0
            state['gripper_val'],       # 夾爪（Arduino 本地驅動）
            state['arm_state'],
            channels['ah_val'],         # 定高開關
        ])

        with _serial_lock:
            _bt = bt_serial
        connected = _bt is not None and _bt.is_open
        if not connected:
            now = time.time()
            if not _reconnecting and now >= reconnect_cooldown:
                _reconnecting = True
                print("\n🔄 藍牙斷線，背景嘗試重連...", end="\r")
                threading.Thread(target=_do_reconnect, daemon=True).start()
        else:
            ok = send_packet(_bt, data_packet)
            if not ok:
                print("\n⚠️  串口寫入失敗，藍牙可能已斷線")
                safe_disconnect(_bt, state['gripper_val'])
                with _serial_lock:
                    bt_serial = None
                state['arm_state'] = 0
                state['alt_hold_active'] = False
                reconnect_cooldown = time.time() + 3.0

        now = time.time()
        if not state['is_exiting'] and now - last_print_time >= 0.2:
            conn_str = "連線中" if connected else "斷線中"
            arm_str  = "解鎖" if state['arm_state'] == 255 else "上鎖"
            ah_str   = f"定高:{state['target_alt']}cm" if state['alt_hold_active'] else f"手動:{channels['throttle']:3d}"
            print(f"[{conn_str}] {arm_str} | {ah_str} | 夾爪:{state['gripper_val']:3d} | Y:{channels['yaw']:3d} P:{channels['pitch']:3d} R:{channels['roll']:3d}", end="\r")
            last_print_time = now

        draw_status(screen, font, state, mode, channels, connected)
        clock.tick(25)

except KeyboardInterrupt:
    print("\n🛑 任務中止（Ctrl+C）。")
finally:
    print("\n🔌 正在關閉連線...")
    if bt_serial is not None and bt_serial.is_open:
        safe_disconnect(bt_serial)
    pygame.quit()
    print("✅ 已安全關閉。")
