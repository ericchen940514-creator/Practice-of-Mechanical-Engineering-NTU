import os
os.environ['SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS'] = '1'  # 允許手把在背景（非焦點）時繼續運作

import pygame
import serial
import time
import sys
import argparse
import threading
import keyboard as kb

# ==========================================
# 啟動參數（COM 埠可從命令列指定）
# 用法：python main.py --port COM5
# ==========================================
parser = argparse.ArgumentParser(description='無人機地面控制站')
parser.add_argument('--port', default='COM4', help='藍牙 COM 埠（預設: COM8）')
args = parser.parse_args()

COM_PORT  = args.port
BAUD_RATE = 9600

JOYSTICK_SENSITIVITY = 60
TILT_SENS  = 0.6
YAW_SENS   = 0.5
STEP_SPEED = 5
DEAD_ZONE  = 0.05   # 搖桿死區（低於此值視為置中）

# --- 手把按鍵編號 ---
BTN_SQUARE  = 2
BTN_TRI     = 3
BTN_L1      = 9
BTN_R1      = 10
BTN_UP, BTN_DOWN, BTN_LEFT, BTN_RIGHT = 11, 12, 13, 14
BTN_OPTIONS = 6
BTN_ESTOP_A = 4   # 緊急停機組合鍵 A（4+6 同時）
BTN_ESTOP_B = 6   # 緊急停機組合鍵 B


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

def safe_disconnect(ser, gripper_val):
    """送出停機封包後再關閉連線，確保無人機收到歸零指令"""
    shutdown = b'S' + bytes([0, 127, 127, 127, gripper_val, 0])
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

# ── 全域鍵盤長按重複計時（不需要 pygame 視窗焦點）──
_key_held_start  = {}
_key_last_repeat = {}

def kb_triggered(key, initial=0.25, repeat=0.10):
    """回傳 True 表示此按鍵現在應觸發一次動作（初始按下 + 長按自動重複）"""
    if not kb.is_pressed(key):
        _key_held_start.pop(key, None)
        _key_last_repeat.pop(key, None)
        return False
    now = time.time()
    if key not in _key_held_start:
        _key_held_start[key] = now
        _key_last_repeat[key] = now
        return True
    if now - _key_held_start[key] >= initial:
        if now - _key_last_repeat[key] >= repeat:
            _key_last_repeat[key] = now
            return True
    return False


# ==========================================
# 連線等待（手把 / 鍵盤通用）
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

        # 偵測中止按鍵
        if mode == 'gamepad' and joystick:
            try:
                if joystick.get_button(BTN_ESTOP_A) and joystick.get_button(BTN_ESTOP_B):
                    print("\n🚨 緊急停機觸發，中止等待。")
                    return None
                held = joystick.get_button(BTN_OPTIONS)
            except pygame.error:
                print("\n⚠️ 手把斷線，中止等待。")
                return None
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
    從手把讀取輸入。
    回傳 (channels, should_exit, estop)
      channels: dict with throttle/yaw/pitch/roll，或 None（estop 時）
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

    # □ 鍵：搖桿校準
    curr_sq = joystick.get_button(BTN_SQUARE)
    if curr_sq and not state['prev_sq']:
        state['offset'] = (
            joystick.get_axis(1),
            joystick.get_axis(0),
            joystick.get_axis(3),
            joystick.get_axis(2),
        )
    state['prev_sq'] = curr_sq

    # 方向鍵：油門基準 / 步進
    curr_up    = joystick.get_button(BTN_UP)
    curr_down  = joystick.get_button(BTN_DOWN)
    curr_left  = joystick.get_button(BTN_LEFT)
    curr_right = joystick.get_button(BTN_RIGHT)
    if curr_up    and not state['prev_up']:    state['base_throttle'] = min(255, state['base_throttle'] + state['throttle_step'])
    if curr_down  and not state['prev_down']:  state['base_throttle'] = max(0,   state['base_throttle'] - state['throttle_step'])
    if curr_right and not state['prev_right']: state['throttle_step'] = min(20,  state['throttle_step'] + 1)
    if curr_left  and not state['prev_left']:  state['throttle_step'] = max(1,   state['throttle_step'] - 1)
    state['prev_up'], state['prev_down'] = curr_up, curr_down
    state['prev_left'], state['prev_right'] = curr_left, curr_right

    # 搖桿軸（套用死區與校準偏移）
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

    # L1 / R1：夾爪
    if joystick.get_button(BTN_L1): state['gripper_val'] = max(0,   state['gripper_val'] - STEP_SPEED)
    if joystick.get_button(BTN_R1): state['gripper_val'] = min(255, state['gripper_val'] + STEP_SPEED)

    channels = {
        'throttle': final_throttle,
        'yaw':   int((max(-1.0, min(1.0, raw_yaw   * YAW_SENS))  + 1.0) / 2.0 * 255),
        'pitch': int((max(-1.0, min(1.0, -raw_pitch * TILT_SENS)) + 1.0) / 2.0 * 255),
        'roll':  int((max(-1.0, min(1.0, raw_roll   * TILT_SENS)) + 1.0) / 2.0 * 255),
    }
    return channels, should_exit, False


def read_keyboard(state):
    """鍵盤輸入讀取（使用全域 keyboard 函式庫，不需要 pygame 視窗焦點）"""
    # X 長按 3 秒退出
    should_exit = False
    if kb.is_pressed('x'):
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

    # Tab / Shift / C / Z：初始按下 + 長按自動重複
    if kb_triggered('tab'):
        state['base_throttle'] = min(255, state['base_throttle'] + state['throttle_step'])
    if kb_triggered('shift'):
        state['base_throttle'] = max(0, state['base_throttle'] - state['throttle_step'])
    if kb_triggered('c'):
        state['throttle_step'] = min(20, state['throttle_step'] + 1)
    if kb_triggered('z'):
        state['throttle_step'] = max(1, state['throttle_step'] - 1)

    # W/S/A/D/方向鍵
    raw_throttle = max(-1.0, min(1.0, (-1.0 if kb.is_pressed('w') else 0.0) + (1.0 if kb.is_pressed('s') else 0.0)))
    raw_yaw      = max(-1.0, min(1.0, (-1.0 if kb.is_pressed('a') else 0.0) + (1.0 if kb.is_pressed('d') else 0.0)))
    raw_pitch    = max(-1.0, min(1.0, (-1.0 if kb.is_pressed('up')   else 0.0) + (1.0 if kb.is_pressed('down')  else 0.0)))
    raw_roll     = max(-1.0, min(1.0, (-1.0 if kb.is_pressed('left') else 0.0) + (1.0 if kb.is_pressed('right') else 0.0)))

    final_throttle = max(0, min(255, state['base_throttle'] + int(-raw_throttle * JOYSTICK_SENSITIVITY)))

    # R：解鎖 / 上鎖
    curr_r = kb.is_pressed('r')
    if curr_r and not state['prev_r']:
        if state['arm_state'] == 0 and final_throttle <= 5:
            state['arm_state'] = 255
        else:
            state['arm_state'] = 0
    state['prev_r'] = curr_r

    # Q / E：夾爪
    if kb.is_pressed('q'): state['gripper_val'] = max(0,   state['gripper_val'] - STEP_SPEED)
    if kb.is_pressed('e'): state['gripper_val'] = min(255, state['gripper_val'] + STEP_SPEED)

    channels = {
        'throttle': final_throttle,
        'yaw':   int((max(-1.0, min(1.0, raw_yaw   * YAW_SENS))  + 1.0) / 2.0 * 255),
        'pitch': int((max(-1.0, min(1.0, -raw_pitch * TILT_SENS)) + 1.0) / 2.0 * 255),
        'roll':  int((max(-1.0, min(1.0, raw_roll   * TILT_SENS)) + 1.0) / 2.0 * 255),
    }
    return channels, should_exit


# ==========================================
# pygame 視窗狀態顯示
# ==========================================

def draw_status(screen, font, state, mode, channels, connected):
    screen.fill((20, 20, 20))

    conn_color = (80, 220, 80) if connected else (220, 80, 80)

    lines = [
        (f"模式: {'手把' if mode == 'gamepad' else '鍵盤'}   "
         f"連線: {'連線中' if connected else '斷線'}   "
         f"狀態: {'解鎖' if state['arm_state'] == 255 else '上鎖'}",
         conn_color),
        (f"油門: {channels['throttle']:3d}  "
         f"基準: {state['base_throttle']:3d}(步進:{state['throttle_step']:2d})  "
         f"夾爪: {state['gripper_val']:3d}",
         (200, 200, 200)),
        (f"Y: {channels['yaw']:3d}  P: {channels['pitch']:3d}  R: {channels['roll']:3d}  "
         f"COM: {COM_PORT}",
         (180, 180, 180)),
        ("退出: 長按 Options / X 鍵 3 秒   強制中止: Ctrl+C",
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
pygame.key.set_repeat(250, 100)   # 長按自動重複（初始 250ms，之後每 100ms）

# 自動偵測手把，沒有就改用鍵盤
joystick = None
try:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    mode = 'gamepad'
    print(f"✅ 手把已連線：{joystick.get_name()}  → 手把模式")
except Exception:
    mode = 'keyboard'
    print("ℹ️  未偵測到手把  → 鍵盤模式")

screen = pygame.display.set_mode((560, 130))
pygame.display.set_caption(f"無人機控制站 [{COM_PORT}] — {'手把' if mode == 'gamepad' else '鍵盤'}模式")
font  = pygame.font.SysFont("Microsoft JhengHei", 18)
clock = pygame.time.Clock()

print(f"\n嘗試連線到 {COM_PORT}...")
bt_serial = wait_for_connection(mode, joystick, timeout=60)
if bt_serial is None:
    pygame.quit()
    sys.exit()

# 共用狀態字典
state = {
    'base_throttle': 0,
    'throttle_step': 5,
    'gripper_val': 127,
    'arm_state': 0,
    'is_exiting': False,
    'exit_press_time': 0,
    # 手把
    'offset': (0.0, 0.0, 0.0, 0.0),
    'prev_sq': 0, 'prev_tri': 0,
    'prev_up': 0, 'prev_down': 0, 'prev_left': 0, 'prev_right': 0,
    # 鍵盤（toggle 類仍需 prev；Tab/Shift/C/Z 改用 KEYDOWN，不需要 prev）
    'prev_r': False,
}
channels = {'throttle': 0, 'yaw': 127, 'pitch': 127, 'roll': 127}
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
    print("\n🎮 手把模式已上線！")
    print("方向鍵=油門基準  □=搖桿校準  △=解鎖/上鎖  L1/R1=夾爪  Options長按3秒=退出  4+6=緊急停機\n")
else:
    print("\n⌨️  鍵盤模式已上線！（全域輸入，不需點選視窗）")
    print("W/S=油門  A/D=偏航  ↑↓=俯仰  ←→=翻滾  Tab=基準↑  Shift=基準↓  C/Z=步進±")
    print("Q=夾爪閉  E=夾爪開  R=解鎖/上鎖  X長按3秒=退出\n")

try:
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt

        # 讀取輸入
        if mode == 'gamepad':
            try:
                channels, should_exit, estop = read_gamepad(joystick, state)
            except pygame.error as e:
                print(f"\n⚠️ 手把讀取失敗（{e}），安全退出...")
                should_exit, estop = True, False
                channels = {'throttle': 0, 'yaw': 127, 'pitch': 127, 'roll': 127}
            if estop:
                print("\n🚨 [緊急停機] 觸發！立即送出停機指令...")
                if bt_serial and bt_serial.is_open:
                    estop_pkt = b'S' + bytes([0, 127, 127, 127, state['gripper_val'], 0])
                    for _ in range(5):
                        send_packet(bt_serial, estop_pkt)
                raise KeyboardInterrupt
        else:
            channels, should_exit = read_keyboard(state)

        if should_exit:
            break

        # 打包
        data_packet = b'S' + bytes([
            channels['throttle'],
            channels['yaw'],
            channels['pitch'],
            channels['roll'],
            state['gripper_val'],
            state['arm_state'],
        ])

        # 發送 / 重連
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
                reconnect_cooldown = time.time() + 3.0

        # 終端機狀態列
        now = time.time()
        if not state['is_exiting'] and now - last_print_time >= 0.2:
            conn_str = "連線中" if connected else "斷線中"
            arm_str  = "解鎖" if state['arm_state'] == 255 else "上鎖"
            print(f"[{conn_str}] {arm_str} | 基準:{state['base_throttle']:3d}(步進:{state['throttle_step']:2d}) | 油門:{channels['throttle']:3d} | 夾爪:{state['gripper_val']:3d} | Y:{channels['yaw']:3d} P:{channels['pitch']:3d} R:{channels['roll']:3d}",
                  end="\r", flush=True)
            last_print_time = now

        # 視窗繪製
        try:
            draw_status(screen, font, state, mode, channels, connected)
        except pygame.error:
            pass
        clock.tick(25)

except KeyboardInterrupt:
    print("\n🛑 任務中止（Ctrl+C）。")
except pygame.error as e:
    print(f"\n💥 pygame 錯誤導致程式終止：{e}")
finally:
    print("\n🔌 正在關閉連線...")
    if bt_serial is not None and bt_serial.is_open:
        safe_disconnect(bt_serial, state['gripper_val'])
    pygame.quit()
    print("✅ 已安全關閉。")
