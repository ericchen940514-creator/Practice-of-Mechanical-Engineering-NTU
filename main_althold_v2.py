import os
os.environ['SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS'] = '1'  # 允許手把在背景（非焦點）時繼續運作

import pygame
import serial
import time
import sys
import argparse
import threading
from collections import deque
import keyboard as kb
try:
    import pid_logger
except ImportError:
    class pid_logger:
        start  = staticmethod(lambda *_: None)
        stop   = staticmethod(lambda *_: None)
        record = staticmethod(lambda *_: None)

__version__ = "0.4.16.0"

# ==========================================
# 啟動參數
# ==========================================
parser = argparse.ArgumentParser(description='無人機地面控制站（定高版）')
parser.add_argument('--port', default='COM4', help='藍牙 COM 埠（預設: COM4）')
args = parser.parse_args()

COM_PORT  = args.port
BAUD_RATE = 9600

JOYSTICK_SENSITIVITY = 60
TILT_SENS  = 0.3
YAW_SENS   = 0.25
STEP_SPEED = 5
DEAD_ZONE  = 0.05

ALT_STEP_DEFAULT = 10
ALT_MAX_CM       = 120

# ── 手把按鍵編號 ──
BTN_CIRCLE = 1
BTN_SQUARE = 2
BTN_TRI    = 3
BTN_L1     = 9
BTN_R1     = 10
BTN_UP, BTN_DOWN, BTN_LEFT, BTN_RIGHT = 11, 12, 13, 14
BTN_OPTIONS   = 6
BTN_ESTOP_A   = 4
BTN_ESTOP_B   = 6

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
        ser.write(packet)
        return True
    except Exception:
        return False

def make_packet(thr, yaw, pitch, roll, alt, grip, arm, ah):
    payload = bytes([thr, yaw, pitch, roll, alt, grip, arm, ah])
    chk = 0
    for b in payload:
        chk ^= b
    return b'S' + payload + bytes([chk])

def safe_disconnect(ser, gripper_val=127):
    shutdown = make_packet(0, 127, 127, 127, 0, gripper_val, 0, 0)
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
# 連線等待
# ==========================================
def wait_for_connection(mode, joystick=None, timeout=60):
    deadline   = time.time() + timeout
    next_retry = 0
    hold_start = None
    print(f"\n🔄 等待藍牙連線（最多 {timeout} 秒）[{COM_PORT}]...")
    if mode == 'gamepad':
        print(" Options 長按 3 秒 或 緊急停機（4+6）可中止")
    else:
        print(" X 長按 3 秒可中止等待")

    while time.time() < deadline:
        pygame.event.pump()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return None

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
                print("\n⚠️ 偵測到中止指令，繼續按住 3 秒...")
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
            print(f" ⏳ 剩餘 {remaining} 秒，3 秒後重試...", end="\r")
            next_retry = time.time() + 3.0

        time.sleep(0.05)

    print(f"\n❌ 超過 {timeout} 秒仍無法連線，結束程式。")
    return None

# ==========================================
# 輸入讀取
# ==========================================
def read_gamepad(joystick, state):
    # 緊急停機
    if joystick.get_button(BTN_ESTOP_A) and joystick.get_button(BTN_ESTOP_B):
        return None, False, True

    # Options 長按 3 秒退出
    should_exit = False
    if joystick.get_button(BTN_OPTIONS):
        if not state['is_exiting']:
            state['is_exiting']      = True
            state['exit_press_time'] = time.time()
            print("\n⚠️ 偵測到退出指令，繼續按住 Options 3 秒...")
        elif time.time() - state['exit_press_time'] >= 3.0:
            should_exit = True
    else:
        if state['is_exiting']:
            state['is_exiting'] = False
            print("\n✅ 已放開按鍵，退出指令取消。")

    # ○ 鍵：切換定高（邊緣觸發）
    curr_circle = joystick.get_button(BTN_CIRCLE)
    if curr_circle and not state['prev_circle']:
        state['alt_hold_active'] = not state['alt_hold_active']
        if state['alt_hold_active']:
            # 快照當下實際總油門（步進 + 搖桿）作為定高基準
            oy = state['offset'][0]
            raw_t = apply_dead_zone(joystick.get_axis(1) - oy)
            state['base_throttle'] = max(0, min(255,
                state['base_throttle'] + int(-raw_t * JOYSTICK_SENSITIVITY)))
            snap = get_alt_snapshot()
            if snap >= 0:
                state['target_alt'] = round(max(5, min(ALT_MAX_CM, snap)))
                print(f"\n🔒 定高啟動！目標高度：{state['target_alt']} cm")
                print(" D-pad 上/下 調整目標高度 | D-pad 左/右 調整步進量")
                pid_logger.start(state['target_alt'])
            else:
                state['alt_hold_active'] = False
                print("\n⚠️ 感測器尚無資料，無法切入定高。")
        else:
            pid_logger.stop()
            print("\n🔓 定高關閉，回手動模式（等待 Arduino 同步基準油門）")
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
        print("\n⚙️ 搖桿校準完成。")
    state['prev_sq'] = curr_sq

    # 方向鍵
    curr_up    = joystick.get_button(BTN_UP)
    curr_down  = joystick.get_button(BTN_DOWN)
    curr_left  = joystick.get_button(BTN_LEFT)
    curr_right = joystick.get_button(BTN_RIGHT)

    if state['alt_hold_active']:
        if curr_up    and not state['prev_up']:
            state['target_alt'] = min(ALT_MAX_CM, state['target_alt'] + state['alt_step'])
            print(f"\n📡 目標高度 → {state['target_alt']} cm")
        if curr_down  and not state['prev_down']:
            state['target_alt'] = max(5, state['target_alt'] - state['alt_step'])
            print(f"\n📡 目標高度 → {state['target_alt']} cm")
        if curr_right and not state['prev_right']:
            state['target_alt'] = min(ALT_MAX_CM, state['target_alt'] + 3)
            print(f"\n📡 目標高度 → {state['target_alt']} cm")
        if curr_left  and not state['prev_left']:
            state['target_alt'] = max(5, state['target_alt'] - 3)
            print(f"\n📡 目標高度 → {state['target_alt']} cm")
    else:
        if curr_up    and not state['prev_up']:
            state['base_throttle'] = min(255, state['base_throttle'] + state['throttle_step'])
        if curr_down  and not state['prev_down']:
            state['base_throttle'] = max(0,   state['base_throttle'] - state['throttle_step'])
        if curr_right and not state['prev_right']:
            state['throttle_step'] = min(20, state['throttle_step'] + 1)
        if curr_left  and not state['prev_left']:
            state['throttle_step'] = max(1,  state['throttle_step'] - 1)

    state['prev_up'],    state['prev_down']  = curr_up,    curr_down
    state['prev_left'],  state['prev_right'] = curr_left,  curr_right

    # 搖桿軸（加死區與校準偏移）
    oy, ox, op, or_ = state['offset']
    raw_throttle = apply_dead_zone(joystick.get_axis(1) - oy)
    raw_yaw      = apply_dead_zone(joystick.get_axis(0) - ox)
    raw_pitch    = apply_dead_zone(joystick.get_axis(3) - op)
    raw_roll     = apply_dead_zone(joystick.get_axis(2) - or_)

    # ── 定高模式：送固定 base_throttle，不加搖桿偏移 ──
    # ── 手動模式：base_throttle + 搖桿暫時調整 ──
    if state['alt_hold_active']:
        final_throttle = state['base_throttle']
    else:
        final_throttle = max(0, min(255,
            state['base_throttle'] + int(-raw_throttle * JOYSTICK_SENSITIVITY)))

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

    target_alt_byte = int(state['target_alt']) if state['alt_hold_active'] else 0
    ah_val          = 1 if state['alt_hold_active'] else 0

    channels = {
        'throttle':   final_throttle,
        'target_alt': target_alt_byte,
        'yaw':   int((max(-1.0, min(1.0,  raw_yaw   * YAW_SENS )) + 1.0) / 2.0 * 255),
        'pitch': int((max(-1.0, min(1.0, -raw_pitch  * TILT_SENS)) + 1.0) / 2.0 * 255),
        'roll':  int((max(-1.0, min(1.0,  raw_roll   * TILT_SENS)) + 1.0) / 2.0 * 255),
        'ah_val': ah_val,
    }
    return channels, should_exit, False

def read_keyboard(state):
    """鍵盤輸入讀取（使用全域 keyboard 函式庫，不需要 pygame 視窗焦點）"""
    # X 長按 3 秒退出
    should_exit = False
    if kb.is_pressed('x'):
        if not state['is_exiting']:
            state['is_exiting']      = True
            state['exit_press_time'] = time.time()
            print("\n⚠️ 偵測到退出指令，繼續按住 X 鍵 3 秒...")
        elif time.time() - state['exit_press_time'] >= 3.0:
            should_exit = True
    else:
        if state['is_exiting']:
            state['is_exiting'] = False
            print("\n✅ 已放開 X 鍵，退出指令取消。")

    # H：切換定高（edge trigger）
    curr_h = kb.is_pressed('h')
    if curr_h and not state['prev_h']:
        state['alt_hold_active'] = not state['alt_hold_active']
        if state['alt_hold_active']:
            # 快照當下實際總油門（步進 + W/S 按鍵）作為定高基準
            raw_t = max(-1.0, min(1.0,
                (-1.0 if kb.is_pressed('w') else 0.0) +
                ( 1.0 if kb.is_pressed('s') else 0.0)))
            state['base_throttle'] = max(0, min(255,
                state['base_throttle'] + int(-raw_t * JOYSTICK_SENSITIVITY)))
            snap = get_alt_snapshot()
            if snap >= 0:
                state['target_alt'] = round(max(5, min(ALT_MAX_CM, snap)))
                print(f"\n🔒 定高啟動！目標高度：{state['target_alt']} cm")
                print(" Tab=高度↑ Shift=高度↓")
                pid_logger.start(state['target_alt'])
            else:
                state['alt_hold_active'] = False
                print("\n⚠️ 感測器尚無資料，無法切入定高。")
        else:
            pid_logger.stop()
            print("\n🔓 定高關閉，回手動模式（等待 Arduino 同步基準油門）")
    state['prev_h'] = curr_h

    # Tab / Shift / C / Z：初始按下 + 長按自動重複
    if state['alt_hold_active']:
        if kb_triggered('tab'):
            state['target_alt'] = min(ALT_MAX_CM, state['target_alt'] + state['alt_step'])
            print(f"\n📡 目標高度 → {state['target_alt']} cm")
        if kb_triggered('shift'):
            state['target_alt'] = max(5, state['target_alt'] - state['alt_step'])
            print(f"\n📡 目標高度 → {state['target_alt']} cm")
    else:
        if kb_triggered('tab'):
            state['base_throttle'] = min(255, state['base_throttle'] + state['throttle_step'])
        if kb_triggered('shift'):
            state['base_throttle'] = max(0, state['base_throttle'] - state['throttle_step'])
        if kb_triggered('c'):
            state['throttle_step'] = min(20, state['throttle_step'] + 1)
        if kb_triggered('z'):
            state['throttle_step'] = max(1, state['throttle_step'] - 1)

    raw_throttle = max(-1.0, min(1.0, (-1.0 if kb.is_pressed('w') else 0.0) + (1.0 if kb.is_pressed('s') else 0.0)))
    raw_yaw      = max(-1.0, min(1.0, (-1.0 if kb.is_pressed('a') else 0.0) + (1.0 if kb.is_pressed('d') else 0.0)))
    raw_pitch    = max(-1.0, min(1.0, (-1.0 if kb.is_pressed('up')    else 0.0) + (1.0 if kb.is_pressed('down')  else 0.0)))
    raw_roll     = max(-1.0, min(1.0, (-1.0 if kb.is_pressed('left')  else 0.0) + (1.0 if kb.is_pressed('right') else 0.0)))

    if state['alt_hold_active']:
        final_throttle = state['base_throttle']
    else:
        final_throttle = max(0, min(255,
            state['base_throttle'] + int(-raw_throttle * JOYSTICK_SENSITIVITY)))

    curr_r = kb.is_pressed('r')
    if curr_r and not state['prev_r']:
        if state['arm_state'] == 0 and final_throttle <= 5:
            state['arm_state'] = 255
        else:
            state['arm_state'] = 0
    state['prev_r'] = curr_r

    if kb.is_pressed('q'): state['gripper_val'] = max(0,   state['gripper_val'] - STEP_SPEED)
    if kb.is_pressed('e'): state['gripper_val'] = min(255, state['gripper_val'] + STEP_SPEED)

    target_alt_byte = int(state['target_alt']) if state['alt_hold_active'] else 0
    ah_val          = 1 if state['alt_hold_active'] else 0

    channels = {
        'throttle':   final_throttle,
        'target_alt': target_alt_byte,
        'yaw':   int((max(-1.0, min(1.0,  raw_yaw   * YAW_SENS )) + 1.0) / 2.0 * 255),
        'pitch': int((max(-1.0, min(1.0, -raw_pitch  * TILT_SENS)) + 1.0) / 2.0 * 255),
        'roll':  int((max(-1.0, min(1.0,  raw_roll   * TILT_SENS)) + 1.0) / 2.0 * 255),
        'ah_val': ah_val,
    }
    return channels, should_exit

# ==========================================
# pygame 視窗狀態顯示
# ==========================================
def draw_status(screen, font, state, mode, channels, connected):
    screen.fill((20, 20, 20))
    conn_color = (80, 220, 80)  if connected              else (220, 80, 80)
    ah_color   = (80, 180, 255) if state['alt_hold_active'] else (160, 160, 160)

    with _alt_lock:
        cur_alt = _current_alt
    alt_str = f"{cur_alt:.1f} cm" if cur_alt >= 0 else "---"

    if state['alt_hold_active']:
        if _pid_throttle >= 0:
            pid_conv = int((_pid_throttle - 1000) / 1000.0 * 255)
            pid_str = f" PID油門: {pid_conv}"
        else:
            pid_str = ""
        throttle_str = f"目標高度: {state['target_alt']:3d} cm (步進:{state['alt_step']:2d}cm){pid_str}"
    else:
        throttle_str = f"油門: {channels['throttle']:3d} 基準: {state['base_throttle']:3d}(步進:{state['throttle_step']:2d})"

    lines = [
        (f"模式: {'手把' if mode == 'gamepad' else '鍵盤'} "
         f"連線: {'連線中' if connected else '斷線'} "
         f"狀態: {'解鎖' if state['arm_state'] == 255 else '上鎖'}",
         conn_color),
        (f"定高: {'開啟' if state['alt_hold_active'] else '關閉'} " + throttle_str, ah_color),
        (f"當前高度: {alt_str}", (255, 220, 60)),
        (f"夾爪: {state['gripper_val']:3d} (L1/R1) Y: {channels['yaw']:3d} "
         f"P: {channels['pitch']:3d} R: {channels['roll']:3d} COM: {COM_PORT}",
         (180, 180, 180)),
        ("○/H=定高切換 定高:上下=大步 左右=±3cm 手動:上下=油門 Options/X長按3秒=退出 4+6=緊急停機",
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
joystick = None
try:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    mode = 'gamepad'
    print(f"✅ 手把已連線：{joystick.get_name()} → 手把模式")
except Exception:
    mode = 'keyboard'
    print("ℹ️ 未偵測到手把 → 鍵盤模式")

screen = pygame.display.set_mode((600, 158))
pygame.display.set_caption(
    f"無人機控制站（定高版）[{COM_PORT}] — {'手把' if mode == 'gamepad' else '鍵盤'}模式")
font  = pygame.font.SysFont("Microsoft JhengHei", 18)
clock = pygame.time.Clock()

print(f"\n嘗試連線到 {COM_PORT}...")
bt_serial = wait_for_connection(mode, joystick, timeout=60)
if bt_serial is None:
    pygame.quit()
    sys.exit()

state = {
    'base_throttle':  0,
    'throttle_step':  5,
    'gripper_val':    127,
    'arm_state':      0,
    'is_exiting':     False,
    'exit_press_time': 0,
    # 定高
    'alt_hold_active': False,
    'target_alt':      50,
    'alt_step':        ALT_STEP_DEFAULT,
    # 手把
    'offset':     (0.0, 0.0, 0.0, 0.0),
    'prev_sq':    0, 'prev_tri': 0, 'prev_circle': 0,
    'prev_up':    0, 'prev_down': 0, 'prev_left': 0, 'prev_right': 0,
    # 鍵盤（toggle 類仍需 prev；Tab/Shift/C/Z 改用 KEYDOWN，不需要 prev）
    'prev_r':     False, 'prev_h': False,
}

channels = {'throttle': 0, 'target_alt': 0, 'yaw': 127, 'pitch': 127, 'roll': 127, 'ah_val': 0}

reconnect_cooldown = 0
last_print_time    = 0

_serial_lock  = threading.Lock()
_reconnecting = False

_alt_lock    = threading.Lock()
_state_lock  = threading.Lock()   # 保護背景執行緒修改 state
_current_alt = -1
_alt_history = deque(maxlen=10)
_pid_throttle = -1  # Arduino 定高 PID 實際油門（IBUS 1000~2000），-1 表示尚無資料


def get_alt_snapshot():
    with _alt_lock:
        if not _alt_history:
            return _current_alt
        return round(sum(_alt_history) / len(_alt_history), 1)

def _serial_reader():
    global _current_alt, _pid_throttle
    while True:
        with _serial_lock:
            ser = bt_serial
        if ser is None or not ser.is_open:
            time.sleep(0.1)
            continue
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()

            if line.startswith('D:'):
                # 高度回報
                mm  = int(line[2:])
                val = round(mm / 10.0, 1)
                with _alt_lock:
                    _current_alt = val
                    _alt_history.append(val)
                pid_logger.record(val, state['target_alt'], _pid_throttle)

            elif line.startswith('P:'):
                # 定高中 PID 實際輸出油門（IBUS 1000~2000）
                _pid_throttle = int(line[2:])

            elif line.startswith('T:'):
                # ── 定高結束，Arduino 回傳最後 PID 油門（IBUS 值 1000~2000） ──
                # 換算回 base_throttle（0~255）並更新，作為手動模式新基準
                ibus_val = int(line[2:])
                new_base = int((ibus_val - 1000) / 1000.0 * 255)
                new_base = max(0, min(255, new_base))
                with _state_lock:
                    state['base_throttle'] = new_base
                print(f"\n✅ 基準油門已同步：{new_base}（IBUS {ibus_val}）")

        except (ValueError, UnicodeDecodeError):
            pass
        except Exception:
            time.sleep(0.05)

def _do_reconnect():
    global bt_serial, _reconnecting, reconnect_cooldown
    new_ser = connect_serial()
    with _serial_lock:
        bt_serial = new_ser
    _reconnecting      = False
    reconnect_cooldown = time.time() + 3.0

threading.Thread(target=_serial_reader, daemon=True).start()

if mode == 'gamepad':
    print("\n🎮 手把模式已上線！（定高版）")
    print("△=解鎖/上鎖 ○=定高切換 □=搖桿校準")
    print("定高開啟時：D-pad上下=目標高度±大步 D-pad左右=目標高度±3cm")
    print("手動模式時：D-pad上下=基準油門± Options長按3秒=退出 4+6=緊急停機\n")
else:
    print("\n⌨️ 鍵盤模式已上線！（定高版，全域輸入，不需點選視窗）")
    print("W/S=油門 A/D=偏航 ↑↓=俯仰 ←→=翻滾 R=解鎖/上鎖 H=定高切換")
    print("定高開啟時：Tab=高度↑ Shift=高度↓")
    print("手動模式時：Tab=油門↑ Shift=油門↓ C/Z=步進± X長按3秒=退出\n")

try:
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt

        if mode == 'gamepad':
            try:
                channels, should_exit, estop = read_gamepad(joystick, state)
            except pygame.error as e:
                print(f"\n⚠️ 手把讀取失敗（{e}），安全退出...")
                should_exit, estop = True, False
                channels = {'throttle': 0, 'target_alt': 0,
                            'yaw': 127, 'pitch': 127, 'roll': 127, 'ah_val': 0}
            if estop:
                print("\n🚨 [緊急停機] 觸發！立即送出停機指令...")
                if bt_serial and bt_serial.is_open:
                    estop_pkt = make_packet(0, 127, 127, 127, 0, state['gripper_val'], 0, 0)
                    for _ in range(5):
                        send_packet(bt_serial, estop_pkt)
                raise KeyboardInterrupt
        else:
            channels, should_exit = read_keyboard(state)

        if should_exit:
            break

        # base_throttle 可能被背景執行緒更新，讀取時加鎖
        with _state_lock:
            current_base = state['base_throttle']

        data_packet = make_packet(
            channels['throttle'],
            channels['yaw'],
            channels['pitch'],
            channels['roll'],
            channels['target_alt'],
            state['gripper_val'],
            state['arm_state'],
            channels['ah_val'],
        )

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
                print("\n⚠️ 串口寫入失敗，藍牙可能已斷線")
                safe_disconnect(_bt, state['gripper_val'])
                with _serial_lock:
                    bt_serial = None
                state['arm_state']      = 0
                state['alt_hold_active'] = False
                reconnect_cooldown = time.time() + 3.0

        now = time.time()
        if not state['is_exiting'] and now - last_print_time >= 0.2:
            conn_str = "連線中" if connected else "斷線中"
            arm_str  = "解鎖"   if state['arm_state'] == 255 else "上鎖"
            if state['alt_hold_active']:
                if _pid_throttle >= 0:
                    pid_conv = int((_pid_throttle - 1000) / 1000.0 * 255)
                    pid_disp = f" PID:{pid_conv:3d}"
                else:
                    pid_disp = ""
                ah_str = f"定高:{state['target_alt']}cm{pid_disp}"
            else:
                ah_str = f"手動:{channels['throttle']:3d}"
            with _alt_lock:
                cur = _current_alt
            cur_str = f"{cur:.1f}cm" if cur >= 0 else " ---"
            print(f"[{conn_str}] {arm_str} | {ah_str} | 當前:{cur_str} | "
                  f"基準:{current_base:3d} | 夾爪:{state['gripper_val']:3d} | "
                  f"Y:{channels['yaw']:3d} P:{channels['pitch']:3d} R:{channels['roll']:3d}",
                  end="\r", flush=True)
            last_print_time = now

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
    pid_logger.stop()
    print("\n🔌 正在關閉連線...")
    if bt_serial is not None and bt_serial.is_open:
        safe_disconnect(bt_serial)
    pygame.quit()
    print("✅ 已安全關閉。")