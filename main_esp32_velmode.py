import os
os.environ['SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS'] = '1'

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
    pid_logger.configure(log_dir='vel_logs', prefix='velmode')
except ImportError:
    class pid_logger:
        configure = staticmethod(lambda *_: None)
        start     = staticmethod(lambda *_: None)
        stop      = staticmethod(lambda *_: None)
        record    = staticmethod(lambda *_: None)

try:
    import flow_logger
    flow_logger.configure(log_dir='flow_logs', prefix='flow')
except ImportError:
    class flow_logger:
        configure = staticmethod(lambda *_: None)
        start     = staticmethod(lambda *_: None)
        stop      = staticmethod(lambda *_: None)
        record    = staticmethod(lambda *_: None)

__version__ = "0.5.0 (Velocity Mode)"

# ==========================================
# 啟動參數
# ==========================================
parser = argparse.ArgumentParser(description='無人機地面控制站（速度模式定高）')
parser.add_argument('--port', default='COM5', help='藍牙 COM 埠（預設: COM5）')
args = parser.parse_args()

COM_PORT  = args.port
BAUD_RATE = 9600

JOYSTICK_SENSITIVITY = 60
TILT_SENS  = 0.3
YAW_SENS   = 0.25
STEP_SPEED = 5
DEAD_ZONE  = 0.05

THROTTLE_EXPO = 0.75
TILT_EXPO     = 0.50
YAW_EXPO      = 0.50

ALT_MAX_CM    = 300
ALT_VEL_SCALE = 60   # cm/s，搖桿全推對應的最大速率

# ── 手把按鍵 ──
BTN_CIRCLE = 1
BTN_CROSS  = 0
BTN_SQUARE = 2
BTN_TRI    = 3
BTN_L1     = 9
BTN_R1     = 10
BTN_UP, BTN_DOWN, BTN_LEFT, BTN_RIGHT = 11, 12, 13, 14
BTN_OPTIONS = 6
BTN_ESTOP_A = 4
BTN_ESTOP_B = 6
BTN_CALIB   = 15   # 右搖桿偏移補正（原 X 鍵，改到 15 防誤觸）

# ==========================================
# 藍牙工具
# ==========================================
def connect_serial():
    try:
        s = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.05)
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
    shutdown = make_packet(0, 127, 127, 127, 128, gripper_val, 0, 0)
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

def do_emergency_lock(ser, state_dict):
    """所有緊急路徑共用：更新 Python 狀態、停止記錄、送 5 次上鎖封包。"""
    state_dict['arm_state']       = 0
    state_dict['alt_hold_active'] = False
    pid_logger.stop()
    flow_logger.stop()
    if ser is not None and ser.is_open:
        lock_pkt = make_packet(0, 127, 127, 127, 128, state_dict['gripper_val'], 0, 0)
        for _ in range(5):
            try:
                ser.write(lock_pkt)
                time.sleep(0.01)
            except Exception:
                pass

def apply_dead_zone(value):
    return 0.0 if abs(value) < DEAD_ZONE else value

def apply_expo(val, expo_factor):
    return (1.0 - expo_factor) * val + expo_factor * (val ** 3)

# ── vel_cmd 編碼：128 = 0 cm/s，每 1 unit = 1 cm/s ──
def encode_vel(vel_cm_s):
    return max(0, min(255, int(round(vel_cm_s)) + 128))

_key_held_start  = {}
_key_last_repeat = {}
_btn_held_start  = {}
_btn_last_repeat = {}

def btn_triggered(name, pressed, initial=0.30, repeat=0.10):
    if not pressed:
        _btn_held_start.pop(name, None)
        _btn_last_repeat.pop(name, None)
        return False
    now = time.time()
    if name not in _btn_held_start:
        _btn_held_start[name] = now
        _btn_last_repeat[name] = now
        return True
    if now - _btn_held_start[name] >= initial:
        if now - _btn_last_repeat[name] >= repeat:
            _btn_last_repeat[name] = now
            return True
    return False

def kb_triggered(key, initial=0.25, repeat=0.10):
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

    while time.time() < deadline:
        pygame.event.pump()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return None

        if mode == 'gamepad' and joystick:
            try:
                if joystick.get_button(BTN_ESTOP_A) and joystick.get_button(BTN_ESTOP_B):
                    return None
                held = joystick.get_button(BTN_OPTIONS)
            except pygame.error:
                return None
        else:
            keys = pygame.key.get_pressed()
            held = keys[pygame.K_x]

        if held:
            if hold_start is None:
                hold_start = time.time()
            elif time.time() - hold_start >= 3.0:
                return None
        else:
            hold_start = None

        if time.time() >= next_retry:
            ser = connect_serial()
            if ser is not None:
                return ser
            next_retry = time.time() + 3.0

        time.sleep(0.05)

    return None

# ==========================================
# 輸入讀取
# ==========================================
def read_gamepad(joystick, state):
    if joystick.get_button(BTN_ESTOP_A) and joystick.get_button(BTN_ESTOP_B):
        return None, False, True

    should_exit = False
    if joystick.get_button(BTN_OPTIONS):
        if not state['is_exiting']:
            state['is_exiting']      = True
            state['exit_press_time'] = time.time()
            print("\n⚠️ 繼續按住 Options 3 秒退出...")
        elif time.time() - state['exit_press_time'] >= 3.0:
            should_exit = True
    else:
        if state['is_exiting']:
            state['is_exiting'] = False
            print("\n✅ 退出指令取消。")

    # ○：切換定高
    curr_circle = joystick.get_button(BTN_CIRCLE)
    if curr_circle and not state['prev_circle']:
        if not state['alt_hold_active']:
            if state['arm_state'] != 255:
                print("\n⚠️ 未解鎖，無法啟動定高。")
            else:
                snap = get_alt_snapshot()
                if snap >= 0:
                    state['alt_hold_active'] = True
                    pid_logger.start(snap)
                    print(f"\n🔒 速度模式啟動！當前高度：{snap:.1f} cm")
                    print(" 左搖桿死區=懸停(vel=0)，推上/下=爬升/下降速率")
                else:
                    print("\n⚠️ 感測器尚無資料，無法切入。")
        else:
            state['alt_hold_active'] = False
            pid_logger.stop()
            state['syncing_throttle']   = True
            state['syncing_throttle_t'] = time.time()
            print("\n🔓 定高關閉，等待油門同步...")
    state['prev_circle'] = curr_circle

    # □：定高時重設積分（reref）；手動時校準搖桿
    curr_sq = joystick.get_button(BTN_SQUARE)
    if curr_sq and not state['prev_sq']:
        if state['alt_hold_active']:
            state['reref_pending'] = True
            print("\n🔄 積分重設中...")
        else:
            state['offset'] = (
                joystick.get_axis(1), joystick.get_axis(0),
                joystick.get_axis(3), joystick.get_axis(2),
            )
            print("\n⚙️ 搖桿校準完成。")
    state['prev_sq'] = curr_sq

    # 15：右搖桿偏移補正 trim（搖桿推到補正位置後按，置中後補正量自動生效）
    curr_calib = joystick.get_button(BTN_CALIB)
    if curr_calib and not state['prev_calib']:
        oy, ox, op, or_ = state['offset']
        current_stick_pitch = apply_expo(apply_dead_zone(joystick.get_axis(3) - op), TILT_EXPO)
        current_stick_roll  = apply_expo(apply_dead_zone(joystick.get_axis(2) - or_), TILT_EXPO)
        old_trim_pitch, old_trim_roll = state['right_stick_trim']
        new_trim_pitch = max(-1.0, min(1.0, old_trim_pitch + current_stick_pitch))
        new_trim_roll  = max(-1.0, min(1.0, old_trim_roll  + current_stick_roll))
        state['right_stick_trim'] = (new_trim_pitch, new_trim_roll)
        state['right_stick_recenter_required'] = True
        print(f"[Trim] P:{new_trim_pitch:+.3f} R:{new_trim_roll:+.3f} 已記錄並凍結，請將搖桿置中")
    state['prev_calib'] = curr_calib

    # 方向鍵：手動模式下調整基準油門
    curr_up    = joystick.get_button(BTN_UP)
    curr_down  = joystick.get_button(BTN_DOWN)
    curr_left  = joystick.get_button(BTN_LEFT)
    curr_right = joystick.get_button(BTN_RIGHT)

    if not state['alt_hold_active']:
        if btn_triggered('dpad_up',    curr_up):
            state['base_throttle'] = min(255, state['base_throttle'] + state['throttle_step'])
        if btn_triggered('dpad_down',  curr_down):
            state['base_throttle'] = max(0,   state['base_throttle'] - state['throttle_step'])
        if btn_triggered('dpad_right', curr_right):
            state['throttle_step'] = min(20, state['throttle_step'] + 1)
        if btn_triggered('dpad_left',  curr_left):
            state['throttle_step'] = max(1,  state['throttle_step'] - 1)

    state['prev_up'],   state['prev_down']  = curr_up,   curr_down
    state['prev_left'], state['prev_right'] = curr_left, curr_right

    oy, ox, op, or_ = state['offset']
    # 速度模式下左搖桿直接用原始軸值，不套 oy，確保物理中心 = 0 速率
    _thr_raw = joystick.get_axis(1) if state['alt_hold_active'] else joystick.get_axis(1) - oy
    raw_throttle = apply_expo(apply_dead_zone(_thr_raw), THROTTLE_EXPO)
    raw_yaw      = apply_expo(apply_dead_zone(joystick.get_axis(0) - ox), YAW_EXPO)
    raw_pitch    = apply_expo(apply_dead_zone(joystick.get_axis(3) - op), TILT_EXPO)
    raw_roll     = apply_expo(apply_dead_zone(joystick.get_axis(2) - or_), TILT_EXPO)

    if state['right_stick_recenter_required']:
        if abs(joystick.get_axis(3)) < 0.12 and abs(joystick.get_axis(2)) < 0.12:
            state['right_stick_recenter_required'] = False
            print("[Trim] 搖桿已回中，補正生效")
        else:
            raw_pitch = 0.0
            raw_roll  = 0.0

    trim_pitch, trim_roll = state['right_stick_trim']
    raw_pitch = max(-1.0, min(1.0, raw_pitch + trim_pitch))
    raw_roll  = max(-1.0, min(1.0, raw_roll  + trim_roll))

    # 解鎖序列：arm_pending 期間強制送 0 油門，時間到再真正解鎖並跳到 60
    if state['arm_pending'] and not state['alt_hold_active']:
        if time.time() - state['arm_pending_t'] >= 0.25:
            state['arm_pending']    = False
            state['arm_state']      = 255
            state['base_throttle']  = 60
            flow_logger.start()
            print("\n✅ 解鎖！")

    if state['alt_hold_active']:
        raw_throttle_vel = apply_dead_zone(_thr_raw)
        vel_cmd = -raw_throttle_vel * ALT_VEL_SCALE
        alt_byte = encode_vel(vel_cmd)
        final_throttle = state['base_throttle']
    else:
        vel_cmd    = 0
        alt_byte   = 128
        if state['arm_pending']:
            final_throttle = 0
        elif state['syncing_throttle']:
            if time.time() - state['syncing_throttle_t'] > 5.0:
                state['syncing_throttle'] = False
                print("\n⚠️ 油門同步逾時，已自動解除凍結。")
            final_throttle = state['base_throttle']
        else:
            final_throttle = max(0, min(255,
                state['base_throttle'] + int(round(-raw_throttle * JOYSTICK_SENSITIVITY))))

    curr_tri = joystick.get_button(BTN_TRI)
    if curr_tri and not state['prev_tri']:
        if state['arm_state'] == 0 and not state['arm_pending']:
            if state['alt_hold_active']:
                state['arm_state'] = 255
                snap = get_alt_snapshot()
                if snap >= 0:
                    pid_logger.start(snap)
                flow_logger.start()
            else:
                state['arm_pending']   = True
                state['arm_pending_t'] = time.time()
                state['base_throttle'] = 0
                print("\n⏳ 解鎖中...")
        else:
            if state['alt_hold_active']:
                pid_logger.stop()
            flow_logger.stop()
            state['arm_pending']   = False
            state['arm_state']     = 0
            state['base_throttle'] = 90
    state['prev_tri'] = curr_tri

    if joystick.get_button(BTN_L1): state['gripper_val'] = max(0,   state['gripper_val'] - STEP_SPEED)
    if joystick.get_button(BTN_R1): state['gripper_val'] = min(255, state['gripper_val'] + STEP_SPEED)

    ah_val = 1 if state['alt_hold_active'] else 0
    channels = {
        'throttle':   final_throttle,
        'alt_byte':   alt_byte,
        'vel_cmd':    vel_cmd,
        'yaw':   int((max(-1.0, min(1.0,  raw_yaw   * YAW_SENS )) + 1.0) / 2.0 * 255),
        'pitch': int((max(-1.0, min(1.0, -raw_pitch  * TILT_SENS)) + 1.0) / 2.0 * 255),
        'roll':  int((max(-1.0, min(1.0,  raw_roll   * TILT_SENS)) + 1.0) / 2.0 * 255),
        'ah_val': ah_val,
    }
    return channels, should_exit, False

def read_keyboard(state):
    should_exit = False
    if kb.is_pressed('x'):
        if not state['is_exiting']:
            state['is_exiting']      = True
            state['exit_press_time'] = time.time()
            print("\n⚠️ 繼續按住 X 鍵 3 秒退出...")
        elif time.time() - state['exit_press_time'] >= 3.0:
            should_exit = True
    else:
        if state['is_exiting']:
            state['is_exiting'] = False
            print("\n✅ 退出指令取消。")

    # H：切換定高
    curr_h = kb.is_pressed('h')
    if curr_h and not state['prev_h']:
        if not state['alt_hold_active']:
            if state['arm_state'] != 255:
                print("\n⚠️ 未解鎖，無法啟動定高。")
            else:
                snap = get_alt_snapshot()
                if snap >= 0:
                    state['alt_hold_active'] = True
                    state['last_alt_update_t'] = time.time()
                    pid_logger.start(snap)
                    print(f"\n🔒 速度模式定高啟動！當前高度：{snap:.1f} cm")
                else:
                    print("\n⚠️ 感測器尚無資料，無法切入定高。")
        else:
            state['alt_hold_active'] = False
            pid_logger.stop()
            state['syncing_throttle']   = True
            state['syncing_throttle_t'] = time.time()
            print("\n🔓 定高關閉，等待油門同步...")
    state['prev_h'] = curr_h

    # F：重設積分
    curr_f = kb.is_pressed('f')
    if curr_f and not state['prev_f'] and state['alt_hold_active']:
        state['reref_pending'] = True
        print("\n🔄 積分重設中...")
    state['prev_f'] = curr_f

    if not state['alt_hold_active']:
        if kb_triggered('tab'):   state['base_throttle'] = min(255, state['base_throttle'] + state['throttle_step'])
        if kb_triggered('shift'): state['base_throttle'] = max(0,   state['base_throttle'] - state['throttle_step'])
        if kb_triggered('c'):     state['throttle_step'] = min(20, state['throttle_step'] + 1)
        if kb_triggered('z'):     state['throttle_step'] = max(1,  state['throttle_step'] - 1)

    raw_throttle = apply_expo(max(-1.0, min(1.0, (-1.0 if kb.is_pressed('w') else 0.0) + (1.0 if kb.is_pressed('s') else 0.0))), THROTTLE_EXPO)
    raw_yaw      = apply_expo(max(-1.0, min(1.0, (-1.0 if kb.is_pressed('a') else 0.0) + (1.0 if kb.is_pressed('d') else 0.0))), YAW_EXPO)
    raw_pitch    = apply_expo(max(-1.0, min(1.0, (-1.0 if kb.is_pressed('up') else 0.0) + (1.0 if kb.is_pressed('down') else 0.0))), TILT_EXPO)
    raw_roll     = apply_expo(max(-1.0, min(1.0, (-1.0 if kb.is_pressed('left') else 0.0) + (1.0 if kb.is_pressed('right') else 0.0))), TILT_EXPO)

    # 解鎖序列
    if state['arm_pending'] and not state['alt_hold_active']:
        if time.time() - state['arm_pending_t'] >= 0.25:
            state['arm_pending']    = False
            state['arm_state']      = 255
            state['base_throttle']  = 60
            print("\n✅ 解鎖！")

    if state['alt_hold_active']:
        raw_throttle_vel = apply_dead_zone(max(-1.0, min(1.0,
            (-1.0 if kb.is_pressed('w') else 0.0) + (1.0 if kb.is_pressed('s') else 0.0))))
        vel_cmd    = -raw_throttle_vel * ALT_VEL_SCALE
        alt_byte   = encode_vel(vel_cmd)
        final_throttle = state['base_throttle']
    else:
        vel_cmd    = 0
        alt_byte   = 128
        if state['arm_pending']:
            final_throttle = 0
        elif state['syncing_throttle']:
            if time.time() - state['syncing_throttle_t'] > 5.0:
                state['syncing_throttle'] = False
                print("\n⚠️ 油門同步逾時，已自動解除凍結。")
            final_throttle = state['base_throttle']
        else:
            final_throttle = max(0, min(255,
                state['base_throttle'] + int(round(-raw_throttle * JOYSTICK_SENSITIVITY))))

    curr_r = kb.is_pressed('r')
    if curr_r and not state['prev_r']:
        if state['arm_state'] == 0 and not state['arm_pending']:
            if state['alt_hold_active']:
                state['arm_state'] = 255
                snap = get_alt_snapshot()
                if snap >= 0:
                    pid_logger.start(snap)
                flow_logger.start()
            else:
                state['arm_pending']   = True
                state['arm_pending_t'] = time.time()
                state['base_throttle'] = 0
                print("\n⏳ 解鎖中...")
        else:
            if state['alt_hold_active']:
                pid_logger.stop()
            flow_logger.stop()
            state['arm_pending']   = False
            state['arm_state']     = 0
            state['base_throttle'] = 90
    state['prev_r'] = curr_r

    if kb.is_pressed('q'): state['gripper_val'] = max(0,   state['gripper_val'] - STEP_SPEED)
    if kb.is_pressed('e'): state['gripper_val'] = min(255, state['gripper_val'] + STEP_SPEED)

    ah_val = 1 if state['alt_hold_active'] else 0
    channels = {
        'throttle':   final_throttle,
        'alt_byte':   alt_byte,
        'vel_cmd':    vel_cmd,
        'yaw':   int((max(-1.0, min(1.0,  raw_yaw   * YAW_SENS )) + 1.0) / 2.0 * 255),
        'pitch': int((max(-1.0, min(1.0, -raw_pitch  * TILT_SENS)) + 1.0) / 2.0 * 255),
        'roll':  int((max(-1.0, min(1.0,  raw_roll   * TILT_SENS)) + 1.0) / 2.0 * 255),
        'ah_val': ah_val,
    }
    return channels, should_exit

# ==========================================
# pygame 狀態顯示
# ==========================================
def draw_status(screen, font, state, mode, channels, connected):
    screen.fill((20, 20, 20))
    conn_color = (80, 220, 80)  if connected               else (220, 80, 80)
    ah_color   = (80, 180, 255) if state['alt_hold_active'] else (160, 160, 160)

    with _alt_lock:
        cur_alt = _current_alt

    alt_str = f"{cur_alt:.1f} cm" if cur_alt >= 0 else "---"

    if state['alt_hold_active']:
        vel = channels.get('vel_cmd', 0)
        vel_str = f"速度命令: {vel:+.1f} cm/s"
        if _pid_throttle >= 0:
            pid_conv = int((_pid_throttle - 1000) / 1000.0 * 255)
            vel_str += f"  PID油門: {pid_conv}"
        throttle_str = vel_str
    else:
        throttle_str = f"油門: {channels['throttle']:3d}  基準: {state['base_throttle']:3d} (步進:{state['throttle_step']:2d})"

    lines = [
        (f"模式: {'手把' if mode == 'gamepad' else '鍵盤'}  "
         f"連線: {'連線中' if connected else '斷線'}  "
         f"狀態: {'解鎖' if state['arm_state'] == 255 else '上鎖'}",
         conn_color),
        (f"定高[速度模式]: {'開啟' if state['alt_hold_active'] else '關閉'}  " + throttle_str, ah_color),
        (f"當前高度: {alt_str}", (255, 220, 60)),
        (f"夾爪: {state['gripper_val']:3d}  Y: {channels['yaw']:3d}  "
         f"P: {channels['pitch']:3d}  R: {channels['roll']:3d}  COM: {COM_PORT}",
         (180, 180, 180)),
        ("○/H=定高切換  定高:搖桿=速度  死區=懸停  □/F=積分重設  Options/X長按3秒=退出  4+6=緊急停機",
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
pygame.key.set_repeat(250, 100)
joystick = None
try:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    mode = 'gamepad'
    print(f"✅ 手把已連線：{joystick.get_name()} → 手把模式")
except Exception:
    mode = 'keyboard'
    print("ℹ️ 未偵測到手把 → 鍵盤模式")

screen = pygame.display.set_mode((620, 158))
pygame.display.set_caption(
    f"無人機控制站（速度模式定高）[{COM_PORT}] — {'手把' if mode == 'gamepad' else '鍵盤'}模式")
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
    'alt_hold_active':  False,
    'last_alt_update_t': 0.0,
    'offset':     (0.0, 0.0, 0.0, 0.0),
    'right_stick_trim': (0.0, 0.0),
    'right_stick_recenter_required': False,
    'prev_sq':    0, 'prev_tri': 0, 'prev_circle': 0, 'prev_calib': 0,
    'prev_up':    0, 'prev_down': 0, 'prev_left': 0, 'prev_right': 0,
    'reref_pending': False,
    'prev_r': False, 'prev_h': False, 'prev_f': False,
    'syncing_throttle': False,  # 退出定高後等 T: 同步完成前凍結手動油門
    'syncing_throttle_t': 0.0,  # syncing 開始時間，逾時自動解除
    'arm_pending': False,        # 解鎖序列：先送 0 油門讓 Betaflight 接受，再跳 60
    'arm_pending_t': 0.0,
}

channels = {'throttle': 0, 'alt_byte': 128, 'vel_cmd': 0,
            'yaw': 127, 'pitch': 127, 'roll': 127, 'ah_val': 0}

reconnect_cooldown = 0
last_print_time    = 0

_serial_lock  = threading.Lock()
_reconnecting = False

_alt_lock    = threading.Lock()
_state_lock  = threading.Lock()
_current_alt = -1.0
_alt_history = deque(maxlen=10)
_pid_throttle = -1

_alt_last_accepted = -1.0
_alt_last_accept_t =  0.0

def get_alt_snapshot():
    with _alt_lock:
        if not _alt_history:
            return _current_alt
        return round(sum(_alt_history) / len(_alt_history), 1)

def _handle_bt_line(line):
    """處理一行 ESP32 遙測文字（不含換行）。"""
    global _current_alt, _pid_throttle
    try:
        if line.startswith('D:'):
            val = round(int(line[2:]) / 10.0, 1)
            with _alt_lock:
                _current_alt = val
                _alt_history.append(val)
            pid_logger.record(val, 0, _pid_throttle, channels.get('vel_cmd', 0))
        elif line.startswith('OF:'):
            parts = line[3:].split(',')
            if len(parts) == 2:
                dx, dy = int(parts[0]), int(parts[1])
                with _alt_lock:
                    cur_alt_mm = _current_alt * 10.0
                flow_logger.record(dx, dy, cur_alt_mm)
        elif line.startswith('P:'):
            _pid_throttle = int(line[2:])
        elif line.startswith('T:'):
            new_base = int((int(line[2:]) - 1000) / 1000.0 * 255)
            new_base = max(0, min(255, new_base))
            with _state_lock:
                state['base_throttle']    = new_base
                state['syncing_throttle'] = False
            print(f"\n✅ 基準油門已同步：{new_base}")
        elif line.startswith('F:2'):
            with _state_lock:
                if state['alt_hold_active']:
                    state['alt_hold_active']    = False
                    state['syncing_throttle']   = True
                    state['syncing_throttle_t'] = time.time()
            pid_logger.stop()
            print("\n⚠️ 感測器逾時，定高已被 ESP32 強制關閉！")
    except (ValueError, UnicodeDecodeError):
        pass

def _serial_reader():
    # 用 in_waiting 輪詢 + 手動 buffer 累積到換行才處理。
    # Windows Bluetooth 虛擬 COM 上 readline()+timeout 常漏接，
    # 這個方式和 test_esp32_bt.py 相同，確保 D:/T:/P:/F: 都收得到。
    global _alt_last_accepted, _alt_last_accept_t
    buf = b''
    while True:
        with _serial_lock:
            ser = bt_serial
        if ser is None or not ser.is_open:
            buf = b''
            time.sleep(0.1)
            continue
        try:
            n = ser.in_waiting
            if n:
                buf += ser.read(n)
                while b'\n' in buf:
                    raw, buf = buf.split(b'\n', 1)
                    line = raw.decode('utf-8', errors='ignore').strip()
                    if line:
                        _handle_bt_line(line)
            else:
                time.sleep(0.01)
        except Exception:
            buf = b''
            time.sleep(0.05)

def _do_reconnect():
    global bt_serial, _reconnecting, reconnect_cooldown
    new_ser = connect_serial()
    with _serial_lock:
        bt_serial = new_ser
    state['arm_state']       = 0
    state['alt_hold_active'] = False
    _reconnecting      = False
    reconnect_cooldown = time.time() + 3.0

threading.Thread(target=_serial_reader, daemon=True).start()

if mode == 'gamepad':
    print("\n🎮 手把模式（速度模式定高）")
    print("△=解鎖/上鎖  ○=定高切換  □=積分重設  15=右搖桿偏移補正")
    print("定高中：左搖桿死區=懸停，推上/下=爬升/下降速率")
    print("手動：D-pad上下=基準油門±  Options長按3秒=退出  4+6=緊急停機\n")
else:
    print("\n⌨️ 鍵盤模式（速度模式定高）")
    print("W/S=油門  A/D=偏航  ↑↓=俯仰  ←→=翻滾  R=解鎖/上鎖  H=定高切換")
    print("定高中：W/S=爬升/下降速率，放開=懸停  F=積分重設")
    print("手動：Tab=油門↑  Shift=油門↓  C/Z=步進±  X長按3秒=退出\n")

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
                with _serial_lock:
                    _bt_pe = bt_serial
                do_emergency_lock(_bt_pe, state)
                should_exit = True
                channels = {'throttle': 0, 'alt_byte': 128, 'vel_cmd': 0,
                            'yaw': 127, 'pitch': 127, 'roll': 127, 'ah_val': 0}
            if estop:
                print("\n🚨 [緊急停機]！")
                with _serial_lock:
                    _bt_es = bt_serial
                do_emergency_lock(_bt_es, state)
                raise KeyboardInterrupt
        else:
            channels, should_exit = read_keyboard(state)

        if should_exit:
            with _serial_lock:
                _bt_exit = bt_serial
            do_emergency_lock(_bt_exit, state)
            break

        # 積分重設：送 ah_val=2
        if state['reref_pending'] and state['alt_hold_active']:
            state['reref_pending'] = False
            channels['ah_val'] = 2

        with _state_lock:
            current_base = state['base_throttle']

        data_packet = make_packet(
            channels['throttle'],
            channels['yaw'],
            channels['pitch'],
            channels['roll'],
            channels['alt_byte'],   # 速度命令（128=0, >128=上, <128=下）
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
                print("\n⚠️ 串口寫入失敗")
                safe_disconnect(_bt, state['gripper_val'])
                with _serial_lock:
                    bt_serial = None
                state['arm_state']       = 0
                state['alt_hold_active'] = False
                pid_logger.stop()
                reconnect_cooldown = time.time() + 3.0

        now = time.time()
        if not state['is_exiting'] and now - last_print_time >= 0.2:
            conn_str = "連線中" if connected else "斷線中"
            arm_str  = "解鎖"   if state['arm_state'] == 255 else "上鎖"
            if state['alt_hold_active']:
                vel = channels.get('vel_cmd', 0)
                ah_str = f"速度:{vel:+.1f}cm/s"
                if _pid_throttle >= 0:
                    pid_conv = int((_pid_throttle - 1000) / 1000.0 * 255)
                    ah_str += f" PID:{pid_conv:3d}"
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
    print("\n🛑 任務中止。")
except pygame.error as e:
    print(f"\n💥 pygame 錯誤：{e}")

finally:
    pid_logger.stop()
    flow_logger.stop()
    print("\n🔌 正在關閉連線...")
    if bt_serial is not None and bt_serial.is_open:
        safe_disconnect(bt_serial)
    pygame.quit()
    print("✅ 已安全關閉。")
