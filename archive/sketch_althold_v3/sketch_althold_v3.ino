// Version: 0.4.15.0
#include <Servo.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <VL53L0X.h>

// ─────────────────────────────────────────
// 硬體初始化
// ─────────────────────────────────────────
SoftwareSerial BTSerial(10, 11);
Servo gripper_servo;
VL53L0X altSensor;

// ─────────────────────────────────────────
// IBUS 通道陣列
// CH1(0): Roll  CH2(1): Pitch
// CH3(2): Throttle  CH4(3): Yaw
// CH5(4): ARM  CH6(5): Angle Mode (固定2000)
// ─────────────────────────────────────────
uint16_t ibus_channels[14] = {
  1500, 1500, 1000, 1500,
  1000, 2000,
  1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500
};
unsigned long last_ibus_time = 0;

// ─────────────────────────────────────────
// 定高 PID
// ─────────────────────────────────────────
bool  alt_hold_active = false;
bool  takeoff_phase   = false;   // 起飛爬升中（尚未離地）
float target_alt_cm   = 0;
float last_error      = 0;
float integral        = 0;
float d_filtered      = 0;
int   base_throttle   = 1500;
int   manual_throttle = 1500;
unsigned long last_pid_time    = 0;
unsigned long takeoff_start_ms = 0;

// ─────────────────────────────────────────
// 感測器
// ─────────────────────────────────────────
bool  sensor_ok        = false;
float filtered_mm      = -1;
float prev_filtered_mm = -1;
bool  filter_inited    = false;
unsigned long last_sensor_time = 0;
unsigned long last_alt_report  = 0;

// ─────────────────────────────────────────
// 係數
// ─────────────────────────────────────────
const float Kp    = 1.2;
const float Ki    = 0.05;
const float Kd    = 4.0;
const float ALPHA = 0.15;
const float BETA  = 0.2;
const int   TAKEOFF_LIFTOFF_CM  = 15;    // 超過此高度視為已離地
const int   TAKEOFF_MAX_IBUS    = 1780;  // 起飛爬升油門安全上限
const unsigned long TAKEOFF_TIMEOUT_MS = 8000;  // 8 秒內未離地則中止
const float MAX_ALT_CM          = 100.0f; // 超過此高度強制解除定高
const float MAX_CORRECTION      = 150.0f; // PID 單次修正量上限（IBUS）

// ─────────────────────────────────────────
// IBUS 發送
// ─────────────────────────────────────────
void sendIBUS() {
  uint8_t packet[32];
  packet[0] = 0x20;
  packet[1] = 0x40;
  uint16_t checksum = 0xFFFF - packet[0] - packet[1];
  for (int i = 0; i < 14; i++) {
    packet[2 + i*2] = ibus_channels[i] & 0xFF;
    packet[3 + i*2] = (ibus_channels[i] >> 8) & 0xFF;
    checksum -= packet[2 + i*2];
    checksum -= packet[3 + i*2];
  }
  packet[30] = checksum & 0xFF;
  packet[31] = (checksum >> 8) & 0xFF;
  Serial.write(packet, 32);
}

// ─────────────────────────────────────────
// 定高 PID 更新（50Hz）
// ─────────────────────────────────────────
void updateAltHold(float filt_mm) {
  unsigned long now = millis();
  float dt = (now - last_pid_time) / 1000.0f;
  if (dt < 0.02f) return;
  last_pid_time = now;

  if (filt_mm < 0) return;
  float current_cm = filt_mm / 10.0f;
  if (current_cm > 130) return;

  // 高度上限保護：用暫時目標計算 error，不永久修改 target_alt_cm
  float effective_target = min(target_alt_cm, MAX_ALT_CM);
  float error = effective_target - current_cm;

  // anti-windup：地面（< 8cm）或 correction 已飽和時不累積積分
  float correction_unsat = Kp * error + Ki * integral + Kd * d_filtered;
  bool saturated = (correction_unsat > MAX_CORRECTION && error > 0) ||
                   (correction_unsat < -MAX_CORRECTION && error < 0);
  if (current_cm > 8.0f && !saturated) {
    integral += error * dt;
    integral = constrain(integral, -300.0f, 300.0f);
  }

  float raw_deriv = (error - last_error) / dt;
  d_filtered = BETA * raw_deriv + (1.0f - BETA) * d_filtered;
  last_error = error;

  float correction = Kp * error + Ki * integral + Kd * d_filtered;
  correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
  ibus_channels[2] = constrain(base_throttle + (int)correction, 1100, 1900);
}

// ─────────────────────────────────────────
// 起飛爬升（地面切入定高用）
// ─────────────────────────────────────────
void updateTakeoff() {
  unsigned long now = millis();
  if (now - last_pid_time < 20) return;
  last_pid_time = now;

  // 逾時保護：8 秒內未離地則自動解除定高
  if (now - takeoff_start_ms > TAKEOFF_TIMEOUT_MS) {
    alt_hold_active  = false;
    takeoff_phase    = false;
    ibus_channels[2] = manual_throttle;
    return;
  }

  float cur_alt = (filtered_mm > 0) ? filtered_mm / 10.0f : 0;

  if (cur_alt >= TAKEOFF_LIFTOFF_CM) {
    // 已離地：快照當下油門為懸停基準，切換為 PID
    base_throttle = (int)ibus_channels[2];
    takeoff_phase = false;
    integral      = 0;
    last_error    = target_alt_cm - cur_alt;
    // 用前後兩筆估算上升速度，初始化 D 項，避免第一拍誤以為靜止而繼續加油
    if (prev_filtered_mm > 0 && filtered_mm > 0) {
      d_filtered = -(filtered_mm - prev_filtered_mm) / 0.02f / 10.0f;
    } else {
      d_filtered = 0;
    }
    return;
  }

  // 緩坡加油門（每 20ms +2 IBUS = 100 IBUS/秒）
  ibus_channels[2] = constrain((int)ibus_channels[2] + 2, 1000, TAKEOFF_MAX_IBUS);
}

// ─────────────────────────────────────────
// 感測器就緒檢查（非阻塞）
// ─────────────────────────────────────────
bool sensorDataReady() {
  return (altSensor.readReg(0x13) & 0x07) != 0;
}

// ─────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  BTSerial.begin(9600);
  Wire.begin();
  gripper_servo.attach(6);
  if (altSensor.init()) {
    altSensor.setTimeout(50);
    altSensor.setMeasurementTimingBudget(20000);
    altSensor.startContinuous();
    sensor_ok = true;
  }
}

// ─────────────────────────────────────────
// 藍牙協議：S + 8 bytes payload + 1 byte XOR checksum（共 10 bytes）
// [0] thr_val  搖桿油門 (0~255)
// [1] y_val    偏航
// [2] p_val    俯仰
// [3] r_val    翻滾
// [4] alt_val  目標高度 cm；手動時為 0
// [5] g_val    夾爪
// [6] arm_val  解鎖狀態
// [7] ah_val   定高開關
// ─────────────────────────────────────────
void loop() {

  // ── 藍牙封包解析（每次 loop 只處理一包，避免積壓時卡住 IBUS 心跳） ──
  if (BTSerial.available() >= 10) {
    if (BTSerial.read() == 'S') {
      uint8_t buf[9];
      for (int i = 0; i < 9; i++) buf[i] = BTSerial.read();

      uint8_t chk = 0;
      for (int i = 0; i < 8; i++) chk ^= buf[i];
      if (chk == buf[8]) {  // checksum 正確才處理

        int thr_val = buf[0];
        int y_val   = buf[1];
        int p_val   = buf[2];
        int r_val   = buf[3];
        int alt_val = buf[4];
        int g_val   = buf[5];
        int arm_val = buf[6];
        int ah_val  = buf[7];

        ibus_channels[0] = map(r_val,   0, 255, 1000, 2000);
        ibus_channels[1] = map(p_val,   0, 255, 1000, 2000);
        ibus_channels[3] = map(y_val,   0, 255, 1000, 2000);
        ibus_channels[4] = map(arm_val, 0, 255, 1000, 2000);
        ibus_channels[5] = 2000;

        gripper_servo.writeMicroseconds(map(g_val, 0, 255, 1000, 2000));

        // manual_throttle 只在手動模式下更新，切入定高後保留最後手動值
        if (ah_val == 0) {
          manual_throttle = map(thr_val, 0, 255, 1000, 2000);
        }

        if (ah_val == 1 && !alt_hold_active) {
          if (sensor_ok && filtered_mm < 1300) {
            target_alt_cm   = constrain((float)alt_val, 10.0f, MAX_ALT_CM);
            integral        = 0;
            last_error      = 0;
            d_filtered      = 0;
            last_pid_time   = millis();
            alt_hold_active = true;

            float cur_alt = (filtered_mm > 0) ? filtered_mm / 10.0f : 0;
            if (cur_alt < TAKEOFF_LIFTOFF_CM) {
              // ── 地面起飛：緩坡爬升直到離地，再切 PID ──
              takeoff_phase    = true;
              takeoff_start_ms = millis();
              ibus_channels[2] = max(manual_throttle, 1200);
            } else {
              // ── 已在空中：直接 PID ──
              takeoff_phase = false;
              base_throttle = manual_throttle;
            }
          }

        } else if (ah_val == 1 && alt_hold_active) {
          // ── 定高中：base_throttle 鎖定，只更新目標高度 ──
          float new_target = constrain((float)alt_val, 3.0f, MAX_ALT_CM);
          if (abs(new_target - target_alt_cm) > 0.5f) {
            target_alt_cm = new_target;
          }

        } else if (ah_val == 0 && alt_hold_active) {
          // ── 關閉定高：保持 PID 最後輸出，回傳給 Python 當新基準油門 ──
          int exit_thr    = ibus_channels[2];
          alt_hold_active  = false;
          takeoff_phase    = false;
          ibus_channels[2] = exit_thr;
          char tbuf[16];
          snprintf(tbuf, sizeof(tbuf), "T:%d\n", exit_thr);
          BTSerial.print(tbuf);                 // 通知 Python 更新 base_throttle（一次 TX）

        } else {
          // ── 手動模式 ──
          ibus_channels[2] = manual_throttle;
        }
      }
    }
  }

  // ── 感測器讀取 + EMA 濾波 ──
  if (sensor_ok && millis() - last_sensor_time >= 20 && sensorDataReady()) {
    last_sensor_time = millis();
    int mm = altSensor.readRangeContinuousMillimeters();
    if (!altSensor.timeoutOccurred() && mm > 0 && mm < 2000) {
      if (!filter_inited) {
        filtered_mm   = (float)mm;
        filter_inited = true;
      } else {
        prev_filtered_mm = filtered_mm;
        filtered_mm = ALPHA * mm + (1.0f - ALPHA) * filtered_mm;
      }
    }
  }

  // ── 定高 PID / 起飛爬升 ──
  if (alt_hold_active) {
    if (takeoff_phase) updateTakeoff();
    else               updateAltHold(filtered_mm);
  }

  // ── 回傳高度（+定高時 PID 油門）給 PC（每 200ms） ──
  if (sensor_ok && millis() - last_alt_report >= 200 && filtered_mm > 0) {
    char dbuf[16];
    snprintf(dbuf, sizeof(dbuf), "D:%d\n", (int)filtered_mm);
    BTSerial.print(dbuf);
    if (alt_hold_active) {
      char pbuf[16];
      snprintf(pbuf, sizeof(pbuf), "P:%d\n", ibus_channels[2]);
      BTSerial.print(pbuf);
    }
    last_alt_report = millis();
  }

  // ── IBUS 心跳（每 20ms） ──
  if (millis() - last_ibus_time >= 20) {
    sendIBUS();
    last_ibus_time = millis();
  }
}