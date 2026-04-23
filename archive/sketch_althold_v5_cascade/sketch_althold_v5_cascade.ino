// Version: 0.4.20  (Cascade PID + Outlier Rejection + 修正後抗積分風暴 + 消除 Yo-Yo 效應)
// 最終保守版：已修正 d_vel 計算順序 + 條件積分防 windup
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
// IBUS 通道
// CH1(0): Roll  CH2(1): Pitch  CH3(2): Throttle  CH4(3): Yaw
// CH5(4): ARM   CH6(5): Angle Mode (固定 2000)
// ─────────────────────────────────────────
uint16_t ibus_channels[14] = {
  1500, 1500, 1000, 1500,
  1000, 2000,
  1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500
};
unsigned long last_ibus_time = 0;

// ─────────────────────────────────────────
// 定高狀態
// ─────────────────────────────────────────
bool  alt_hold_active = false;
bool  takeoff_phase   = false;
float target_alt_cm   = 0;
int   manual_throttle = 1500;

// 串級 PID 狀態
float v_est          = 0;       // 速度估計 (cm/s)
float integral_vel   = 0;       // 內環速度積分
float last_vel_error = 0;
float prev_cm_filt   = -1;      // 速度估計用（只在接受讀值時更新）
float prev_cm_raw    = -1;      // outlier 邊界追蹤（拒絕時也更新）
int   hover_estimate = 1400;

unsigned long last_pid_time    = 0;
unsigned long takeoff_start_ms = 0;

// 感測器
bool  sensor_ok        = false;
float filtered_mm      = -1;
float prev_filtered_mm = -1;
bool  filter_inited    = false;
unsigned long last_sensor_time = 0;
unsigned long last_alt_report  = 0;

// 平台突波濾波器狀態
float         spike_accepted_mm    = -1.0f;
unsigned long spike_accepted_t     = 0;
bool          pending_target_update = false;  // 高度基準重設：等下一次 sensor 讀值後更新 target

// 凍結狀態偵測（hardOutlierReject 連續觸發 → Python 顯示警示）
bool alt_reading_frozen = false;
int  freeze_count       = 0;

// ─────────────────────────────────────────
// 【第 4 版】串級 PID 參數 ─ Yo-Yo + 抗積分風暴 最終保守版
// ─────────────────────────────────────────
const float Kp_pos            = 0.8f;    // 外環溫和
const float Kp_vel            = 0.60f;    // 消除溜溜球效應關鍵值
const float Ki_vel            = 0.40f;    // 積分適中
const float Kd_vel            = 0.25f;    // 阻尼加強
const float MAX_VEL_CMD       = 25.0f;
const float MAX_INTEGRAL_VEL  = 85.0f;
const float MAX_THR_OFFSET    = 185.0f;

const float ALPHA             = 0.35f;    // 位置 EMA
const float GAMMA             = 0.20f;    // 速度 EMA

const int   TAKEOFF_LIFTOFF_CM    = 12;   // 晚一點切 PID
const int   TAKEOFF_MAX_IBUS      = 1627;
const unsigned long TAKEOFF_TIMEOUT_MS = 8000;
const float MAX_ALT_CM            = 90.0f;
const int   MAX_THR_STEP          = 10;
const float ALT_SPIKE_MAX_DROP_RATE = 30.0f;  // cm/s, 下降速率上限（防平台誤觸）

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
// 串級 PID 更新（50 Hz）─ 已修正 d_vel 順序 + 完整抗積分風暴
// ─────────────────────────────────────────
void updateAltHold(float filt_mm) {
  unsigned long now = millis();
  float dt = (now - last_pid_time) / 1000.0f;
  if (dt < 0.02f) return;
  last_pid_time = now;

  if (filt_mm < 0) return;
  float current_cm = filt_mm / 10.0f;
  if (current_cm > 130) return;

  // 高度突變保護（防 outlier）
  // prev_cm_raw 追蹤邊界（拒絕時也更新），prev_cm_filt 只在接受時更新（保護速度估計）
  if (prev_cm_raw > 0 && fabs(current_cm - prev_cm_raw) > 15.0f) {
    prev_cm_raw = current_cm;   // 更新邊界讓下一拍能恢復
    return;                     // prev_cm_filt 保持不動，避免速度估計飛衝
  }
  prev_cm_raw = current_cm;

  float effective_target = (current_cm > MAX_ALT_CM) ? MAX_ALT_CM : target_alt_cm;

  // 速度估計
  if (prev_cm_filt < 0) prev_cm_filt = current_cm;
  float v_raw = (current_cm - prev_cm_filt) / dt;
  v_est = GAMMA * v_raw + (1.0f - GAMMA) * v_est;
  prev_cm_filt = current_cm;

  // 外環：位置 P
  float pos_error = effective_target - current_cm;
  float vel_cmd   = Kp_pos * pos_error;
  vel_cmd = constrain(vel_cmd, -MAX_VEL_CMD, MAX_VEL_CMD);

  // ── 內環：速度 PID + 條件積分防 windup ──
  float vel_error = vel_cmd - v_est;

  // 先計算微分項（修正順序！）
  float d_vel = (vel_error - last_vel_error) / dt;
  last_vel_error = vel_error;

  // 預估這一拍總輸出（用來判斷是否即將飽和）
  float tentative_offset = Kp_vel * vel_error 
                         + Ki_vel * integral_vel 
                         + Kd_vel * d_vel;

  // 工業級抗積分風暴
  if (fabs(tentative_offset) < MAX_THR_OFFSET * 0.85f) {
      integral_vel += vel_error * dt;            // 正常累積
  } else if (vel_error * integral_vel > 0) {     // 飽和且誤差同向 → 洩漏
      integral_vel *= 0.92f;
  }

  integral_vel = constrain(integral_vel, -MAX_INTEGRAL_VEL, MAX_INTEGRAL_VEL);

  // 最終油門偏移
  float thr_offset = Kp_vel * vel_error
                   + Ki_vel * integral_vel
                   + Kd_vel * d_vel;

  thr_offset = constrain(thr_offset, -MAX_THR_OFFSET, MAX_THR_OFFSET);

  int desired = constrain(hover_estimate + (int)thr_offset, 1100, 1627);

  // 油門速率限制
  ibus_channels[2] = constrain(desired,
                                (int)ibus_channels[2] - MAX_THR_STEP,
                                (int)ibus_channels[2] + MAX_THR_STEP);
}

// ─────────────────────────────────────────
// 起飛爬升
// ─────────────────────────────────────────
void updateTakeoff() {
  unsigned long now = millis();
  if (now - last_pid_time < 20) return;
  last_pid_time = now;

  if (now - takeoff_start_ms > TAKEOFF_TIMEOUT_MS) {
    alt_hold_active  = false;
    takeoff_phase    = false;
    ibus_channels[2] = manual_throttle;
    return;
  }

  float cur_alt = (filtered_mm > 0) ? filtered_mm / 10.0f : 0;

  if (cur_alt >= TAKEOFF_LIFTOFF_CM) {
    hover_estimate = (int)ibus_channels[2];
    takeoff_phase  = false;
    integral_vel   = 0;
    last_vel_error = 0;
    prev_cm_filt   = cur_alt;
    prev_cm_raw    = cur_alt;
    if (prev_filtered_mm > 0 && filtered_mm > 0) {
      v_est = (filtered_mm - prev_filtered_mm) / 0.02f / 10.0f;
    } else {
      v_est = 0;
    }
    return;
  }

  ibus_channels[2] = constrain((int)ibus_channels[2] + 2, 1000, TAKEOFF_MAX_IBUS);
}

// ─────────────────────────────────────────
// 感測器
// ─────────────────────────────────────────
bool sensorDataReady() {
  return (altSensor.readReg(0x13) & 0x07) != 0;
}

// ─────────────────────────────────────────
// setup
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
// loop
// ─────────────────────────────────────────
void loop() {

  // ── 藍牙封包解析（完全保留原版）──
  if (BTSerial.available() >= 10) {
    if (BTSerial.read() == 'S') {
      uint8_t buf[9];
      for (int i = 0; i < 9; i++) buf[i] = BTSerial.read();

      uint8_t chk = 0;
      for (int i = 0; i < 8; i++) chk ^= buf[i];
      if (chk == buf[8]) {

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

        if (ah_val == 0) {
          manual_throttle = map(thr_val, 0, 255, 1000, 2000);
        }

        if (ah_val == 1 && !alt_hold_active) {
          if (sensor_ok && filtered_mm < 1300) {
            target_alt_cm  = constrain((float)alt_val, 10.0f, MAX_ALT_CM);
            integral_vel   = 0;
            last_vel_error = 0;
            v_est          = 0;
            prev_cm_filt   = -1;
            prev_cm_raw    = -1;
            last_pid_time  = millis();
            alt_hold_active = true;

            float cur_alt = (filtered_mm > 0) ? filtered_mm / 10.0f : 0;
            if (cur_alt < TAKEOFF_LIFTOFF_CM) {
              takeoff_phase    = true;
              takeoff_start_ms = millis();
              ibus_channels[2] = max(manual_throttle, 1200);
            } else {
              takeoff_phase  = false;
              hover_estimate = manual_throttle;
              prev_cm_filt   = cur_alt;
              prev_cm_raw    = cur_alt;
              if (prev_filtered_mm > 0 && filtered_mm > 0) {
                v_est = (filtered_mm - prev_filtered_mm) / 0.02f / 10.0f;
              }
            }
          }

        } else if (ah_val == 1 && alt_hold_active) {
          float new_target = constrain((float)alt_val, 3.0f, MAX_ALT_CM);
          if (abs(new_target - target_alt_cm) > 0.5f) {
            target_alt_cm = new_target;
          }

        } else if (ah_val == 2 && alt_hold_active) {
          // 高度基準重設：解凍感測器，等下一個讀值後重新定義 target_alt_cm
          filter_inited         = false;
          spike_accepted_mm     = -1.0f;
          pending_target_update = true;
          integral_vel          = 0;
          last_vel_error        = 0;
          v_est                 = 0;
          prev_cm_filt          = -1;
          prev_cm_raw           = -1;

        } else if (ah_val == 0 && alt_hold_active) {
          int exit_thr     = ibus_channels[2];
          alt_hold_active  = false;
          takeoff_phase    = false;
          ibus_channels[2] = exit_thr;
          char tbuf[16];
          snprintf(tbuf, sizeof(tbuf), "T:%d\n", exit_thr);
          BTSerial.print(tbuf);

        } else {
          ibus_channels[2] = manual_throttle;
        }
      }
    }
  }

  // ── 感測器讀取 + EMA + Outlier Rejection ──
  if (sensor_ok && millis() - last_sensor_time >= 20 && sensorDataReady()) {
    last_sensor_time = millis();
    int mm = altSensor.readRangeContinuousMillimeters();
    if (!altSensor.timeoutOccurred() && mm > 0 && mm < 2000) {
      float new_filt = (float)mm;
      
      // Outlier Rejection：±200mm 以上直接忽略；記錄是否被拒絕
      bool was_rejected = false;
      if (filter_inited && fabs(new_filt - filtered_mm) > 200.0f) {
        new_filt      = filtered_mm;
        was_rejected  = true;
      }

      // 凍結狀態通知 Python（連續 3 次被拒絕 → 送 F:1；恢復後送 F:0）
      if (was_rejected) {
        if (++freeze_count >= 3 && !alt_reading_frozen) {
          alt_reading_frozen = true;
          BTSerial.print("F:1\n");
        }
      } else {
        freeze_count = 0;
        if (alt_reading_frozen) {
          alt_reading_frozen = false;
          BTSerial.print("F:0\n");
        }
      }

      if (!filter_inited) {
        filtered_mm   = new_filt;
        filter_inited = true;
      } else {
        prev_filtered_mm = filtered_mm;
        filtered_mm = ALPHA * new_filt + (1.0f - ALPHA) * filtered_mm;
      }

      // 平台突波濾波：限制 filtered_mm 的下降速率，防止飛到平台上時 PID 誤判墜落
      float filt_cm = filtered_mm / 10.0f;
      if (spike_accepted_mm >= 0 && last_sensor_time > spike_accepted_t) {
        float dt_s = (last_sensor_time - spike_accepted_t) / 1000.0f;
        float drop_rate = (spike_accepted_mm - filt_cm) / dt_s;
        if (drop_rate > ALT_SPIKE_MAX_DROP_RATE) {
          filt_cm     = spike_accepted_mm - ALT_SPIKE_MAX_DROP_RATE * dt_s;
          filtered_mm = filt_cm * 10.0f;
        }
      }
      spike_accepted_mm = filt_cm;
      spike_accepted_t  = last_sensor_time;

      // 高度基準重設完成：以新讀值為 target，通知 Python
      if (pending_target_update) {
        target_alt_cm         = constrain(filt_cm, 3.0f, MAX_ALT_CM);
        pending_target_update = false;
        char nbuf[16];
        snprintf(nbuf, sizeof(nbuf), "N:%d\n", (int)filtered_mm);
        BTSerial.print(nbuf);
      }
    }
  }

  // ── 定高 PID / 起飛爬升 ──
  if (alt_hold_active) {
    if (takeoff_phase) updateTakeoff();
    else               updateAltHold(filtered_mm);
  }

  // ── 回傳高度 + PID 油門 ──
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

  // ── IBUS 心跳 ──
  if (millis() - last_ibus_time >= 20) {
    sendIBUS();
    last_ibus_time = millis();
  }
}