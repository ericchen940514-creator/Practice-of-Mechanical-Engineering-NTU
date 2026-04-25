// Version: 0.6.0  (Velocity Mode Alt Hold)
// 核心改動：拿掉外環位置 P，直接用 Python 送來的目標速度 (vel_cmd) 控制內環
// alt_val byte 重新定義：128 = 0 cm/s，>128 = 上升，<128 = 下降
// 保留：起飛爬升、Outlier Reject、spike filter、連續定高通訊
#include <Servo.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <VL53L0X.h>

SoftwareSerial BTSerial(10, 11);
Servo gripper_servo;
VL53L0X altSensor;

uint16_t ibus_channels[14] = {
  1500, 1500, 1000, 1500,
  1000, 2000,
  1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500
};
unsigned long last_ibus_time = 0;

bool  alt_hold_active = false;
int   manual_throttle = 1500;

// 速度模式：Python 送來的目標速度 (cm/s)，128=0
float received_vel_cmd = 0;

// 串級 PID 狀態（保留速度估計，拿掉位置外環）
float v_est          = 0;
float integral_vel   = 0;
float last_vel_error = 0;
float prev_cm_filt   = -1;
float prev_cm_raw    = -1;
int   hover_estimate = 1500;

unsigned long last_pid_time    = 0;

bool  sensor_ok        = false;
float filtered_mm      = -1;
float prev_filtered_mm = -1;
bool  filter_inited    = false;
unsigned long last_sensor_time = 0;
unsigned long last_alt_report  = 0;

bool alt_reading_frozen = false;
int  freeze_count       = 0;

// ─── PID 參數（同 v5，只移除 Kp_pos）───
const float Kp_vel            = 0.80f;   // 速度誤差 P：直接推油門
const float Ki_vel            = 1.20f;   // 積分負責找到懸停油門 + 維持速度
const float Kd_vel            = 0.0f;    // 速度模式不用 D：搖桿跳動會讓 D 項暴衝
const float MAX_VEL_CMD       = 60.0f;
const float MAX_INTEGRAL_VEL  = 220.0f; // 允許積分完整補償懸停誤差
const float MAX_THR_OFFSET    = 220.0f;

const float ALPHA             = 0.35f;
const float GAMMA             = 0.20f;

const float MAX_ALT_CM            = 300.0f;
const int   MAX_THR_STEP          = 20;

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

// ─── 速度模式 PID（移除外環，直接用 received_vel_cmd）───
void updateAltHold(float filt_mm) {
  unsigned long now = millis();
  float dt = (now - last_pid_time) / 1000.0f;
  if (dt < 0.02f) return;
  last_pid_time = now;

  if (filt_mm < 0) return;
  float current_cm = filt_mm / 10.0f;
  if (current_cm > 130) return;

  if (prev_cm_raw > 0 && fabs(current_cm - prev_cm_raw) > 15.0f) {
    prev_cm_raw  = current_cm;
    prev_cm_filt = current_cm;   // 重設基準，防止下一拍速度估計爆炸
    v_est = 0;                   // 清除殘留速度估計
    return;
  }
  prev_cm_raw = current_cm;

  // 高度上限安全：超過 MAX_ALT_CM 強制速度為負（往下）
  float safe_vel_cmd = received_vel_cmd;
  if (current_cm > MAX_ALT_CM) {
    safe_vel_cmd = min(safe_vel_cmd, -5.0f);
  }

  // 速度估計（clamp 防感測器雜訊產生不合理速度）
  if (prev_cm_filt < 0) prev_cm_filt = current_cm;
  float v_raw = constrain((current_cm - prev_cm_filt) / dt, -100.0f, 100.0f);
  v_est = GAMMA * v_raw + (1.0f - GAMMA) * v_est;
  prev_cm_filt = current_cm;

  // 內環速度 PID（直接對 vel_cmd，無外環位置 P）
  float vel_error = safe_vel_cmd - v_est;

  float d_vel = (vel_error - last_vel_error) / dt;
  last_vel_error = vel_error;

  float tentative_offset = Kp_vel * vel_error
                         + Ki_vel * integral_vel
                         + Kd_vel * d_vel;

  if (fabs(tentative_offset) < MAX_THR_OFFSET * 0.85f) {
    integral_vel += vel_error * dt;
  } else if (vel_error * integral_vel > 0) {
    integral_vel *= 0.92f;
  }
  integral_vel = constrain(integral_vel, -MAX_INTEGRAL_VEL, MAX_INTEGRAL_VEL);

  float thr_offset = Kp_vel * vel_error
                   + Ki_vel * integral_vel
                   + Kd_vel * d_vel;
  thr_offset = constrain(thr_offset, -MAX_THR_OFFSET, MAX_THR_OFFSET);

  int desired = constrain(hover_estimate + (int)thr_offset, 1100, 1750);
  ibus_channels[2] = constrain(desired,
                                (int)ibus_channels[2] - MAX_THR_STEP,
                                (int)ibus_channels[2] + MAX_THR_STEP);
}


bool sensorDataReady() {
  return (altSensor.readReg(0x13) & 0x07) != 0;
}

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

void loop() {

  // 封包對齊：丟棄非 'S' 的雜訊 byte，避免永久錯位
  while (BTSerial.available()) {
    if (BTSerial.peek() != 'S') { BTSerial.read(); continue; }
    if (BTSerial.available() < 10) break;
    BTSerial.read(); // 吃掉 'S'
    uint8_t buf[9];
    for (int i = 0; i < 9; i++) buf[i] = BTSerial.read();

      uint8_t chk = 0;
      for (int i = 0; i < 8; i++) chk ^= buf[i];
      if (chk == buf[8]) {

        int thr_val = buf[0];
        int y_val   = buf[1];
        int p_val   = buf[2];
        int r_val   = buf[3];
        int alt_val = buf[4];   // 速度模式：128=0 cm/s
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

        // 持續更新速度命令
        if (alt_hold_active) {
          received_vel_cmd = constrain((float)(alt_val - 128), -MAX_VEL_CMD, MAX_VEL_CMD);
        }

        if (ah_val == 1 && !alt_hold_active) {
          if (sensor_ok) {
            integral_vel     = 0;
            last_vel_error   = 0;
            v_est            = 0;
            prev_cm_filt     = -1;
            prev_cm_raw      = -1;
            received_vel_cmd = 0;
            last_pid_time    = millis();
            // manual_throttle 可能因 Python base_throttle=0 而是 1000，
            // 太低的話用 1500 當懸停基準，讓積分從合理起點開始收斂
            hover_estimate   = (manual_throttle >= 1250) ? manual_throttle : 1500;
            alt_hold_active  = true;
          }

        } else if (ah_val == 2 && alt_hold_active) {
          // 積分重設（相當於讓懸停油門重新收斂）
          integral_vel     = 0;
          last_vel_error   = 0;
          v_est            = 0;
          prev_cm_filt     = -1;
          prev_cm_raw      = -1;
          received_vel_cmd = 0;

        } else if (ah_val == 0 && alt_hold_active) {
          int exit_thr     = ibus_channels[2];
          alt_hold_active  = false;
          // 退出時把懸停油門直接同步給 manual_throttle，
          // 避免 Python 收到 T: 前這幾幀用舊油門拉扯
          manual_throttle  = exit_thr;
          ibus_channels[2] = exit_thr;
          char tbuf[16];
          snprintf(tbuf, sizeof(tbuf), "T:%d\n", exit_thr);
          BTSerial.print(tbuf);

        } else if (ah_val == 0 && !alt_hold_active) {
          ibus_channels[2] = manual_throttle;
        }
      }
    break; // 一次 loop 只處理一個有效封包
  }

  // ── 感測器讀取 + EMA + Outlier Rejection ──
  if (sensor_ok && millis() - last_sensor_time >= 20 && sensorDataReady()) {
    last_sensor_time = millis();
    int mm = altSensor.readRangeContinuousMillimeters();
    if (!altSensor.timeoutOccurred() && mm > 0 && mm < 2000) {
      float new_filt = (float)mm;

      bool was_rejected = false;
      if (filter_inited && fabs(new_filt - filtered_mm) > 200.0f) {
        new_filt     = filtered_mm;
        was_rejected = true;
      }

      bool skip_ema = false;
      if (was_rejected) {
        if (++freeze_count >= 5) {
          // 連續 5 次被拒絕：真實高度已改變，強制接受新讀值並重置濾波器
          if (!alt_reading_frozen) {
            alt_reading_frozen = true;
            BTSerial.print("F:1\n");
          }
          filtered_mm   = (float)mm;
          filter_inited = true;
          freeze_count  = 0;
          skip_ema      = true;   // 這拍直接用原始值，不做 EMA 混合
          // 高度已確認大幅改變，清除 PID 狀態，防止舊積分造成突然上衝
          integral_vel   = 0;
          last_vel_error = 0;
          v_est          = 0;
          prev_cm_filt   = -1;
          prev_cm_raw    = -1;
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
      } else if (!skip_ema) {
        prev_filtered_mm = filtered_mm;
        filtered_mm = ALPHA * new_filt + (1.0f - ALPHA) * filtered_mm;
      }
    }
  }

  if (alt_hold_active) {
    updateAltHold(filtered_mm);
  }

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

  if (millis() - last_ibus_time >= 20) {
    sendIBUS();
    last_ibus_time = millis();
  }
}
