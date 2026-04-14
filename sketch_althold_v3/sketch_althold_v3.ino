// Version: 0.4.14.1
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
float target_alt_cm   = 0;
float last_error      = 0;
float integral        = 0;
float d_filtered      = 0;      // D 項低通濾波器狀態
int   base_throttle   = 1500;   // PID 用，切入定高時鎖定，不再更新
int   manual_throttle = 1500;   // 永遠跟著搖桿，關閉定高時的 fallback
unsigned long last_pid_time = 0;

// ─────────────────────────────────────────
// 感測器
// ─────────────────────────────────────────
bool  sensor_ok     = false;
float filtered_mm   = -1;
bool  filter_inited = false;
unsigned long last_sensor_time = 0;
unsigned long last_alt_report  = 0;

// ─────────────────────────────────────────
// 係數
// ─────────────────────────────────────────
const float Kp    = 2.5;
const float Ki    = 0.05;
const float Kd    = 0.8;   // 原本 6.0，過大會把感測器噪訊放大成油門跳動
const float ALPHA = 0.15;  // 感測器 EMA（越小越平滑）
const float BETA  = 0.2;   // D 項低通濾波（越小越平滑）

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
  if (current_cm < 3 || current_cm > 130) return;

  float error = target_alt_cm - current_cm;

  integral += error * dt;
  integral = constrain(integral, -30.0f, 30.0f);

  float raw_deriv = (error - last_error) / dt;
  d_filtered = BETA * raw_deriv + (1.0f - BETA) * d_filtered;
  last_error = error;

  float correction = Kp * error + Ki * integral + Kd * d_filtered;
  ibus_channels[2] = constrain(base_throttle + (int)correction, 1100, 1900);
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

        // manual_throttle 永遠跟著搖桿更新
        manual_throttle = map(thr_val, 0, 255, 1000, 2000);

        if (ah_val == 1 && !alt_hold_active) {
          // ── 切入定高：快照當下濾波高度與油門，鎖定 base_throttle ──
          if (sensor_ok && filtered_mm > 30 && filtered_mm < 1300) {
            target_alt_cm   = filtered_mm / 10.0f;
            base_throttle   = manual_throttle;
            integral        = 0;
            last_error      = 0;
            d_filtered      = 0;
            last_pid_time   = millis();
            alt_hold_active = true;
          }
          // 感測器無效時不切入（安全保護）

        } else if (ah_val == 1 && alt_hold_active) {
          // ── 定高中：base_throttle 鎖定，只更新目標高度 ──
          float new_target = constrain((float)alt_val, 3.0f, 120.0f);
          if (abs(new_target - target_alt_cm) > 0.5f) {
            target_alt_cm = new_target;
            integral = 0;  // 目標改變時重置積分，避免 wind-up
          }

        } else if (ah_val == 0 && alt_hold_active) {
          // ── 關閉定高：保持 PID 最後輸出，回傳給 Python 當新基準油門 ──
          int exit_thr    = ibus_channels[2];  // PID 最後輸出值
          alt_hold_active = false;
          ibus_channels[2] = exit_thr;          // 油門不跳
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
        filtered_mm = ALPHA * mm + (1.0f - ALPHA) * filtered_mm;
      }
    }
  }

  // ── 定高 PID ──
  if (alt_hold_active) {
    updateAltHold(filtered_mm);
  }

  // ── 回傳高度給 PC（每 200ms，一次 TX 避免中斷停用兩次） ──
  if (sensor_ok && millis() - last_alt_report >= 200 && filtered_mm > 0) {
    char dbuf[16];
    snprintf(dbuf, sizeof(dbuf), "D:%d\n", (int)filtered_mm);
    BTSerial.print(dbuf);
    last_alt_report = millis();
  }

  // ── IBUS 心跳（每 20ms） ──
  if (millis() - last_ibus_time >= 20) {
    sendIBUS();
    last_ibus_time = millis();
  }
}