#include <Servo.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <VL53L0X.h>

// ─────────────────────────────────────────
// 硬體初始化
// ─────────────────────────────────────────
SoftwareSerial BTSerial(10, 11); // RX=D10, TX=D11
Servo gripper_servo;
VL53L0X altSensor;

// ─────────────────────────────────────────
// IBUS 通道陣列
//   CH1(0): Roll   CH2(1): Pitch
//   CH3(2): Throttle  CH4(3): Yaw
//   CH5(4): ARM    CH6(5): Angle Mode (固定2000)
// ─────────────────────────────────────────
uint16_t ibus_channels[14] = {
  1500, 1500, 1000, 1500,
  1000, 2000,
  1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500
};
unsigned long last_ibus_time = 0;

// ─────────────────────────────────────────
// 定高 PID 參數
// ─────────────────────────────────────────
bool  alt_hold_active = false;
float target_alt_cm   = 0;
float last_error      = 0;
float integral        = 0;
int   base_throttle   = 1500;
unsigned long last_pid_time = 0;

const float Kp = 2.5;
const float Ki = 0.05;
const float Kd = 6.0;

// ─────────────────────────────────────────
// IBUS 封包發送
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
// 定高 PID 迴圈（50Hz 更新）
// ─────────────────────────────────────────
void updateAltHold() {
  unsigned long now = millis();
  float dt = (now - last_pid_time) / 1000.0f;
  if (dt < 0.02f) return;
  last_pid_time = now;

  int raw_mm = altSensor.readRangeContinuousMillimeters();
  if (altSensor.timeoutOccurred()) return;
  float current_alt_cm = raw_mm / 10.0f;

  if (current_alt_cm < 3 || current_alt_cm > 130) return;

  float error      = target_alt_cm - current_alt_cm;
  integral        += error * dt;
  integral         = constrain(integral, -30.0f, 30.0f);
  float derivative = (error - last_error) / dt;
  last_error       = error;

  float correction = Kp * error + Ki * integral + Kd * derivative;

  int throttle_out = base_throttle + (int)correction;
  ibus_channels[2] = constrain(throttle_out, 1100, 1900);
}

// ─────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  BTSerial.begin(9600);
  Wire.begin();
  gripper_servo.attach(6);

  altSensor.init();
  altSensor.setTimeout(300);
  altSensor.setMeasurementTimingBudget(20000);
  altSensor.startContinuous();
}

// ─────────────────────────────────────────
// 藍牙協議：S + 8 bytes（共 9 bytes）
//   [0] thr_val    當前真實油門 (0~255)，Arduino 以此為 PID base_throttle
//   [1] y_val      偏航
//   [2] p_val      俯仰
//   [3] r_val      翻滾
//   [4] alt_val    目標高度 cm (0~120)；手動模式時送 0
//   [5] g_val      夾爪 (0~255)，Arduino 直接驅動伺服馬達（D6），不送飛控
//   [6] arm_val    解鎖狀態
//   [7] ah_val     定高開關 (0=手動, 1=定高)
// ─────────────────────────────────────────
void loop() {
  while (BTSerial.available() >= 9) {
    if (BTSerial.read() == 'S') {
      int thr_val = BTSerial.read();   // 當前油門 0~255
      int y_val   = BTSerial.read();
      int p_val   = BTSerial.read();
      int r_val   = BTSerial.read();
      int alt_val = BTSerial.read();   // 目標高度 cm（定高時有效）
      int g_val   = BTSerial.read();   // 夾爪
      int arm_val = BTSerial.read();
      int ah_val  = BTSerial.read();

      // Roll / Pitch / Yaw / ARM 照常更新（送飛控）
      ibus_channels[0] = map(r_val, 0, 255, 1000, 2000);
      ibus_channels[1] = map(p_val, 0, 255, 1000, 2000);
      ibus_channels[3] = map(y_val, 0, 255, 1000, 2000);
      ibus_channels[4] = map(arm_val, 0, 255, 1000, 2000);
      ibus_channels[5] = 2000;

      // 夾爪：Arduino 直接驅動伺服馬達，不經過飛控
      gripper_servo.writeMicroseconds(map(g_val, 0, 255, 1000, 2000));

      // ── 定高狀態機 ──
      if (ah_val == 1 && !alt_hold_active) {
        // 【剛切入定高】以當前油門為懸停基準，以當下感測高度為目標
        int raw_mm = altSensor.readRangeContinuousMillimeters();
        if (!altSensor.timeoutOccurred() && raw_mm > 30 && raw_mm < 1300) {
          target_alt_cm   = raw_mm / 10.0f;         // 先用真實高度
          base_throttle   = map(thr_val, 0, 255, 1000, 2000);  // 用真實油門
          integral        = 0;
          last_error      = 0;
          last_pid_time   = millis();
          alt_hold_active = true;
        }
        // 感測器無效時不切入，保持手動（安全保護）

      } else if (ah_val == 1 && alt_hold_active) {
        // 【定高中】持續更新 base_throttle 與目標高度
        base_throttle = map(thr_val, 0, 255, 1000, 2000);
        float new_target = constrain((float)alt_val, 3.0f, 120.0f);
        if (new_target != target_alt_cm) {
          target_alt_cm = new_target;   // PID 平滑過渡，不重置積分
        }

      } else if (ah_val == 0 && alt_hold_active) {
        // 【關閉定高】回到手動油門
        alt_hold_active  = false;
        ibus_channels[2] = map(thr_val, 0, 255, 1000, 2000);

      } else {
        // 【手動模式】正常更新油門
        ibus_channels[2] = map(thr_val, 0, 255, 1000, 2000);
      }
    }
  }

  // 定高 PID 更新
  if (alt_hold_active) {
    updateAltHold();
  }

  // IBUS 心跳：每 20ms 發一次
  if (millis() - last_ibus_time >= 20) {
    sendIBUS();
    last_ibus_time = millis();
  }
}
