// Version: 0.6.1 (ESP32 Ported - Velocity Mode Alt Hold)
// ESP32 port:
// - BluetoothSerial replaces SoftwareSerial
// - ESP32Servo replaces Servo
// - HardwareSerial Serial2 used for iBUS output
// - Explicit I2C pins for ESP32
//
// Notes:
// - Tested logic target: classic ESP32 with BluetoothSerial support
// - Servo signal pin: GPIO 13
// - I2C pins: SDA 21, SCL 22
// - iBUS UART2 pins: RX2 16, TX2 17

#include <Wire.h>
#include <VL53L0X.h>
#include "BluetoothSerial.h"
#include <ESP32Servo.h>
#include <math.h>

// -------------------- Hardware --------------------
BluetoothSerial SerialBT;
Servo gripper_servo;
VL53L0X altSensor;

const int SERVO_PIN = 13;
const int IBUS_RX_PIN = 16;
const int IBUS_TX_PIN = 17;
const int I2C_SDA_PIN = 21;
const int I2C_SCL_PIN = 22;

// -------------------- iBUS Channels --------------------
uint16_t ibus_channels[14] = {
  1500, 1500, 1000, 1500,
  1000, 2000,
  1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500
};

unsigned long last_ibus_time = 0;

// -------------------- Control State --------------------
bool  alt_hold_active = false;
int   manual_throttle = 1500;
float received_vel_cmd = 0.0f;

// Cascaded / velocity PID state
float v_est          = 0.0f;
float integral_vel   = 0.0f;
float last_vel_error = 0.0f;
float prev_cm_filt   = -1.0f;
float prev_cm_raw    = -1.0f;
int   hover_estimate = 1500;
unsigned long last_pid_time = 0;

// -------------------- Sensor State --------------------
bool  sensor_ok          = false;
float filtered_mm        = -1.0f;
float prev_filtered_mm   = -1.0f;
bool  filter_inited      = false;
unsigned long last_sensor_time   = 0;
unsigned long last_valid_range_time = 0;
unsigned long last_alt_report    = 0;
bool  alt_reading_frozen = false;
int   freeze_count       = 0;

// -------------------- PID Parameters --------------------
const float Kp_vel           = 0.80f;
const float Ki_vel           = 1.20f;
const float Kd_vel           = 0.0f;
const float MAX_VEL_CMD      = 60.0f;
const float MAX_INTEGRAL_VEL = 220.0f;
const float MAX_THR_OFFSET   = 220.0f;

const float ALPHA            = 0.35f;
const float GAMMA            = 0.20f;

// Altitude / control limits
const float MAX_ALT_CM       = 130.0f;   // Effective control ceiling
const int   MAX_THR_STEP     = 20;

// Failsafe timing
const unsigned long SENSOR_READ_INTERVAL_MS   = 20;
const unsigned long ALT_REPORT_INTERVAL_MS    = 200;
const unsigned long IBUS_SEND_INTERVAL_MS     = 20;
const unsigned long SENSOR_FRESH_TIMEOUT_MS   = 120;

// -------------------- Helpers --------------------
void resetAltHoldController() {
  integral_vel     = 0.0f;
  last_vel_error   = 0.0f;
  v_est            = 0.0f;
  prev_cm_filt     = -1.0f;
  prev_cm_raw      = -1.0f;
  received_vel_cmd = 0.0f;
  last_pid_time    = millis();
}

void exitAltHoldToManual(int exit_thr) {
  alt_hold_active  = false;
  manual_throttle  = exit_thr;
  ibus_channels[2] = exit_thr;
  resetAltHoldController();

  char tbuf[16];
  snprintf(tbuf, sizeof(tbuf), "T:%d\n", exit_thr);
  SerialBT.print(tbuf);

  Serial.println("Alt Hold OFF -> Manual throttle resumed");
}

bool sensorDataReady() {
  return (altSensor.readReg(0x13) & 0x07) != 0;
}

bool sensorFreshEnough() {
  return filter_inited && (millis() - last_valid_range_time <= SENSOR_FRESH_TIMEOUT_MS);
}

void sendIBUS() {
  uint8_t packet[32];
  packet[0] = 0x20;
  packet[1] = 0x40;

  uint16_t checksum = 0xFFFF - packet[0] - packet[1];

  for (int i = 0; i < 14; i++) {
    packet[2 + i * 2] = ibus_channels[i] & 0xFF;
    packet[3 + i * 2] = (ibus_channels[i] >> 8) & 0xFF;
    checksum -= packet[2 + i * 2];
    checksum -= packet[3 + i * 2];
  }

  packet[30] = checksum & 0xFF;
  packet[31] = (checksum >> 8) & 0xFF;

  Serial2.write(packet, 32);
}

void updateAltHold(float filt_mm) {
  unsigned long now = millis();
  float dt = (now - last_pid_time) / 1000.0f;

  if (dt < 0.02f) return;
  last_pid_time = now;

  // Failsafe: sensor data stale
  if (!sensorFreshEnough()) {
    if (alt_hold_active) {
      int fallback_thr = manual_throttle;
      alt_hold_active = false;
      ibus_channels[2] = fallback_thr;
      manual_throttle = fallback_thr;
      resetAltHoldController();
      SerialBT.print("F:2\n");
      Serial.println("Alt Hold disabled: sensor data stale");
    }
    return;
  }

  if (filt_mm < 0.0f) return;

  float current_cm = filt_mm / 10.0f;

  // Out-of-range protection for this controller
  if (current_cm > MAX_ALT_CM) {
    received_vel_cmd = min(received_vel_cmd, -5.0f);
  }

  if (prev_cm_raw > 0.0f && fabs(current_cm - prev_cm_raw) > 15.0f) {
    prev_cm_raw  = current_cm;
    prev_cm_filt = current_cm;
    v_est = 0.0f;
    return;
  }
  prev_cm_raw = current_cm;

  if (prev_cm_filt < 0.0f) prev_cm_filt = current_cm;

  float v_raw = constrain((current_cm - prev_cm_filt) / dt, -100.0f, 100.0f);
  v_est = GAMMA * v_raw + (1.0f - GAMMA) * v_est;
  prev_cm_filt = current_cm;

  float vel_error = received_vel_cmd - v_est;
  float d_vel = (vel_error - last_vel_error) / dt;
  last_vel_error = vel_error;

  float tentative_offset = Kp_vel * vel_error
                         + Ki_vel * integral_vel
                         + Kd_vel * d_vel;

  if (fabs(tentative_offset) < MAX_THR_OFFSET * 0.85f) {
    integral_vel += vel_error * dt;
  } else if (vel_error * integral_vel > 0.0f) {
    integral_vel *= 0.92f;
  }

  integral_vel = constrain(integral_vel, -MAX_INTEGRAL_VEL, MAX_INTEGRAL_VEL);

  float thr_offset = Kp_vel * vel_error
                   + Ki_vel * integral_vel
                   + Kd_vel * d_vel;

  thr_offset = constrain(thr_offset, -MAX_THR_OFFSET, MAX_THR_OFFSET);

  int desired = constrain(hover_estimate + (int)thr_offset, 1100, 1750);

  ibus_channels[2] = constrain(
    desired,
    (int)ibus_channels[2] - MAX_THR_STEP,
    (int)ibus_channels[2] + MAX_THR_STEP
  );
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // iBUS UART2
  Serial2.begin(115200, SERIAL_8N1, IBUS_RX_PIN, IBUS_TX_PIN);

  // Bluetooth
  SerialBT.begin("ESP32_Drone_Hub");
  Serial.println("ESP32 Bluetooth Started!");

  // I2C
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // Servo
  ESP32PWM::allocateTimer(0);
  gripper_servo.setPeriodHertz(50);
  gripper_servo.attach(SERVO_PIN, 500, 2400);

  // ToF sensor
  if (altSensor.init()) {
    altSensor.setTimeout(50);
    altSensor.setMeasurementTimingBudget(20000);
    altSensor.startContinuous();
    sensor_ok = true;
    Serial.println("VL53L0X Init OK");
  } else {
    sensor_ok = false;
    Serial.println("VL53L0X Init Failed");
  }
}

void loop() {
  // -------------------- Bluetooth Packet Parsing --------------------
  while (SerialBT.available()) {
    if (SerialBT.peek() != 'S') {
      SerialBT.read();
      continue;
    }

    if (SerialBT.available() < 10) break;

    SerialBT.read(); // consume 'S'
    uint8_t buf[9];
    for (int i = 0; i < 9; i++) {
      buf[i] = SerialBT.read();
    }

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

      if (alt_hold_active) {
        received_vel_cmd = constrain((float)(alt_val - 128), -MAX_VEL_CMD, MAX_VEL_CMD);
      }

      if (ah_val == 1 && !alt_hold_active) {
        if (sensor_ok && sensorFreshEnough()) {
          resetAltHoldController();
          hover_estimate  = (manual_throttle >= 1250) ? manual_throttle : 1500;
          alt_hold_active = true;
          Serial.println("Alt Hold ON");
        } else {
          Serial.println("Alt Hold request rejected: sensor not ready");
        }

      } else if (ah_val == 2 && alt_hold_active) {
        hover_estimate = ibus_channels[2];
        resetAltHoldController();
        Serial.println("Alt Hold recentered");

      } else if (ah_val == 0 && alt_hold_active) {
        int exit_thr = ibus_channels[2];
        exitAltHoldToManual(exit_thr);

      } else if (ah_val == 0 && !alt_hold_active) {
        ibus_channels[2] = manual_throttle;
      }
    }

    // Keep one packet per loop for predictable cycle timing
    break;
  }

  // -------------------- Sensor Read --------------------
  if (sensor_ok && millis() - last_sensor_time >= SENSOR_READ_INTERVAL_MS && sensorDataReady()) {
    last_sensor_time = millis();

    int mm = altSensor.readRangeContinuousMillimeters();

    if (!altSensor.timeoutOccurred() && mm > 0 && mm < 2000) {
      last_valid_range_time = millis();

      float new_filt = (float)mm;
      bool was_rejected = false;

      if (filter_inited && fabs(new_filt - filtered_mm) > 200.0f) {
        new_filt = filtered_mm;
        was_rejected = true;
      }

      bool skip_ema = false;

      if (was_rejected) {
        if (++freeze_count >= 5) {
          if (!alt_reading_frozen) {
            alt_reading_frozen = true;
            SerialBT.print("F:1\n");
          }

          filtered_mm   = (float)mm;
          filter_inited = true;
          freeze_count  = 0;
          skip_ema      = true;

          resetAltHoldController();
        }
      } else {
        freeze_count = 0;
        if (alt_reading_frozen) {
          alt_reading_frozen = false;
          SerialBT.print("F:0\n");
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

  // -------------------- Alt Hold Update --------------------
  if (alt_hold_active) {
    updateAltHold(filtered_mm);
  }

  // -------------------- Telemetry --------------------
  if (sensor_ok && millis() - last_alt_report >= ALT_REPORT_INTERVAL_MS && filtered_mm > 0.0f) {
    char dbuf[16];
    snprintf(dbuf, sizeof(dbuf), "D:%d\n", (int)filtered_mm);
    SerialBT.print(dbuf);

    if (alt_hold_active) {
      char pbuf[16];
      snprintf(pbuf, sizeof(pbuf), "P:%d\n", ibus_channels[2]);
      SerialBT.print(pbuf);
    }

    last_alt_report = millis();
  }

  // -------------------- iBUS Output --------------------
  if (millis() - last_ibus_time >= IBUS_SEND_INTERVAL_MS) {
    sendIBUS();
    last_ibus_time = millis();
  }
}
