// Version: 0.8.0 (no MSP2 sensor output to FC)
// Changes from 0.7.0:
//   - 移除所有送往飛控的 MSP2 感測器資料（Rangefinder + Optical Flow）
//   - VL53L1X / PMW3901 仍由 ESP32 自行讀取（定高 PID + BT 回傳 PC 記錄）
//   - BT telemetry: D:mm (5 Hz) + OF:dx,dy (25 Hz) for PC logging
//
// Wiring:
//   VL53L1X  I2C   SDA=21  SCL=22
//   PMW3901  SPI   MOSI=23 MISO=19 SCK=18 CS=5
//   iBUS     UART2 TX=17   RX=16 (unused)  → FC UART1 RX
//   Servo    GPIO13
//
// iNAV setup required:
//   Ports → UART3 → Sensor Input = MSP,  baud = 115200

#include <Wire.h>
#include <SPI.h>
#include <VL53L1X.h>
#include <Bitcraze_PMW3901.h>
#include "BluetoothSerial.h"
#include <ESP32Servo.h>
#include <math.h>

// -------------------- Hardware --------------------
BluetoothSerial    SerialBT;
Servo              gripper_servo;
VL53L1X            distSensor;
Bitcraze_PMW3901   flowSensor(5);   // CS = GPIO5

const int SERVO_PIN      = 13;
const int IBUS_RX_PIN    = 16;
const int IBUS_TX_PIN    = 17;
const int I2C_SDA_PIN    = 21;
const int I2C_SCL_PIN    = 22;
const int FLOW_RST_PIN   = 26;  // PMW3901 NRESET（低電位重置）

// -------------------- iBUS Channels --------------------
uint16_t ibus_channels[14] = {
  1500, 1500, 1000, 1500,
  1000, 2000,
  1000, 1000, 1500, 1500, 1500, 1500, 1500, 1500
};
unsigned long last_ibus_time = 0;

// -------------------- Control State --------------------
int  manual_throttle = 1500;
int  servo_angle     = 0;    // 0–70 度，L1=0° R1=70°

// ─── ESP32 速度模式定高 PID（從 sketch_althold_v6_velmode 移植）───
bool  alt_hold_pid     = false;
float received_vel_cmd = 0.0f;
float v_est            = 0.0f;
float integral_vel     = 0.0f;
float last_vel_error   = 0.0f;
float prev_cm_filt_pid = -1.0f;
float prev_cm_raw_pid  = -1.0f;
int   hover_estimate   = 1500;
unsigned long last_pid_time = 0;

const float Kp_vel           = 0.80f;
const float Ki_vel           = 1.20f;
const float Kd_vel           = 0.0f;
const float MAX_VEL_CMD      = 60.0f;
const float MAX_INTEGRAL_VEL = 220.0f;
const float MAX_THR_OFFSET   = 220.0f;
const float GAMMA            = 0.20f;
const float MAX_ALT_CM_PID   = 250.0f;
const int   MAX_THR_STEP_PID = 20;

// -------------------- Sensor State --------------------
bool  sensor_ok          = false;
bool  flow_ok            = false;
float filtered_mm        = -1.0f;
bool  filter_inited      = false;
bool  alt_reading_frozen = false;
int   freeze_count       = 0;
unsigned long last_valid_range_time = 0;
unsigned long last_alt_report       = 0;
unsigned long last_flow_time        = 0;
unsigned long last_of_report        = 0;

int32_t accum_dx = 0;
int32_t accum_dy = 0;

// PMW3901 watchdog：偵測感測器卡死（持續全零）並自動重初始化
int           flow_zero_count      = 0;
const int     FLOW_REINIT_THRESH   = 300;  // 300 × 20 ms = 6 s 全零 → 重初始化
bool          flow_reinit_pending  = false;

// -------------------- Filter --------------------
const float ALPHA = 0.35f;

const unsigned long ALT_REPORT_INTERVAL_MS  = 200;
const unsigned long IBUS_SEND_INTERVAL_MS   = 20;
const unsigned long SENSOR_FRESH_TIMEOUT_MS = 200;
const unsigned long FLOW_INTERVAL_MS        = 20;   // MSP2 optical flow rate
const unsigned long OF_REPORT_INTERVAL_MS   = 40;   // BT OF telemetry rate (25 Hz)


// -------------------- Helpers --------------------
bool resetFlowSensor() {
    digitalWrite(FLOW_RST_PIN, LOW);
    delay(10);
    digitalWrite(FLOW_RST_PIN, HIGH);
    delay(50);  // PMW3901 開機穩定時間
    return flowSensor.begin();
}

void updateAltHold(float filt_mm) {
    unsigned long now = millis();
    float dt = (now - last_pid_time) / 1000.0f;
    if (dt < 0.02f) return;
    last_pid_time = now;

    if (filt_mm < 0) return;
    float current_cm = filt_mm / 10.0f;
    if (current_cm > MAX_ALT_CM_PID) return;

    if (prev_cm_raw_pid > 0 && fabs(current_cm - prev_cm_raw_pid) > 15.0f) {
        prev_cm_raw_pid  = current_cm;
        prev_cm_filt_pid = current_cm;
        v_est = 0;
        return;
    }
    prev_cm_raw_pid = current_cm;

    float safe_vel_cmd = constrain(received_vel_cmd, -MAX_VEL_CMD, MAX_VEL_CMD);

    if (prev_cm_filt_pid < 0) prev_cm_filt_pid = current_cm;
    float v_raw = constrain((current_cm - prev_cm_filt_pid) / dt, -100.0f, 100.0f);
    v_est = GAMMA * v_raw + (1.0f - GAMMA) * v_est;
    prev_cm_filt_pid = current_cm;

    float vel_error = safe_vel_cmd - v_est;
    float d_vel = (vel_error - last_vel_error) / dt;
    last_vel_error = vel_error;

    float tentative = Kp_vel * vel_error + Ki_vel * integral_vel + Kd_vel * d_vel;
    if (fabs(tentative) < MAX_THR_OFFSET * 0.85f) {
        integral_vel += vel_error * dt;
    } else if (vel_error * integral_vel > 0) {
        integral_vel *= 0.92f;
    }
    integral_vel = constrain(integral_vel, -MAX_INTEGRAL_VEL, MAX_INTEGRAL_VEL);

    float thr_offset = Kp_vel * vel_error + Ki_vel * integral_vel + Kd_vel * d_vel;
    thr_offset = constrain(thr_offset, -MAX_THR_OFFSET, MAX_THR_OFFSET);

    int desired = constrain(hover_estimate + (int)thr_offset, 1100, 1800);
    ibus_channels[2] = constrain(desired,
                                 (int)ibus_channels[2] - MAX_THR_STEP_PID,
                                 (int)ibus_channels[2] + MAX_THR_STEP_PID);
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

// -------------------- Setup --------------------
void setup() {
    Serial.begin(115200);
    delay(200);

    Serial2.begin(115200, SERIAL_8N1, IBUS_RX_PIN, IBUS_TX_PIN);

    SerialBT.begin("ESP32_Drone_Hub");
    Serial.println("ESP32 Bluetooth Started!");

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    SPI.begin();  // MOSI=23 MISO=19 SCK=18

    ESP32PWM::allocateTimer(0);
    gripper_servo.setPeriodHertz(50);
    gripper_servo.attach(SERVO_PIN, 500, 2400);
    gripper_servo.write(0);   // 上電歸零

    // VL53L1X
    distSensor.setTimeout(500);
    if (distSensor.init()) {
        distSensor.setDistanceMode(VL53L1X::Medium);
        distSensor.setMeasurementTimingBudget(33000);
        distSensor.startContinuous(33);
        sensor_ok = true;
        Serial.println("VL53L1X Init OK");
    } else {
        Serial.println("VL53L1X Init FAILED – check SDA/SCL wiring");
    }

    // PMW3901
    pinMode(FLOW_RST_PIN, OUTPUT);
    digitalWrite(FLOW_RST_PIN, HIGH);
    delay(10);
    if (resetFlowSensor()) {
        flow_ok = true;
        Serial.println("PMW3901 Init OK");
    } else {
        Serial.println("PMW3901 Init FAILED – check MOSI/MISO/SCK/CS wiring");
    }

    last_flow_time  = millis();
    last_of_report  = millis();
}

// -------------------- Loop --------------------
unsigned long last_thr_query_t       = 0;
const unsigned long THR_QUERY_INTERVAL_MS = 200;

void loop() {

    // ── 定高中週期性回報 ESP32 PID 油門 ──
    if (alt_hold_pid && millis() - last_thr_query_t >= THR_QUERY_INTERVAL_MS) {
        char hbuf[16];
        snprintf(hbuf, sizeof(hbuf), "THR:%d\n", (int)ibus_channels[2]);
        SerialBT.print(hbuf);
        last_thr_query_t = millis();
    }

    // ── BT Packet Parsing ──
    while (SerialBT.available()) {
        if (SerialBT.peek() != 'S') { SerialBT.read(); continue; }
        if (SerialBT.available() < 10) break;

        SerialBT.read();
        uint8_t buf[9];
        for (int i = 0; i < 9; i++) buf[i] = SerialBT.read();

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
            ibus_channels[6] = 1000;  // CH7：不使用 iNav ALTHOLD
            ibus_channels[7] = 1000;  // CH8：不使用 iNav POSHOLD

            if (!alt_hold_pid) {
                // 手動模式：Python 直接控制油門
                manual_throttle  = map(thr_val, 0, 255, 1000, 2000);
                ibus_channels[2] = manual_throttle;
            } else {
                // 定高模式：ESP32 PID 控制油門，接收速度指令
                received_vel_cmd = constrain((float)(alt_val - 128), -MAX_VEL_CMD, MAX_VEL_CMD);
            }

            // 切入定高
            if (ah_val == 1 && !alt_hold_pid && sensor_ok) {
                integral_vel     = 0;
                last_vel_error   = 0;
                v_est            = 0;
                prev_cm_filt_pid = -1;
                prev_cm_raw_pid  = -1;
                received_vel_cmd = 0;
                last_pid_time    = millis();
                hover_estimate   = (manual_throttle >= 1250) ? manual_throttle : 1500;
                alt_hold_pid     = true;
                SerialBT.print("AH:1\n");
            }
            // 退出定高
            if (ah_val == 0 && alt_hold_pid) {
                int exit_thr     = ibus_channels[2];
                alt_hold_pid     = false;
                manual_throttle  = exit_thr;
                ibus_channels[2] = exit_thr;
                char tbuf[16];
                snprintf(tbuf, sizeof(tbuf), "T:%d\n", exit_thr);
                SerialBT.print(tbuf);
                SerialBT.print("AH:0\n");
            }

            // L1(g_val=1)→0°, R1(g_val=2)→70°, 其他→保持
            if      (g_val == 1) servo_angle = 0;
            else if (g_val == 2) servo_angle = 70;
            gripper_servo.write(servo_angle);
        }
    }

    // ── VL53L1X Read（僅 ESP32 內部定高用，不再送 FC）──
    if (sensor_ok && distSensor.dataReady()) {
        int mm = distSensor.read(false);
        bool valid = !distSensor.timeoutOccurred() && mm > 0 && mm < 3000;

        if (valid) {
            last_valid_range_time = millis();
            float new_filt = (float)mm;
            bool rejected  = false;

            if (filter_inited && fabs(new_filt - filtered_mm) > 200.0f) {
                new_filt = filtered_mm;
                rejected = true;
            }

            bool skip_ema = false;
            if (rejected) {
                if (++freeze_count >= 5) {
                    if (!alt_reading_frozen) {
                        alt_reading_frozen = true;
                        SerialBT.print("F:1\n");
                        if (alt_hold_pid) {
                            int exit_thr     = (int)ibus_channels[2];
                            alt_hold_pid     = false;
                            manual_throttle  = exit_thr;
                            ibus_channels[2] = exit_thr;
                            char tbuf[16];
                            snprintf(tbuf, sizeof(tbuf), "T:%d\n", exit_thr);
                            SerialBT.print(tbuf);
                        }
                    }
                    filtered_mm   = (float)mm;
                    filter_inited = true;
                    freeze_count  = 0;
                    skip_ema      = true;
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
                filtered_mm = ALPHA * new_filt + (1.0f - ALPHA) * filtered_mm;
            }
        }

        Serial.printf("[RF] mm=%d valid=%d timeout=%d\n", mm, valid, distSensor.timeoutOccurred());
    }

    // ── PMW3901 Read（僅累積後經 BT 回傳 PC，不再送 FC）──
    if (flow_ok && millis() - last_flow_time >= FLOW_INTERVAL_MS) {
        unsigned long flow_now = millis();
        float dt_s = (flow_now - last_flow_time) / 1000.0f;
        last_flow_time = flow_now;

        int16_t dx = 0, dy = 0;
        flowSensor.readMotionCount(&dx, &dy);

        // Watchdog：連續全零超過門檻，認定感測器卡死，嘗試重初始化
        if (dx == 0 && dy == 0) {
            if (++flow_zero_count >= FLOW_REINIT_THRESH) {
                flow_zero_count = 0;
                Serial.println("PMW3901 watchdog: continuous zero detected, attempting HW reset...");
                SerialBT.print("OF:REINIT\n");
                flow_ok = resetFlowSensor();  // 硬體 RST + SPI 重初始化
                if (flow_ok) {
                    SerialBT.print("OF:REINIT_OK\n");
                    Serial.println("PMW3901 HW reset + re-init OK");
                } else {
                    SerialBT.print("OF:REINIT_FAIL\n");
                    Serial.println("PMW3901 HW reset + re-init FAILED");
                }
            }
        } else {
            flow_zero_count = 0;
        }

        accum_dx += dx;
        accum_dy += dy;
    }

    // ── ESP32 定高 PID ──
    if (alt_hold_pid && sensor_ok) {
        updateAltHold(filtered_mm);
    }

    // ── BT Altitude Telemetry ──
    if (sensor_ok && millis() - last_alt_report >= ALT_REPORT_INTERVAL_MS && filtered_mm > 0.0f) {
        char dbuf[16];
        snprintf(dbuf, sizeof(dbuf), "D:%d\n", (int)filtered_mm);
        SerialBT.print(dbuf);
        last_alt_report = millis();
    }

    // ── BT Optical Flow Telemetry (25 Hz, accumulated) ──
    if (flow_ok && millis() - last_of_report >= OF_REPORT_INTERVAL_MS) {
        char ofbuf[24];
        snprintf(ofbuf, sizeof(ofbuf), "OF:%d,%d\n", (int)accum_dx, (int)accum_dy);
        SerialBT.print(ofbuf);
        accum_dx      = 0;
        accum_dy      = 0;
        last_of_report = millis();
    }

    // ── iBUS Output ──
    if (millis() - last_ibus_time >= IBUS_SEND_INTERVAL_MS) {
        sendIBUS();
        last_ibus_time = millis();
    }
}
