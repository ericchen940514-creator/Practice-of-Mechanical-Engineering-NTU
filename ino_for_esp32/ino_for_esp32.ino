// Version: 0.7.0 (MSP2 Sensor Hub)
// Changes from 0.6.1:
//   - VL53L0X → VL53L1X (Pololu, Short mode, up to ~1.3 m)
//   - Added PMW3901 optical flow via SPI (MOSI=23 MISO=19 SCK=18 CS=5)
//   - Serial1 (GPIO25) → FC UART1 RX as MSP2 sensor input
//     · MSP2_SENSOR_RANGEFINDER (0x1F01) at ~50 Hz
//     · MSP2_SENSOR_OPTIC_FLOW  (0x1F02) at ~50 Hz
//   - BT telemetry: D:mm (5 Hz) + OF:dx,dy (25 Hz) for PC logging
//
// Wiring:
//   VL53L1X  I2C   SDA=21  SCL=22
//   PMW3901  SPI   MOSI=23 MISO=19 SCK=18 CS=5
//   iBUS     UART2 TX=17   RX=16 (unused)  → FC UART6 RX
//   MSP2     UART1 TX=25                   → FC UART1 RX (iNAV: Sensor Input = MSP)
//   Servo    GPIO13
//
// iNAV setup required:
//   Ports → UART1 → Sensor Input = MSP,  baud = 115200

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
const int MSP_TX_PIN     = 25;
const int FLOW_RST_PIN   = 26;  // PMW3901 NRESET（低電位重置）

// -------------------- iBUS Channels --------------------
uint16_t ibus_channels[14] = {
  1500, 1500, 1000, 1500,
  1000, 2000,
  1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500
};
unsigned long last_ibus_time = 0;

// -------------------- Control State --------------------
bool  alt_hold_active  = false;
int   manual_throttle  = 1500;
float received_vel_cmd = 0.0f;

float v_est          = 0.0f;
float integral_vel   = 0.0f;
float last_vel_error = 0.0f;
float prev_cm_filt   = -1.0f;
float prev_cm_raw    = -1.0f;
int   hover_estimate = 1500;
unsigned long last_pid_time = 0;

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

// -------------------- PID Parameters --------------------
const float Kp_vel           = 0.80f;
const float Ki_vel           = 1.20f;
const float Kd_vel           = 0.0f;
const float MAX_VEL_CMD      = 60.0f;
const float MAX_INTEGRAL_VEL = 220.0f;
const float MAX_THR_OFFSET   = 220.0f;
const float ALPHA            = 0.35f;
const float GAMMA            = 0.20f;
const float MAX_ALT_CM       = 300.0f;
const int   MAX_THR_STEP     = 20;

const unsigned long ALT_REPORT_INTERVAL_MS  = 200;
const unsigned long IBUS_SEND_INTERVAL_MS   = 20;
const unsigned long SENSOR_FRESH_TIMEOUT_MS = 200;
const unsigned long FLOW_INTERVAL_MS        = 20;   // MSP2 optical flow rate
const unsigned long OF_REPORT_INTERVAL_MS   = 40;   // BT OF telemetry rate (25 Hz)

// -------------------- MSP2 Protocol --------------------
#define MSP2_SENSOR_RANGEFINDER  0x1F01
#define MSP2_SENSOR_OPTIC_FLOW   0x1F02

// 1 PMW3901 count ≈ 0.01 rad (≈ 0.57°); fine-tune with iNAV opflow_scale
const float COUNTS_TO_RAD = 0.01f;

static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a) {
    crc ^= a;
    for (int i = 0; i < 8; i++) {
        crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
    }
    return crc;
}

void sendMSP2(uint16_t func, const uint8_t *payload, uint16_t len) {
    uint8_t flag  = 0x00;
    uint8_t fn_lo = func & 0xFF;
    uint8_t fn_hi = (func >> 8) & 0xFF;
    uint8_t sz_lo = len & 0xFF;
    uint8_t sz_hi = (len >> 8) & 0xFF;

    uint8_t crc = 0;
    crc = crc8_dvb_s2(crc, flag);
    crc = crc8_dvb_s2(crc, fn_lo);
    crc = crc8_dvb_s2(crc, fn_hi);
    crc = crc8_dvb_s2(crc, sz_lo);
    crc = crc8_dvb_s2(crc, sz_hi);
    for (uint16_t i = 0; i < len; i++) {
        crc = crc8_dvb_s2(crc, payload[i]);
    }

    Serial1.write('$');
    Serial1.write('X');
    Serial1.write('<');
    Serial1.write(flag);
    Serial1.write(fn_lo);
    Serial1.write(fn_hi);
    Serial1.write(sz_lo);
    Serial1.write(sz_hi);
    Serial1.write(payload, len);
    Serial1.write(crc);
}

void sendMSP2Rangefinder(int dist_mm) {
    uint8_t quality = (dist_mm > 0 && dist_mm < 4000) ? 255 : 0;
    int32_t d = quality ? (int32_t)dist_mm : -1;
    uint8_t buf[5];
    buf[0] = quality;
    memcpy(&buf[1], &d, 4);
    sendMSP2(MSP2_SENSOR_RANGEFINDER, buf, 5);
}

void sendMSP2OptFlow(int16_t dx, int16_t dy, float dt_s) {
    float fx = (dt_s > 0.001f) ? dx * COUNTS_TO_RAD / dt_s : 0.0f;
    float fy = (dt_s > 0.001f) ? dy * COUNTS_TO_RAD / dt_s : 0.0f;
    float bx = 0.0f, by = 0.0f;
    uint8_t quality = 200;
    uint8_t buf[17];
    buf[0] = quality;
    memcpy(&buf[1],  &fx, 4);
    memcpy(&buf[5],  &fy, 4);
    memcpy(&buf[9],  &bx, 4);
    memcpy(&buf[13], &by, 4);
    sendMSP2(MSP2_SENSOR_OPTIC_FLOW, buf, 17);
}

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

bool sensorFreshEnough() {
    return filter_inited && (millis() - last_valid_range_time <= SENSOR_FRESH_TIMEOUT_MS);
}

bool resetFlowSensor() {
    digitalWrite(FLOW_RST_PIN, LOW);
    delay(10);
    digitalWrite(FLOW_RST_PIN, HIGH);
    delay(50);  // PMW3901 開機穩定時間
    return flowSensor.begin();
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

    if (!sensorFreshEnough()) {
        int fallback_thr = manual_throttle;
        alt_hold_active  = false;
        ibus_channels[2] = fallback_thr;
        manual_throttle  = fallback_thr;
        resetAltHoldController();
        SerialBT.print("F:2\n");
        char tbuf[16];
        snprintf(tbuf, sizeof(tbuf), "T:%d\n", fallback_thr);
        SerialBT.print(tbuf);
        Serial.println("Alt Hold disabled: sensor data stale");
        return;
    }

    if (filt_mm < 0.0f) return;

    float current_cm   = filt_mm / 10.0f;
    float safe_vel_cmd = received_vel_cmd;
    if (current_cm > MAX_ALT_CM) safe_vel_cmd = min(safe_vel_cmd, -5.0f);

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

    float vel_error = safe_vel_cmd - v_est;
    float d_vel     = (vel_error - last_vel_error) / dt;
    last_vel_error  = vel_error;

    float tentative_offset = Kp_vel * vel_error + Ki_vel * integral_vel + Kd_vel * d_vel;
    if (fabs(tentative_offset) < MAX_THR_OFFSET * 0.85f) {
        integral_vel += vel_error * dt;
    } else if (vel_error * integral_vel > 0.0f) {
        integral_vel *= 0.92f;
    }
    integral_vel = constrain(integral_vel, -MAX_INTEGRAL_VEL, MAX_INTEGRAL_VEL);

    float thr_offset = Kp_vel * vel_error + Ki_vel * integral_vel + Kd_vel * d_vel;
    thr_offset = constrain(thr_offset, -MAX_THR_OFFSET, MAX_THR_OFFSET);

    int desired = constrain(hover_estimate + (int)thr_offset, 1100, 1750);
    ibus_channels[2] = constrain(desired,
        (int)ibus_channels[2] - MAX_THR_STEP,
        (int)ibus_channels[2] + MAX_THR_STEP);
}

// -------------------- Setup --------------------
void setup() {
    Serial.begin(115200);
    delay(200);

    Serial2.begin(115200, SERIAL_8N1, IBUS_RX_PIN, IBUS_TX_PIN);
    Serial1.begin(115200, SERIAL_8N1, -1, MSP_TX_PIN);

    SerialBT.begin("ESP32_Drone_Hub");
    Serial.println("ESP32 Bluetooth Started!");

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    SPI.begin();  // MOSI=23 MISO=19 SCK=18

    ESP32PWM::allocateTimer(0);
    gripper_servo.setPeriodHertz(50);
    gripper_servo.attach(SERVO_PIN, 500, 2400);

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
void loop() {

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
            ibus_channels[6] = 2000;   // CH7 NAV POSHOLD 永遠開

            gripper_servo.writeMicroseconds(map(g_val, 0, 255, 1000, 2000));

            if (ah_val == 0) {
                manual_throttle = map(thr_val, 0, 255, 1000, 2000);
            }

            if (alt_hold_active) {
                received_vel_cmd = constrain((float)(alt_val - 128), -MAX_VEL_CMD, MAX_VEL_CMD);
            }

            if (ah_val == 1 && !alt_hold_active) {
                if (arm_val > 127 && sensor_ok && sensorFreshEnough()) {
                    resetAltHoldController();
                    hover_estimate  = (manual_throttle >= 1250) ? manual_throttle : 1500;
                    alt_hold_active = true;
                    Serial.println("Alt Hold ON");
                } else {
                    Serial.println("Alt Hold request rejected: not armed or sensor not ready");
                }
            } else if (ah_val == 2 && alt_hold_active) {
                hover_estimate = ibus_channels[2];
                resetAltHoldController();
                Serial.println("Alt Hold recentered");
            } else if (ah_val == 0 && alt_hold_active) {
                exitAltHoldToManual(ibus_channels[2]);
            } else if (ah_val == 0 && !alt_hold_active) {
                ibus_channels[2] = manual_throttle;
            }
        }
    }

    // ── VL53L1X Read + MSP2 Rangefinder ──
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
                filtered_mm = ALPHA * new_filt + (1.0f - ALPHA) * filtered_mm;
            }
        }

        sendMSP2Rangefinder(valid ? mm : 0);
        Serial.printf("[RF] mm=%d valid=%d timeout=%d\n", mm, valid, distSensor.timeoutOccurred());
    }

    // ── PMW3901 Read + MSP2 Optical Flow ──
    if (flow_ok && millis() - last_flow_time >= FLOW_INTERVAL_MS) {
        float dt_s = (millis() - last_flow_time) / 1000.0f;
        last_flow_time = millis();

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

        sendMSP2OptFlow(dx, dy, dt_s);

        accum_dx += dx;
        accum_dy += dy;
    }

    // ── Alt Hold Controller ──
    if (alt_hold_active) {
        updateAltHold(filtered_mm);
    }

    // ── BT Altitude Telemetry ──
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
