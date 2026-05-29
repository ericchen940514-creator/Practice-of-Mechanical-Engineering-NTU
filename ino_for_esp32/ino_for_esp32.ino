// Version: 0.7.0 (MSP2 Sensor Hub)
// Changes from 0.6.1:
//   - VL53L0X → VL53L1X (Pololu, Medium mode, up to ~3.0 m)
//   - Added PMW3901 optical flow via SPI (MOSI=23 MISO=19 SCK=18 CS=5)
//   - Serial1 (GPIO25) → FC UART1 RX as MSP2 sensor input
//     · MSP2_SENSOR_RANGEFINDER (0x1F01) at ~50 Hz
//     · MSP2_SENSOR_OPTIC_FLOW  (0x1F02) at ~50 Hz
//   - BT telemetry: D:mm (5 Hz) + OF:dx,dy (25 Hz) for PC logging
//
// Wiring:
//   VL53L1X  I2C   SDA=21  SCL=22
//   PMW3901  SPI   MOSI=23 MISO=19 SCK=18 CS=5
//   iBUS     UART2 TX=17   RX=16 (unused)  → FC UART1 RX
//   MSP2     UART1 TX=25   RX=34           ↔ FC UART3 (iNAV: Sensor Input = MSP)
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
const int MSP_TX_PIN     = 25;
const int MSP_RX_PIN     = 34;  // FC UART1 TX → ESP32（input-only GPIO）
const int FLOW_RST_PIN   = 26;  // PMW3901 NRESET（低電位重置）

// -------------------- iBUS Channels --------------------
uint16_t ibus_channels[14] = {
  1500, 1500, 1000, 1500,
  1000, 2000,
  1000, 1000, 1500, 1500, 1500, 1500, 1500, 1500
};
unsigned long last_ibus_time = 0;

// -------------------- Control State --------------------
bool alt_hold_ch7 = false;   // CH7 狀態（由 ah_val 控制）
int  prev_ah_val  = 0;

// 退出定高時先同步油門再切 CH7/CH8
enum SyncState { SYNC_IDLE, SYNC_WAITING };
SyncState     sync_state      = SYNC_IDLE;
unsigned long sync_request_t  = 0;

// 定點定高中持續回報 FC 油門
enum ThrQueryState { THR_IDLE, THR_WAITING };
ThrQueryState thr_state       = THR_IDLE;
unsigned long thr_request_t   = 0;
unsigned long last_thr_query_t = 0;
const unsigned long THR_QUERY_INTERVAL_MS = 200;

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
    if (fx >  20.0f) fx =  20.0f;
    if (fx < -20.0f) fx = -20.0f;
    if (fy >  20.0f) fy =  20.0f;
    if (fy < -20.0f) fy = -20.0f;
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

// -------------------- MSP v1 Motor Query --------------------
void requestMSPMotor() {
    // MSP v1 request: $ M < len=0 cmd=104 checksum=104
    uint8_t pkt[] = {'$', 'M', '<', 0x00, 104, 104};
    Serial1.write(pkt, sizeof(pkt));
}

// 嘗試從 Serial1 讀取 MSP_MOTOR 回應，成功時填入 motors[4] 並回傳 true
bool readMSPMotorResponse(uint16_t motors[4]) {
    static uint8_t  rbuf[32];
    static int      rpos     = 0;
    static unsigned long rts = 0;

    while (Serial1.available()) {
        uint8_t b = Serial1.read();
        if (rpos == 0) {
            if (b == '$') { rbuf[rpos++] = b; rts = millis(); }
        } else {
            rbuf[rpos++] = b;
            // 等齊 header 6 bytes: $ M > len cmd ...
            if (rpos >= 6) {
                if (rbuf[0]=='$' && rbuf[1]=='M' && rbuf[2]=='>') {
                    uint8_t len = rbuf[3];
                    uint8_t cmd = rbuf[4];
                    if (cmd == 104 && len == 16) {
                        int need = 5 + 16 + 1;  // $M>+len+cmd(5) + payload(16) + checksum(1)
                        if (rpos >= need) {
                            for (int i = 0; i < 4; i++)
                                motors[i] = rbuf[5 + i*2] | ((uint16_t)rbuf[6 + i*2] << 8);
                            rpos = 0;
                            return true;
                        }
                    } else {
                        rpos = 0;
                    }
                } else {
                    rpos = 0;
                }
            }
        }
        if (rpos > 0 && millis() - rts > 80) rpos = 0;
    }
    return false;
}

// -------------------- Helpers --------------------
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

// -------------------- Setup --------------------
void setup() {
    Serial.begin(115200);
    delay(200);

    Serial2.begin(115200, SERIAL_8N1, IBUS_RX_PIN, IBUS_TX_PIN);
    Serial1.begin(115200, SERIAL_8N1, MSP_RX_PIN, MSP_TX_PIN);

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

    // ── 退出定高油門同步 ──
    if (sync_state == SYNC_WAITING) {
        uint16_t motors[4];
        bool got     = readMSPMotorResponse(motors);
        bool timeout = (millis() - sync_request_t > 80);

        if (got || timeout) {
            int sync_thr = 1500;
            if (got) {
                long sum = 0; int cnt = 0;
                for (int i = 0; i < 4; i++) {
                    if (motors[i] >= 1000 && motors[i] <= 2000) { sum += motors[i]; cnt++; }
                }
                if (cnt > 0) sync_thr = (int)(sum / cnt);
            }
            ibus_channels[2] = sync_thr;
            ibus_channels[6] = 1000;  // ALTHOLD off
            ibus_channels[7] = 1000;  // POSHOLD off
            alt_hold_ch7     = false;
            thr_state        = THR_IDLE;
            char tbuf[16];
            snprintf(tbuf, sizeof(tbuf), "T:%d\n", sync_thr);
            SerialBT.print(tbuf);
            Serial.printf("Throttle sync: %d (%s)\n", sync_thr, got ? "MSP" : "timeout");
            sync_state = SYNC_IDLE;
        }
    }

    // ── 定點定高：週期性回報 FC 油門 ──
    if (alt_hold_ch7 && sync_state == SYNC_IDLE) {
        if (thr_state == THR_IDLE && millis() - last_thr_query_t >= THR_QUERY_INTERVAL_MS) {
            requestMSPMotor();
            thr_state      = THR_WAITING;
            thr_request_t  = millis();
        }
        if (thr_state == THR_WAITING) {
            uint16_t motors[4];
            bool got      = readMSPMotorResponse(motors);
            bool thr_tout = (millis() - thr_request_t > 100);
            if (got || thr_tout) {
                if (got) {
                    char dbg[48];
                    snprintf(dbg, sizeof(dbg), "DBG:M%d,%d,%d,%d\n",
                             motors[0], motors[1], motors[2], motors[3]);
                    SerialBT.print(dbg);
                    long sum = 0; int cnt = 0;
                    for (int i = 0; i < 4; i++) {
                        if (motors[i] >= 1000 && motors[i] <= 2000) { sum += motors[i]; cnt++; }
                    }
                    if (cnt > 0) {
                        char hbuf[16];
                        snprintf(hbuf, sizeof(hbuf), "THR:%d\n", (int)(sum / cnt));
                        SerialBT.print(hbuf);
                    }
                } else {
                    SerialBT.print("DBG:THR_TOUT\n");
                }
                thr_state       = THR_IDLE;
                last_thr_query_t = millis();
            }
        }
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
            if (sync_state == SYNC_IDLE)
                ibus_channels[2] = map(thr_val, 0, 255, 1000, 2000);
            ibus_channels[3] = map(y_val,   0, 255, 1000, 2000);
            ibus_channels[4] = map(arm_val, 0, 255, 1000, 2000);
            ibus_channels[5] = 2000;

            // 偵測 1→0（退出定點定高）：先發 MSP_MOTOR 請求，CH7/CH8 暫時保持高
            if (prev_ah_val == 1 && ah_val == 0 && sync_state == SYNC_IDLE) {
                // 清掉可能殘留的 THR query response，避免 SYNC 吃到過時資料
                while (Serial1.available()) Serial1.read();
                requestMSPMotor();
                sync_state       = SYNC_WAITING;
                sync_request_t   = millis();
                thr_state        = THR_IDLE;  // 取消定點輪詢
                ibus_channels[6] = 2000;      // 等同步完再切
                ibus_channels[7] = 2000;
            } else if (sync_state == SYNC_IDLE) {
                ibus_channels[6] = (ah_val == 1) ? 2000 : 1000;  // ALTHOLD
                ibus_channels[7] = (ah_val == 1) ? 2000 : 1000;  // POSHOLD
            }
            prev_ah_val  = ah_val;
            alt_hold_ch7 = (ah_val == 1);
            gripper_servo.writeMicroseconds(map(g_val, 0, 255, 1000, 2000));
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
