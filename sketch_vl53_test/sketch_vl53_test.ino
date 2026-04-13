#include <Wire.h>
#include <VL53L0X.h>
#include <SoftwareSerial.h>

// BTSerial RX=D10, TX=D11（與其他 sketch 相同）
SoftwareSerial BTSerial(10, 11);
VL53L0X sensor;

void setup() {
  Serial.begin(115200);   // USB 除錯用
  BTSerial.begin(9600);   // 藍牙回傳
  Wire.begin();

  Serial.println("VL53L0X 初始化中...");
  if (!sensor.init()) {
    Serial.println("偵測不到感測器！請檢查 A4/A5 接線。");
    while (1) {}
  }
  sensor.setTimeout(500);

  // 長距模式設定（可達約 2m，速度降為 ~10Hz）
  sensor.setSignalRateLimit(0.1);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  sensor.setMeasurementTimingBudget(100000);  // 100ms

  sensor.startContinuous();
  Serial.println("就緒，開始回傳距離數值。");
}

void loop() {
  uint16_t mm = sensor.readRangeContinuousMillimeters();

  if (sensor.timeoutOccurred()) {
    BTSerial.println("TIMEOUT");
    Serial.println("TIMEOUT");
  } else {
    BTSerial.print("D:");
    BTSerial.println(mm);
    Serial.print("D:");
    Serial.println(mm);
  }
}
