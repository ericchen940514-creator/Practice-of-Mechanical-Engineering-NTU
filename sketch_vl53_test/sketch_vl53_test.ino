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
  sensor.init();
  sensor.setTimeout(500);
  sensor.setMeasurementTimingBudget(20000);
  sensor.startContinuous();
  Serial.println("就緒，開始回傳距離數值。");
}

void loop() {
  int mm = sensor.readRangeContinuousMillimeters();

  if (sensor.timeoutOccurred()) {
    BTSerial.println("TIMEOUT");
    Serial.println("TIMEOUT");
  } else {
    // 格式："D:123\n"，Python 端解析
    BTSerial.print("D:");
    BTSerial.println(mm);
    Serial.print("D:");
    Serial.println(mm);
  }

  delay(100);  // 10 Hz
}
