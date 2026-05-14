// Sensor diagnostic: VL53L1X (I2C) + PMW3901 (SPI)
// ESP32 pins:
//   I2C  SDA=21, SCL=22
//   SPI  MOSI=23, MISO=19, SCK=18, CS=5

#include <Wire.h>
#include <SPI.h>
#include <VL53L1X.h>             // Library: "VL53L1X" by Pololu
#include <Bitcraze_PMW3901.h>    // Library: "Bitcraze PMW3901"

#define PMW_CS_PIN 5

VL53L1X distSensor;
Bitcraze_PMW3901 flow(PMW_CS_PIN);

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Sensor Test Start ===");

  // --- VL53L1X ---
  Wire.begin(21, 22);
  distSensor.setTimeout(500);
  Serial.print("[VL53L1X] Init... ");
  if (!distSensor.init()) {
    Serial.println("FAIL - check SDA/SCL wiring or 3.3V power");
  } else {
    distSensor.setDistanceMode(VL53L1X::Short);  // Short = up to ~1.3m, more stable
    distSensor.setMeasurementTimingBudget(50000); // 50ms per reading
    distSensor.startContinuous(50);
    Serial.println("OK");
  }

  // --- PMW3901 ---
  pinMode(PMW_CS_PIN, OUTPUT);
  digitalWrite(PMW_CS_PIN, HIGH);
  Serial.print("[PMW3901] Init... ");
  if (!flow.begin()) {
    Serial.println("FAIL - check MOSI/MISO/SCK/CS wiring or 3.3V power");
  } else {
    // Read Product ID via SPI directly (reg 0x00, should return 0x49)
    SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));
    digitalWrite(PMW_CS_PIN, LOW);
    SPI.transfer(0x00 & 0x7F);  // read: MSB=0
    uint8_t pid = SPI.transfer(0x00);
    digitalWrite(PMW_CS_PIN, HIGH);
    SPI.endTransaction();

    Serial.print("OK  (Product ID = 0x");
    Serial.print(pid, HEX);
    if (pid == 0x49) Serial.println("  correct)");
    else Serial.println("  WRONG - SPI mis-wired or wrong CS pin)");
  }

  Serial.println("=== Loop start (115200 baud) ===");
}

void loop() {
  // VL53L1X reading
  int mm = distSensor.read(false);  // non-blocking
  if (!distSensor.timeoutOccurred()) {
    if (mm > 0 && mm < 4000) {
      Serial.print("[VL53L1X] ");
      Serial.print(mm);
      Serial.println(" mm");
    } else {
      Serial.println("[VL53L1X] out of range");
    }
  } else {
    Serial.println("[VL53L1X] timeout");
  }

  // PMW3901 reading
  int16_t dx = 0, dy = 0;
  flow.readMotionCount(&dx, &dy);
  Serial.print("[PMW3901] dx=");
  Serial.print(dx);
  Serial.print("  dy=");
  Serial.println(dy);

  delay(200);
}
