#include <Servo.h>
#include <SoftwareSerial.h>

SoftwareSerial BTSerial(10, 11); // RX接D10, TX接D11
Servo gripper_servo;

// 建立一個儲存 14 個通道的陣列，預設值皆為搖桿置中 (1500)
uint16_t ibus_channels[14] = {1500, 1500, 1000, 1500, 1000, 2000, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
unsigned long last_ibus_time = 0;

void setup() {
  // IBUS 的嚴格標準鮑率是 115200！
  Serial.begin(115200); 
  BTSerial.begin(9600); 
  
  gripper_servo.attach(6); // 夾爪依舊由 Arduino 獨立控制
}

// 核心函數：將陣列打包成 32 byte 的 IBUS 數位協議發送
void sendIBUS() {
  uint8_t packet[32];
  packet[0] = 0x20; // IBUS 封包開頭
  packet[1] = 0x40; // 通訊命令碼
  uint16_t checksum = 0xFFFF - packet[0] - packet[1];

  for (int i = 0; i < 14; i++) {
    packet[2 + i*2] = ibus_channels[i] & 0xFF;         // 低位元組
    packet[3 + i*2] = (ibus_channels[i] >> 8) & 0xFF;  // 高位元組
    checksum = checksum - packet[2 + i*2] - packet[3 + i*2];
  }

  packet[30] = checksum & 0xFF;
  packet[31] = (checksum >> 8) & 0xFF;

  Serial.write(packet, 32); // 從 TX 腳位精準射出給 F405
}

void loop() {
  // 1. 讀取藍牙送來的 7 個 Byte (S + 6個數據)
  while (BTSerial.available() >= 7) { 
    if (BTSerial.read() == 'S') { 
      int t_val = BTSerial.read(); // 油門
      int y_val = BTSerial.read(); // 偏航
      int p_val = BTSerial.read(); // 俯仰
      int r_val = BTSerial.read(); // 翻滾
      int g_val = BTSerial.read(); // 夾爪
      int arm_val = BTSerial.read(); // 解鎖狀態

      // 將 0~255 轉換成飛控聽得懂的 1000~2000 (AETR 順序)
      ibus_channels[0] = map(r_val, 0, 255, 1000, 2000); // CH1: 翻滾 Roll
      ibus_channels[1] = map(p_val, 0, 255, 1000, 2000); // CH2: 俯仰 Pitch
      ibus_channels[2] = map(t_val, 0, 255, 1000, 2000); // CH3: 油門 Throttle
      ibus_channels[3] = map(y_val, 0, 255, 1000, 2000); // CH4: 偏航 Yaw
      ibus_channels[4] = map(arm_val, 0, 255, 1000, 2000); // CH5(AUX1): 解鎖
      ibus_channels[5] = 2000; // CH6(AUX2): 自穩模式 (永遠鎖定在 2000)

      // 夾爪伺服馬達由 Arduino D6 直接驅動
      gripper_servo.writeMicroseconds(map(g_val, 0, 255, 1000, 2000));
    }
  }

  // 2. 飛控心跳機制：每 20 毫秒強制定時發送一次 IBUS 封包
  if (millis() - last_ibus_time >= 20) {
    sendIBUS();
    last_ibus_time = millis();
  }
}