import pygame
import time

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("❌ 沒抓到手把！請檢查藍牙連接。")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"🎮 偵測到手把: {joystick.get_name()}")
print("👉 現在請按下 L1 和 R1，看看畫面上顯示 Button 幾號...")

try:
    while True:
        pygame.event.pump()
        # 掃描所有可能的按鍵 (通常手把有 14-16 個鍵)
        for i in range(joystick.get_numbuttons()):
            if joystick.get_button(i):
                print(f"🔥 偵測到按鍵編號: {i}")
        time.sleep(0.1)
except KeyboardInterrupt:
    pygame.quit()