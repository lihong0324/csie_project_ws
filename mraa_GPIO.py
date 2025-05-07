#!/usr/bin/env python3
import mraa
import time

# === 依照你的接線修改這兩個數字 ===
SERVO1_PIN = 33   # 40‑pin #33 → GPIO33 → PWM0
SERVO2_PIN = 32   # 40‑pin #32 → GPIO32 → PWM1

# --- 初始化 PWM ---
pwm1 = mraa.Pwm(SERVO1_PIN)
pwm2 = mraa.Pwm(SERVO2_PIN)

for p in (pwm1, pwm2):
    p.period_us(20_000)   # 20 ms = 50 Hz
    p.enable(True)

# --- 角度函式 (0‑180°) ---
def set_angle(pwm, angle):
    # 1 ms (0°) → 2 ms (180°)
    pulse_us = 1_000 + (angle / 180) * 1_000
    duty = pulse_us / 20_000
    pwm.write(duty)

# --- 主迴圈 ---
try:
    while True:
        set_angle(pwm1, 0)
        set_angle(pwm2, 0)
        time.sleep(1)

        set_angle(pwm1, 90)
        set_angle(pwm2, 90)
        time.sleep(1)
except KeyboardInterrupt:
    pass
finally:
    pwm1.enable(False)
    pwm2.enable(False)