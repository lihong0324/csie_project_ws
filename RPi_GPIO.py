import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(14, GPIO.OUT)
GPIO.setup(15, GPIO.OUT)

pwm1 = GPIO.PWM(14, 50)  # 50Hz for servo
pwm2 = GPIO.PWM(15, 50)

pwm1.start(0)
pwm2.start(0)

def set_angle(pwm, angle):
    duty = 2 + (angle / 18)
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.3)
    pwm.ChangeDutyCycle(0)

while True:
    set_angle(pwm1, 0)
    set_angle(pwm2, 0)
    time.sleep(1)
    set_angle(pwm1, 90)
    set_angle(pwm2, 90)
    time.sleep(1)