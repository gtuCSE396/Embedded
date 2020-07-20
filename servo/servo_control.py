import RPi.GPIO as GPIO
import time

servoPIN = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)

p = GPIO.PWM(servoPIN, 50) # GPIO 18 for PWM with 50Hz
p.start(0) # Initialization

try:
  while True:
    p.ChangeDutyCycle(5)
    time.sleep(0.5)
    p.ChangeDutyCycle(15)
    time.sleep(0.5)
    p.ChangeDutyCycle(25)
    time.sleep(0.5)
    p.ChangeDutyCycle(40)
    time.sleep(0.5)
    p.ChangeDutyCycle(25)
    time.sleep(0.5)
    p.ChangeDutyCycle(15)
    time.sleep(0.5)
    p.ChangeDutyCycle(5)
    time.sleep(0.5)
    p.ChangeDutyCycle(0)
    time.sleep(0.5)
    
except KeyboardInterrupt:
  p.stop()
  GPIO.cleanup()