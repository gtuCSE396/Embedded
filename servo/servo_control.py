import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
GPIO.setup(18,GPIO.OUT)
GPIO.setup(12,GPIO.OUT)

motor1 = GPIO.PWM(18,50)
motor2 = GPIO.PWM(12,50)

def duty(angle):
    return(float(angle) / 18.0 + 2)

motor1.start(duty(0))
motor2.start(duty(0))

motor1.ChangeDutyCycle(duty(0))
motor2.ChangeDutyCycle(duty(0))
for i in range(0,20):
    #print ("angle:",i)
    motor1.ChangeDutyCycle(duty(0))
    motor2.ChangeDutyCycle(duty(0))
    time.sleep(0.2)
    motor1.ChangeDutyCycle(duty(50))
    motor2.ChangeDutyCycle(duty(50))
    time.sleep(0.2)
    
motor1.stop()
motor2.stop()
GPIO.cleanup()
