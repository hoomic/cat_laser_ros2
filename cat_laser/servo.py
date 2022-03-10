import numpy as np

import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

class Servo():
  def __init__(self, pin, lo=0, hi=np.pi):
    GPIO.setup(pin, GPIO.OUT)
    self.servo = GPIO.PWM(pin, 50)
    self.servo.start(0)
    self.lo = lo; self.hi = hi

  def set_angle(self, angle, delay=0.1):
    angle = max(self.lo, min(self.hi, angle))
    duty = angle * 10 / np.pi + 2. 
    self.servo.ChangeDutyCycle(duty)
    time.sleep(delay)
    self.servo.ChangeDutyCycle(0)
    self.angle = angle

  def increment_angle(self, increment, delay=0.1):
    delay = max(0.01, delay)
    self.set_angle(self.angle + increment, delay)

  def stop(self):
    self.servo.stop()
