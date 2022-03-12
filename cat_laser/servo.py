import numpy as np

import gpiozero
from gpiozero.pins.pigpio import PiGPIOFactory

import time

factory = PiGPIOFactory()

class Servo(gpiozero.Servo):
  def __init__(self, pin, lo=0, hi=np.pi):
    super().__init__(self, pin, pin_factory=factory)
    self.lo = lo; self.hi = hi
    self.angle = 0

  def set_angle(self, angle, delay=0.1):
    angle = max(self.lo, min(self.hi, angle))
    self.value = angle / np.pi/2 - 1.0
    self.angle = angle

  def increment_angle(self, increment, delay=0.1):
    delay = max(0.01, delay)
    self.set_angle(self.angle + increment, delay)

  def stop(self):
    self.servo.stop()
