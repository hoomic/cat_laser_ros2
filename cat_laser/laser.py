import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)

class Laser():
  def __init__(self, pin):
    GPIO.setup(pin, GPIO.OUT)
    self.pin = pin

    GPIO.output(pin, GPIO.LOW)
    self.on = False

  def turn_on(self):
    if not self.on:
      GPIO.output(self.pin, GPIO.HIGH)
    self.on = True

  def turn_off(self):
    if self.on:
      GPIO.output(self.pin, GPIO.LOW)
    self.on = False

  def toggle(self):
    if self.on:
      self.turn_off()
    else:
      self.turn_on()