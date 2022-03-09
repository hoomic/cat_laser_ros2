import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3

import cv2
import numpy as np
from cv_bridge import CvBridge

import RPi.GPIO as GPIO

from .servo import Servo
from .laser import Laser

bridge = CvBridge()

laser_pin = 7
pan_pin = 11
tilt_pin = 12

class CatLaser(Node):
  def __init__(self, verbose=False):
    super().__init__('cat_laser')
    self.verbose = verbose

    self.laser = Laser(laser_pin)
    self.pan_servo = Servo(pan_pin, np.pi/6, np.pi * 5/6)
    self.tilt_servo = Servo(tilt_pin)

    self.floor = FloorMap()

    center = self.floor.get_center()
    self.laser.turn_on()
    self.set_point(center, 0.5)

    self.movement_sub_ = self.create_subscription(
      Vector3
      , 'cat_laser/movement'
      , self.process_movement
      , 10
      )

    self.state_publisher_ = self.create_publisher(
      Vector3
      , 'cat_laser/state'
      , 10
      )

    self.timer = self.create_timer(1./30, self.publish_state)

  def process_movement(self, msg):
    print("Processing movement x={} y={}".format(msg.x, msg.y))
    self.pan_servo.increment_angle(msg.x)
    self.tilt_servo.increment_angle(msg.y)

  def publish_state(self):
    msg = Vector3()
    try:
      msg.x = self.pan_servo.angle
      msg.y = self.tilt_servo.angle
      self.state_publisher_.publish(msg)
    except Exception as e:
      print("Warning the following exception was caught while trying\
       to publish the state:\n{} pan_servo_angle={} tilt_servo_angle={}".format(
         e, self.pan_servo.angle, self.tilt_servo.angle
       ))

  def set_point(self, pos, delay=0.01):
    self.set_angles(*self.floor.get_pitch_yaw(pos), delay)
    self.position = pos

  def set_angles(self, pitch, yaw, delay=0.01):
    self.tilt_servo.set_angle(pitch, delay)
    self.pan_servo.set_angle(yaw, delay)

  def close(self):
    self.laser.turn_off()
    center = self.floor.get_center()
    self.set_point(center, 0.5)
    self.pan_servo.stop()
    self.tilt_servo.stop()
    GPIO.cleanup()

class FloorMap():
  def __init__(self):
    # TODO calculate these from correspondence points or images
    # hard code to my hallway for now
    self.x_lo, self.x_hi = [-40, 7]
    self.y_lo, self.y_hi = [0, 217]
    self.z = 90

    self.pitch_offset = np.deg2rad(4)
    self.yaw_offset = np.deg2rad(10)

  def get_coords(self, pitch, yaw):
    pitch += self.pitch_offset
    yaw += self.yaw_offset

    x = np.sqrt((self.z * np.tan(pitch))**2 / (1 + np.tan(yaw)**2))
    y = np.tan(yaw) * x

    return x, y

  def get_pitch_yaw(self, coord):
    x, y = self.clip(coord)
    r = np.sqrt(x**2 + y**2)
    pitch = np.arctan2(r, self.z) - self.pitch_offset
    yaw = np.arctan2(y, x) - self.yaw_offset

    return pitch, yaw

  def get_center(self):
    return (self.x_lo + self.x_hi)/ 2, (self.y_lo + self.y_hi) / 2

  def get_bounds(self):
    return self.x_lo, self.x_hi, self.y_lo, self.y_hi

  def in_bounds(self, pos):
    x, y = pos
    return self.x_lo <= x <= self.x_hi and self.y_lo <= y <= self.y_hi

  def clip(self, pos):
    x, y = pos
    return max(self.x_lo + 1e-6, min(self.x_hi - 1e-6, x)), max(self.y_lo + 1e-6, min(self.y_hi - 1e-6, y))


def main(args=None):
  try:
    rclpy.init(args=args)

    cat_laser = CatLaser()

    rclpy.spin(cat_laser)

  finally:
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cat_laser.close()
    cat_laser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()