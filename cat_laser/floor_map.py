import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3

import cv2
import numpy as np
from cv_bridge import CvBridge

from enum import Enum

import time

bridge = CvBridge()

class CalibrationState(Enum):
  CALIBRATE_LASER = 1
  CALIBRATE_TILT = 2
  FIND_FLOOR = 3

class Corner():
  def __init__(self, x, y, pan, tilt):
    self.x = x
    self.y = y
    self.pan = pan
    self.tilt = tilt

message_map = {
  CalibrationState.CALIBRATE_LASER : [
    "Right click on the point in the image that contains the laser",
    "Move the camera by left clicking the image",
    None],
  CalibrationState.CALIBRATE_TILT : [
    "Left click point in image until laser points straight down",
    "Then right click to confirm",
    "Middle click to go back to previous calibration state"
  ],
  CalibrationState.FIND_FLOOR : [
    "Guide laser to corners of the room by clicking on them",
    "Right click when you have found all the corners",
    "Middle click to go back to previous calibration state"
  ]
}

class FloorMap(Node):
  def __init__(self):
    super().__init__('floor_map')

    self.frame_subscriber_ = self.create_subscription(
      Image
      , '/camera/video'
      , self.process_frame
      , 1) #set to 1 so that we discard all but the latest frame

    self.state_subscriber_ = self.create_subscription(
      Vector3
      , '/cat_laser/state'
      , self.process_state
      , 10
    )

    self.movement_pub_ = self.create_publisher(Vector3, '/cat_laser/movement', 5)

    self.laser_pub_ = self.create_publisher(Vector3, '/cat_laser/laser_coordinates', 5)

    self.state = CalibrationState.CALIBRATE_LASER
    #self.message = "Click point in image that points straight down"
    self.height = 90 # hard coded now to the height of my setup, but should be a user input

    self.corners = []

    self.laser_coordinates = None

    self.display_factor = 4

  def process_frame(self, msg):
    frame = bridge.imgmsg_to_cv2(msg)
    frame = cv2.resize(frame, (frame.shape[1] * self.display_factor, frame.shape[0] * self.display_factor))

    cv2.namedWindow('Floor Map Calibration')

    cv2.setMouseCallback('Floor Map Calibration', self.click_event)

    cy, cx = frame.shape[0] // 2, frame.shape[1] // 2
    

    self.center = (cx, cy)
    if self.laser_coordinates is None:
      self.laser_coordinates = self.center

    cv2.circle(frame, self.laser_coordinates, 5, (0, 0, 255), 1)

    m1, m2, m3 = message_map[self.state]

    cv2.putText(
      frame
      , m1
      , (10, frame.shape[0] - 30 * self.display_factor)
      , cv2.FONT_HERSHEY_SIMPLEX
      , 0.25 * self.display_factor, (255,255,255), 1)
    if m2 is not None:
      cv2.putText(
        frame
        , m2
        , (10, frame.shape[0] - 20 * self.display_factor)
        , cv2.FONT_HERSHEY_SIMPLEX
        , 0.25 * self.display_factor, (255,255,255), 1)
    if m3 is not None:
      cv2.putText(
        frame
        , m3
        , (10, frame.shape[0] - 10 * self.display_factor)
        , cv2.FONT_HERSHEY_SIMPLEX
        , 0.25 * self.display_factor, (255,255,255), 1)


    cv2.imshow('Floor Map Calibration', frame)
    cv2.waitKey(1000)

    # sleep to give the camera time to move
    time.sleep(1)

  def process_state(self, msg):
    self.servo_state = (msg.x, msg.y)

  def click_event(self, event, x, y, flags, params):
    cx, cy = self.laser_coordinates
    if event == cv2.EVENT_LBUTTONDOWN:
      if np.sqrt((cx - x)**2 + (cy - y)**2) < 10:
        if self.state == CalibrationState.FIND_FLOOR:
          self.add_corner(x, y)
      else:
        msg = Vector3()
        msg.x = float(cx - x) * 0.001 / self.display_factor
        msg.y = float(cy - y) * 0.001 / self.display_factor
        self.movement_pub_.publish(msg)
    elif event == cv2.EVENT_RBUTTONDOWN:
      if self.state == CalibrationState.CALIBRATE_LASER:
        self.laser_coordinates = (x, y)
        msg = Vector3()
        msg.x = float(x) / self.display_factor
        msg.y = float(y) / self.display_factor
        self.laser_pub_.publish(msg)
        self.state = CalibrationState.CALIBRATE_TILT
      elif self.state == CalibrationState.CALIBRATE_TILT:
        print("Setting base_tilt={}".format(self.servo_state[1]))
        self.base_tilt = self.servo_state[1]
        self.state = CalibrationState.FIND_FLOOR
      elif self.state == CalibrationState.FIND_FLOOR:
        self.finish()
    elif event == cv2.EVENT_MBUTTONDOWN:
      if self.state != CalibrationState.CALIBRATE_LASER:
        self.state = CalibrationState(self.state.value - 1)
  
  def add_corner(self, x, y):
    pan, tilt = self.servo_state
    floor_dist = self.height * np.tan(tilt - self.base_tilt)
    x = np.cos(pan) * floor_dist
    y = np.sin(pan) * floor_dist
    print("Adding Corner at x={} y={} pan={} tilt={}".format(x, y, pan, tilt))
    self.corners.append(Corner(x, y, pan, tilt))

  def finish(self):
    pass

def main(args=None):
  rclpy.init(args=args)

  floor_map = FloorMap()

  rclpy.spin(floor_map)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  floor_map.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()