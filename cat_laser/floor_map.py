import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cat_laser_interfaces.msg import PanTilt, FloorMapMsg

import cv2
import numpy as np
from cv_bridge import CvBridge

from enum import Enum

import time
import os

HOME = os.path.expanduser('~')

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

  def to_list(self):
    return [self.x, self.y, self.pan, self.tilt]

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

class FloorCalibration(Node):
  def __init__(self):
    super().__init__('floor_calibration')

    self.floor_map_publisher_ = self.create_publisher(FloorMapMsg, '/cat_laser/floor_map', 5)

    fm_file = HOME + '/cat_laser_floor_map/floor_map.txt'
    if os.path.exists(fm_file):
      fm_str = open(fm_file).read()
      self.floor_map = FloorMap.from_str(fm_str)
      self.get_logger().info("Floor map file found at {}. Publishing floor map ever 1s".format(fm_file))
      self.timer = self.create_timer(1., self.publish_floor_map)
    else:

      self.frame_subscriber_ = self.create_subscription(
        Image
        , '/camera/video'
        , self.process_frame
        , 1) #set to 1 so that we discard all but the latest frame

      self.state_subscriber_ = self.create_subscription(
        PanTilt
        , '/cat_laser/state'
        , self.process_state
        , 10
      )

      self.movement_pub_ = self.create_publisher(PanTilt, '/cat_laser/movement', 5)

      self.laser_pub_ = self.create_publisher(Vector3, '/cat_laser/laser_coordinates', 5)

      self.state = CalibrationState.CALIBRATE_LASER

      self.corners = []

      self.laser_coordinates = None

      self.display_factor = 4

  def process_frame(self, msg):
    frame = bridge.imgmsg_to_cv2(msg)
    frame = cv2.resize(frame, (frame.shape[1] * self.display_factor, frame.shape[0] * self.display_factor))

    cv2.namedWindow('Floor Map Calibration')

    cv2.setMouseCallback('Floor Map Calibration', self.click_event)
    
    if self.laser_coordinates is None:
      cx, cy = frame.shape[1] // 2, frame.shape[0] // 2
      self.laser_coordinates = (cx / self.display_factor, cy / self.display_factor)
    else:
      cx, cy = [int(c * self.display_factor) for c in self.laser_coordinates]

    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), 1)

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
    self.servo_state = (msg.pan, msg.tilt)

  def click_event(self, event, x, y, flags, params):
    if event == cv2.EVENT_LBUTTONDOWN:
      cx, cy = [l * self.display_factor for l in self.laser_coordinates]
      if np.sqrt((cx - x)**2 + (cy - y)**2) < 10:
        if self.state == CalibrationState.FIND_FLOOR:
          self.add_corner(x, y)
      else:
        msg = PanTilt()
        msg.pan = float(cx - x) * 0.001 / self.display_factor
        msg.tilt = float(cy - y) * 0.001 / self.display_factor
        msg.increment = True
        self.movement_pub_.publish(msg)
    elif event == cv2.EVENT_RBUTTONDOWN:
      if self.state == CalibrationState.CALIBRATE_LASER:
        x = float(x) / self.display_factor
        y = float(y) / self.display_factor
        self.laser_coordinates = (x, y)
        msg = Vector3()
        msg.x = x
        msg.y = y
        self.laser_pub_.publish(msg)
        self.state = CalibrationState.CALIBRATE_TILT
      elif self.state == CalibrationState.CALIBRATE_TILT:
        self.get_logger().info("Setting base_tilt={}".format(self.servo_state[1]))
        self.base_tilt = self.servo_state[1]
        self.state = CalibrationState.FIND_FLOOR
      elif self.state == CalibrationState.FIND_FLOOR:
        self.finish()
    elif event == cv2.EVENT_MBUTTONDOWN:
      if self.state != CalibrationState.CALIBRATE_LASER:
        self.state = CalibrationState(self.state.value - 1)
  
  def add_corner(self, x, y):
    pan, tilt = self.servo_state
    x = np.cos(pan) * np.tan(tilt - self.base_tilt)
    y = np.sin(pan) * np.tan(tilt - self.base_tilt)
    if not np.any([np.sqrt((x - c.x)**2 + (y - c.y)**2) < 0.1 for c in self.corners]):
      self.get_logger().info("Adding Corner at x={} y={} pan={} tilt={}".format(x, y, pan, tilt))
      self.corners.append(Corner(x, y, pan, tilt))

  def finish(self):
    self.frame_subscriber_.unregister()
    path = HOME + '/cat_laser_floor_map/'
    if not os.path.exists(path):
      os.makedirs(path)
    outfile = open(path + 'floor_map.txt', 'w')
    self.floor_map = FloorMap(self.base_tilt, self.laser_coordinates, self.corners)
    outfile.write(str(self.floor_map))
    outfile.close()
    self.get_logger().info("Calibration Complete!")
    cv2.destroyAllWindows()
    self.timer = self.create_timer(1., self.publish_floor_map)

  def publish_floor_map(self):
    self.floor_map_publisher_.publish(self.floor_map.to_msg())

class FloorMap():
  def __init__(self, base_tilt, laser_coordinates, corners):
    self.base_tilt = base_tilt
    self.laser_coordinates = laser_coordinates
    self.corners = corners

  def get_pan_tilt(self, x, y):
    """ Returns the pan and tilt angle for a given coordinate
    """
    pan = np.arctan2(y, x)
    tilt = np.arctan2(x, np.cos(pan))
    return pan, tilt

  def get_midpoints(self):
    """ Returns a list of the midpoints between adjacent corners
    """
    midpoints = []
    for i, c1 in enumerate(self.corners):
      c2 = self.corners[(i + 1) % len(self.corners)]
      midpoints.append(((c1.x + c2.x) / 2, (c1.y + c2.y) / 2))
    return midpoints

  @classmethod
  def from_msg(cls, msg):
    corners = []
    for i in range(len(msg.corners) // 4):
      corners.append(Corner(*msg.corners[i * 4: (i + 1) * 4]))
    return cls(msg.base_tilt, msg.laser_coordinates, corners)

  @classmethod
  def from_str(cls, fm_str):
    base_tilt, laser_coordinates, corners = fm_str.split(";")
    laser_coordinates = [float(c) for c in laser_coordinates.split(',')]
    corner_strs = corners.split(',')
    corners = []
    for i in range(len(corner_strs) // 4):
      corners.append(Corner(*[float(c) for c in corner_strs[i * 4: (i + 1) * 4]]))
    return cls(float(base_tilt), laser_coordinates, corners)

  def to_msg(self):
    msg = FloorMapMsg()
    msg.base_tilt = self.base_tilt
    msg.laser_coordinates = self.laser_coordinates
    corner_data = []
    for c in self.corners:
      corner_data.extend(c.to_list())
    msg.corners = corner_data
    return msg

  def __str__(self):
    corner_data = []
    for c in self.corners:
      corner_data.extend(c.to_list())
    corner_str = ','.join([str(d) for d in corner_data])
    return '{};{},{};{}'.format(self.base_tilt, *self.laser_coordinates, corner_str)

def main(args=None):
  rclpy.init(args=args)

  floor_calibration = FloorCalibration()

  rclpy.spin(floor_calibration)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  floor_calibration.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()