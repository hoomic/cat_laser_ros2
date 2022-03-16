import rclpy
from rclpy.node import Node

from cat_laser_interfaces.msg import FloorMapMsg, PanTilt
from .floor_map import FloorMap

import numpy as np
import yaml

class Runner(Node):
  """Node for running the laser along the longest axis of the floor
  """
  def __init__(self):
    super().__init__('runner')
    self.config = yaml.safe_load(open('./install/cat_laser/share/cat_laser/config/runner.yaml', 'r'))

    self.floor_initialized = False

    self.floor_map_subscription_ = self.create_subscription(
      FloorMapMsg
      , '/cat_laser/floor_map'
      , self.process_floor_map
      , 10)

    self.movement_pub_ = self.create_publisher(PanTilt, '/cat_laser/movement', 10)

    self.endpoint_idx = 0
    self.wiggle = False
    self.wiggle_counter = True

    self.timer = self.create_timer(1./self.config['update_rate'], self.run)

  def process_floor_map(self, msg):
    if self.floor_initialized:
      return
    self.floor_map = FloorMap.from_msg(msg)

    #find longest_axis
    mids = self.floor_map.get_midpoints()
    greatest_dist = 0
    for i in range(len(mids)):
      x1, y1 = mids[i]
      for j in range(len(mids)):
        x2, y2 = mids[j]
        dist = np.sqrt((x1 - x2)**2 + (y1 - y2)**2)
        if dist > greatest_dist:
          greatest_dist = dist
          self.endpoints = [mids[i], mids[j]]

    pan_init, tilt_init = self.floor_map.get_pan_tilt(*self.endpoints[0])
    msg = PanTilt()
    msg.pan = pan_init
    msg.tilt = tilt_init
    self.movement_pub_.publish(msg)

    self.floor_initialized = True

  def run(self):
    if not self.floor_initialized:
      return      

def main(args=None):
  rclpy.init(args=args)

  runner = Runner()

  rclpy.spin(runner)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  runner.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()