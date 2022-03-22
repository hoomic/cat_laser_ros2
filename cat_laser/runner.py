import rclpy
from rclpy.node import Node

from cat_laser_interfaces.msg import FloorMapMsg, PanTilt
from .floor_map import FloorMap

import numpy as np
import yaml

class EndPoint():
  def __init__(self, c1, c2):
    self.corner1 = c1
    self.corner2 = c2
    self.p1 = np.array([c1.x, c1.y])
    self.p2 = np.array([c2.x, c2.y])

  def mid(self):
    return (self.p1 + self.p2) / 2

  def interpolate(self, perc):
    return self.p1 * perc + self.p2 * (1. - perc)

  def set_interpolation_inset(self, inset):
    p1 = np.array([self.corner1.x, self.corner1.y])
    p2 = np.array([self.corner2.x, self.corner2.y])
    self.p1 = p1 * (1. - inset) + p2 * inset
    self.p2 = p2 * (1. - inset) + p1 * inset

  def get_line(self, dest):
    return dest.p2 - self.mid()

  def length(self):
    diff = self.p2 - self.p1
    return np.sqrt(diff[0]**2 + diff[1]**2)

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

    self.endpoint_idx = 1
    self.wiggle = True
    self.counter = 0

    self.timer = self.create_timer(1./self.config['update_rate'], self.run)

  def process_floor_map(self, msg):
    if self.floor_initialized:
      return
    self.floor_map = FloorMap.from_msg(msg)

    #find longest_axis
    mids = self.floor_map.get_midpoints()
    n = len(mids)
    greatest_dist = 0
    for i, (x1, y1) in enumerate(mids):
      for j, (x2, y2) in enumerate(mids):
        dist = np.sqrt((x1 - x2)**2 + (y1 - y2)**2)
        if dist > greatest_dist:
          greatest_dist = dist
          self.endpoints = [
              EndPoint(self.floor_map.corners[i], self.floor_map.corners[(i+1)%n])
            , EndPoint(self.floor_map.corners[j], self.floor_map.corners[(j+1)%n])
          ]
    for e in self.endpoints:
      e.set_interpolation_inset(self.config['wiggle']['inset'])
    self.publish_coordinate(self.endpoints[self.endpoint_idx].interpolate(0))

    self.floor_initialized = True

  def run(self):
    if not self.floor_initialized:
      return    
    if self.wiggle:
      endpoint = self.endpoints[self.endpoint_idx]
      n = self.config['wiggle']['interpolation_steps']
      perc = self.counter % (2*n)
      if perc > n:
        perc = (2 * n - perc) / n
      else:
        perc /= n
      new_point = endpoint.interpolate(perc)
      self.publish_coordinate(new_point)
      self.counter += 1
      if self.counter >= self.config['wiggle']['duration']:
        self.get_logger().info("Switching from wiggle to running")
        self.counter = 0
        self.wiggle = False
        # change the target endpoint to the other end of the room
        self.endpoint_idx = (self.endpoint_idx + 1) % 2
        destination = self.endpoints[self.endpoint_idx]
        # calculate the line between the two endpoints
        self.line = endpoint.get_line(destination)
        # calculate the line perpendicular to the direction of travel for the sine wave
        self.perp_line = np.array([-self.line[1], self.line[0]])
        self.perp_line /= np.linalg.norm(self.perp_line)
        self.perp_line *= destination.length()
    else:
      T = self.config['running']['duration']
      perc_through = self.counter / T
      destination = self.endpoints[self.endpoint_idx]
      new_point = destination.p2 - (1. - perc_through ** self.config['running']['accel_power']) * self.line
      phase = self.counter * 2 * np.pi * self.config['running']['oscillations'] / T
      sine = np.sin(phase) * self.config['running']['sin_amplitude']
      new_point += (sine * self.perp_line)
      self.publish_coordinate(new_point)
      self.counter += 1
      if self.counter > T:
        self.get_logger().info("Switching from running to wiggle")
        self.counter = 0
        self.wiggle = True

  def publish_coordinate(self, pt):
    self.get_logger().info("Publishing coordinate {}".format(pt))
    pan, tilt = self.floor_map.get_pan_tilt(*pt)
    msg = PanTilt()
    msg.pan = pan
    msg.tilt = tilt
    self.movement_pub_.publish(msg)
    self.last_coordinate = pt

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