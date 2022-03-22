import rclpy
from rclpy.node import Node

from cat_laser_interfaces.msg import Template

import cv2
import numpy as np
from cv_bridge import CvBridge

from datetime import datetime, timedelta
import os

bridge = CvBridge()

template_dir = './cat_laser/templates/unknown'
record_frequency = 60 #seconds between recording templates

if not os.path.exists(template_dir):
  os.makedirs(template_dir)

template_idx = 0
for f in os.listdir(template_dir):
  if os.path.isfile(os.path.join(template_dir, f)):
    t_idx = int(f.split('.')[0].lstrip('template'))
    if t_idx > template_idx:
      template_idx = t_idx + 1

class TemplateRecorder(Node):
  def __init__(self):
    super().__init__('template_recorder')

    self.template_subscriber_ = self.create_subscription(
      Template
      , '/cat_laser/template'
      , self.process_template
      , 10) #set to 1 so that we discard all but the latest frame

    self.template_pub_ = self.create_publisher(Template, '/cat_laser/template', 10)

    self.template_idx = 0
    for f in os.listdir(template_dir):
      if os.path.isfile(os.path.join(template_dir, f)):
        t_idx = int(f.split('.')[0].lstrip('template'))
        if t_idx > self.template_idx:
          self.template_idx = t_idx + 1
    self.get_logger().info("Setting template_idx={}".format(self.template_idx))

    self.last_record_time = datetime.now() - timedelta(seconds=record_frequency)

  def process_template(self, msg):
    now = datetime.now()
    if now > self.last_record_time + timedelta(seconds=record_frequency):
      image = bridge.imgmsg_to_cv2(msg.image)
      cv2.imwrite(template_dir + '/template{}.jpg'.format(self.template_idx), image)
      self.get_logger().info("Recording template at {}".format(now))
      self.template_idx += 1
      self.last_record_time = now

def main(args=None):
  rclpy.init(args=args)

  template_recorder = TemplateRecorder()

  rclpy.spin(template_recorder)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  template_recorder.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()