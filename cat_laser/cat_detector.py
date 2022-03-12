import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3

import cv2
import numpy as np
from cv_bridge import CvBridge

import torch

from datetime import datetime
from collections import deque

bridge = CvBridge()

display_size = (320 * 4, 240 * 4)

class CatDetector(Node):
  def __init__(self):
    super().__init__('cat_detector')

    self.frame_subscriber_ = self.create_subscription(
      Image
      , '/camera/video'
      , self.process_frame
      , 1) #set to 1 so that we discard all but the latest frame

    self.laser_subscriber_ = self.create_subscription(
      Vector3
      , 'cat_laser/laser_coordinates'
      , self.set_laser_coords
      , 10)

    self.movement_pub_ = self.create_publisher(Vector3, '/cat_laser/movement', 10)

    self.model = torch.hub.load('ultralytics/yolov5', 'yolov5l')
    self.model.multi_label = True

    self.model.classes = [self.model.names.index('cat')]
    #self.model.classes = [0]

    self.laser_coords = None

    self.frame_time = deque(maxlen=100)
    self.last_frame_time = datetime.now()

  def process_frame(self, msg):
    now = datetime.now()
    self.frame_time.append(now - self.last_frame_time)
    self.last_frame_time = now
    frame = bridge.imgmsg_to_cv2(msg)
    if self.laser_coords is None:
      self.laser_coords = (frame.shape[1] / 2, frame.shape[0] / 2)
    results = self.model(frame).pandas().xyxy[0]
    if len(results) and np.any([cat['confidence'] > 0.5 for i, cat in results.iterrows()]):
      results.reset_index()
      x_ave, y_ave, n = 0, 0, 0
      for i in range(len(results)):
        cat = results.iloc[i]
        cx = (cat.xmin + cat.xmax) / 2
        cy = (cat.ymin + cat.ymax) / 2

        x_ave += cx
        y_ave += cy
        n += 1

        cv2.rectangle(
          frame
          , (int(cat.xmin), int(cat.ymin))
          , (int(cat.xmax), int(cat.ymax))
          , (0, 0, 255)
          , 1)
        cv2.putText(
          frame
          , "{0:.1f}".format(cat.confidence * 100)
          , (int(cat.xmin), int(cat.ymin))
          , cv2.FONT_HERSHEY_SIMPLEX
          , 0.5, (0, 0, 255)
        )

      x_ave /= n
      y_ave /= n

      if np.sqrt((x_ave - self.laser_coords[0])**2 + (y_ave - self.laser_coords[1]) **2) > 20:
        msg = Vector3()
        msg.x = float(self.laser_coords[0] - x_ave) * 0.0005 
        msg.y = float(self.laser_coords[1] - y_ave) * 0.0005
        msg.z = float(0.01)
        self.movement_pub_.publish(msg)

    s = np.sum(self.frame_time).total_seconds()
    fps = len(self.frame_time) / s
    cv2.putText(
      frame
      , "FPS: {:.1f}".format(fps)
      , (10, frame.shape[0] - 10)
      , cv2.FONT_HERSHEY_SIMPLEX
      , 0.5, (255,255,255))

    frame = cv2.resize(frame, display_size)
    cv2.imshow('cat_detector', frame)
    cv2.waitKey(1)  

  def set_laser_coords(self, msg):
    self.get_logger().info("Laser coordinates set to: x={} y={}".format(msg.x, msg.y))
    self.laser_coords = (msg.x, msg.y)

def main(args=None):
  rclpy.init(args=args)

  cat_detector = CatDetector()

  rclpy.spin(cat_detector)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  cat_detector.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()

    