import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cat_laser_interfaces.msg import Template

import cv2
import numpy as np
from cv_bridge import CvBridge

import torch

from datetime import datetime
from collections import deque

bridge = CvBridge()

display_size = (320 * 4, 240 * 4)

def get_iou(obj1, obj2):
  x_left = max(obj1.xmin, obj2.xmin)
  y_top = max(obj1.ymin, obj2.ymin)
  x_right = min(obj1.xmax, obj2.xmax)
  y_bottom = max(obj1.ymax, obj2.ymax)

  if x_right < x_left or y_bottom < y_top:
    return 0.0

  A_inter = (x_right - x_left) * (y_bottom - y_top)
  A_1 = (obj1.xmax - obj1.xmin) * (obj1.ymax - obj1.ymin)
  A_2 = (obj2.xmax - obj2.xmin) * (obj2.ymax - obj2.ymin)

  iou = A_inter / (A_1 + A_2 - A_inter)
  return iou

class CatDetector(Node):
  def __init__(self):
    super().__init__('cat_detector')

    self.frame_subscriber_ = self.create_subscription(
      Image
      , '/camera/video'
      , self.process_frame
      , 1) #set to 1 so that we discard all but the latest frame

    self.template_pub_ = self.create_publisher(Template, '/cat_laser/template', 10)

    self.laser_subscriber_ = self.create_subscription(
      Vector3
      , 'cat_laser/laser_coordinates'
      , self.set_laser_coords
      , 10)

    self.model = torch.hub.load('ultralytics/yolov5', 'yolov5m')
    self.model.multi_label = True

    #find people and dogs asw well to disqualify them from being cats
    self.model.classes = [
      self.model.names.index('person')
      , self.model.names.index('dog')
      , self.model.names.index('cat')]
    #self.model.classes = [0]

    self.laser_coords = None

    self.frame_time = deque(maxlen=100)
    self.last_frame_time = datetime.now()

  def process_frame(self, msg):
    now = datetime.now()
    self.frame_time.append(now - self.last_frame_time)
    self.last_frame_time = now
    frame = bridge.imgmsg_to_cv2(msg)
    out = frame.copy()
    if self.laser_coords is None:
      self.laser_coords = (frame.shape[1] / 2, frame.shape[0] / 2)
    results = self.model(frame).pandas().xyxy[0]
    if len(results):
      results.reset_index()
      x_ave, y_ave, n = 0, 0, 0
      for i in range(len(results)):
        obj = results.iloc[i]

        # if this object is a dog or a person, or the cat confidencee is less than 0.5, continue
        if obj['name'] in ['dog', 'person'] or obj['confidence'] < 0.5:
          if obj['name'] == 'dog':
            cv2.rectangle(
            out
            , (int(obj.xmin), int(obj.ymin))
            , (int(obj.xmax), int(obj.ymax))
            , (0, 255, 0)
            , 1)
            cv2.putText(
              out
              , "{0:.1f}".format(obj.confidence * 100)
              , (int(obj.xmin), int(obj.ymax) - 1)
              , cv2.FONT_HERSHEY_SIMPLEX
              , 0.25, (0, 255, 0)
            )
          continue
        
        # if this object is better detected as a dog or a person, continue
        better_match = False
        for j in range(len(results)):
          if i == j:
            continue
          obj2 = results.iloc[j]
          iou = get_iou(obj, obj2)
          if iou > 0.9 and obj2['name'] in ['dog', 'person'] and obj2['confidence'] > obj['confidence']:
            better_match = True
            break
        if better_match:
          continue

        # We have a cat!
        cx = (obj.xmin + obj.xmax) / 2
        cy = (obj.ymin + obj.ymax) / 2

        x_ave += cx
        y_ave += cy
        n += 1

        cv2.rectangle(
          out
          , (int(obj.xmin), int(obj.ymin))
          , (int(obj.xmax), int(obj.ymax))
          , (0, 0, 255)
          , 1)
        cv2.putText(
          out
          , "{0:.1f}".format(obj.confidence * 100)
          , (int(obj.xmin), int(obj.ymin) - 1)
          , cv2.FONT_HERSHEY_SIMPLEX
          , 0.25, (0, 0, 255)
        )
        msg = Template()
        msg.image = bridge.cv2_to_imgmsg(frame[int(obj.ymin):int(obj.ymax), int(obj.xmin):int(obj.xmax)])
        msg.coordinates = [cx, cy]
        self.template_pub_.publish(msg)

    s = np.sum(self.frame_time).total_seconds()
    fps = len(self.frame_time) / s
    cv2.putText(
      out
      , "FPS: {:.1f}".format(fps)
      , (10, out.shape[0] - 10)
      , cv2.FONT_HERSHEY_SIMPLEX
      , 0.5, (255,255,255))

    out = cv2.resize(out, display_size)
    cv2.imshow('cat_detector', out)
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

    