import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3

import numpy as np
import cv2
from cv_bridge import CvBridge

from .image_processing import get_difference_image

bridge = CvBridge()
kernel = cv2.getGaussianKernel(100, 50)

class MotionTracker(Node):
  def __init__(self):
    super().__init__('motion_tracker')

    self.frame_subscriber_ = self.create_subscription(
      Image
      , '/camera/video'
      , self.process_frame
      , 10)

    self.movement_pub_ = self.create_publisher(Vector3, '/cat_laser/movement', 10)
    self.laser_loc = None
    self.prev_gray = None

  def process_frame(self, msg):
    frame = bridge.imgmsg_to_cv2(msg)
    if self.laser_loc is None:
      red = frame[:,:,2]
      cv2.imshow('red', red)
      cv2.waitKey(1)
      y, x, _ = frame.shape
      self.center = (x//2, y//2)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if self.prev_gray is not None:
      # first get the differenced image with perspective shift
      diff = get_difference_image(self.prev_gray, gray)
      if diff is not None:
        # if there is a large difference, then calculate the vector from the pixel 
        # with the greatest difference to the laser and publish that vector
        if np.any(diff > 70):
          y, x = np.unravel_index(np.argmax(diff), diff.shape)
          cv2.circle(diff, (x, y), 20, 255, 2)
          pt = self.center if self.laser_loc is None else self.laser_loc
          msg = Vector3()
          msg.x = float(pt[0] - x)
          msg.y = float(pt[1] - y)
          self.movement_pub_.publish(msg)
        cv2.putText(diff, "max: {}".format(np.max(diff)), (0, 230), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)
        cv2.imshow('diff', diff)
        cv2.waitKey(1)
    self.prev_gray = gray
    return frame

def main(args=None):
  rclpy.init(args=args)

  motion_tracker = MotionTracker()

  rclpy.spin(motion_tracker)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  motion_tracker.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
