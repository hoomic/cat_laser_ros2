import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge

bridge = CvBridge()

class VideoWatcher(Node):
  def __init__(self, src=0):
    super().__init__('video_watcher')

    self.frame_subscriber_ = self.create_subscription(
      Image
      , '/camera/video'
      , self.display
      , 10)

  def display(self, msg):
    frame = bridge.imgmsg_to_cv2(msg)
    cv2.imshow('cat_laser', frame)
    cv2.waitKey(1)

def main(args=None):
  rclpy.init(args=args)

  video_watcher = VideoWatcher()

  rclpy.spin(video_watcher)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  video_watcher.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()