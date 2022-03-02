import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge

bridge = CvBridge()

class VideoStream(Node):
  def __init__(self, src=0):
    super().__init__('video_stream')

    self.frame_publisher_ = self.create_publisher(Image, '/camera/video', 10)

    fps = 24.

    self.timer = self.create_timer(1./fps, self.publish_frame)

    # initialize the video camera stream and read the first frame
    # from the stream
    self.stream = cv2.VideoCapture(src)
    self.stream.set(cv2.CV_CAP_PROP_FRAME_WIDTH,640)
    self.stream.set(cv2.CV_CAP_PROP_FRAME_HEIGHT,480)

  def publish_frame(self):
    ret, frame = self.stream.read()
    if ret:
      msg = bridge.cv2_to_imgmsg(frame, encoding="passthrough")
      self.frame_publisher_.publish(msg)


def main(args=None):
  rclpy.init(args=args)

  video_stream = VideoStream()

  rclpy.spin(video_stream)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  video_stream.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()