from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='cat_laser',
      namespace='cat_laser',
      executable='cat_laser',
      name='cat_laser'
    ),
    Node(
      package='cat_laser',
      namespace='cat_laser',
      executable='video_streamer',
      name='video_streamer'
    )
  ])