from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='cat_laser',
      namespace='cat_laser',
      executable='video_watcher',
      name='video_watcher'
    ),
    Node(
      package='cat_laser',
      namespace='cat_laser',
      executable='floor_map',
      name='floor_map'
    )
  ])