from setuptools import setup
import os
from glob import glob

package_name = 'cat_laser'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
          glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'launch'),
          glob(os.path.join('launch', '*launch*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mhooi',
    maintainer_email='michael.r.hooi@gmail.com',
    description='Automated cat laser toy',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'video_streamer = cat_laser.video_streamer:main'
          , 'video_watcher = cat_laser.video_watcher:main'
          , 'motion_tracker = cat_laser.motion_tracker:main'
          , 'cat_laser = cat_laser.cat_laser:main'
          , 'floor_map = cat_laser.floor_map:main'
          , 'cat_detector = cat_laser.cat_detector:main'
          , 'cat_tracker = cat_laser.cat_tracker:main'
          , 'runner = cat_laser.runner:main'
          , 'template_recorder = cat_laser.template_recorder:main'
        ],
    },
)
