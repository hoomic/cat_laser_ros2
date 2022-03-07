# Summary

This repository contains a ROS2 package for controlling an automated laser pointer for cats! I run this project on a Raspberry Pi running Ubuntu Server 20.04 with ROS2 foxy installed. The laser and camera share a housing and are on a pan-tilt mount which is actuated by two SG-90 servos. 

# cat_laser.py

This file contains a ROS2 node for controlling the laser. It subscribes to a topic `/motion_tracker/movement` which gives it an (x, y) direction to move. Movements in the x direction cause the pan motor to move, and movements in the y direction cause the tilt motor to move.

`cat_laser.py` also contains a class `FloorMap` which currently has values hard-coded to my hallway. Eventually, this class will be made into its own node and put in its own file, and the floor plan will be estimated from the camera. 

The `CatLaser` node runs on the Pi, and it must be run as root in because the default `ubuntu` user doesn't have permissions to control the GPIO pins. 

# video_streamer.py

This file publishes data from the camera to the `/camera/video` and needs to be run on the Pi

# motion_tracker.py

This file contains the MotionTracker node which subscribes to the video topic and publishes to the `/motion_tracker/movement` topic. The motion tracking works by differencing images from consecutive frames and finding areas with significant amounts of motion. It then publishes the pixel disparity between the location of motion and the center of the camera so that the `CatLaser` node will move the laser towards the movement. This node can either be run on the Pi or another computer connected to the same network. If the computation in this node becomes too much for the Pi, then running it on a separate server may be necessary. 

# image_processing.py

This file is where all of the background OpenCV routines will be kept. `

## get_difference_image
 
 This function takes the absolute difference of consecutive frames after first lining up the frames to account for camera movement. Differencing only works to find motion when the camera is still, so to account for the camera motion, this function calculates SIFT features from both images and uses RANSAC to find the homography between them. Then it warps the perspective of the first image as if it were taken from the perspective of the second image. Then it takes absolute difference between the second image and the warped first image, and the bright pixels resulting from this difference are the objects that are moving in the frame. 
 
 # video_watcher.py
 
 Contains a node that subscribes to the `/camera/video` topic and displays it. This is meant to be run on a computer that has a monitor so that you can see what the camera sees in real time. 
 
 # TODOS

- [ ] Organize into launch files 
- [ ] Create a cat object detector to track the cat instead of naively following motions
- [ ] Create blog post describing the full setup including how to 3D print the files, how to set up ubuntu on the Pi, and how to wire the components
- [ ] Add IoT to turn it on/off: "Hey Google. Play with the cats."
