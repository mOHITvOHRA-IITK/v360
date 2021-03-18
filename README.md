# Visualthree60

**INTRODUCTION**
This repository has some codes for estimating human dimensions from single image. [I2L](https://github.com/mks0601/I2L-MeshNet_RELEASE), [RootNet](https://github.com/mks0601/3DMPPE_ROOTNET_RELEASE)



**Camera Calibration**
Camera calibration (webcam) for intrinsic parameters is included in this repo. This Package uses the Robot Operating System (ROS). To calibrate the webcam, run the file `webcam_topic_publish.cpp`. This file publishes the webcam images on the topic name `/webcam_images`. Now Run the command `rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/webcam_images`


**Repos Need to be tested**
[Self Correction for Human Parsing](https://github.com/PeikeLi/Self-Correction-Human-Parsing)
