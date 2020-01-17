# Duckiebot ROS and RR Survey
## Introduction
In this survey, we want students to learn and compare robotics middleware: widely used [Robot Operating System](http://wiki.ros.org/) (ROS) and [Robot Raconteur](https://www.robotraconteur.com/) (RR) designed by RPI alumni. Given a duckiebot with Ubuntu Mate (18.04) image, the goal is to use ROS and Robot Raconteur to achieve lane following.
## Duckiebot
[Duckiebot](https://www.duckietown.org/) is a wheeled robot with 2 motors, one Raspberry Pi, a motor HAT and a Pi Camera. You have the control over 2 motor wheels and read image or video from Pi Cam. Please follow Duckiebot Setup Guide before proceeding.
## Robot Raconteur Tutorial
Robot Raconteur is an object oriented Service-Client middleware. An RR service generally runs with a sensor/robot to have communication directly with them. An RR client usually can receive sensor messages from service and call object function to command the robot. Take a look into given webcam examples on https://github.com/robotraconteur/RobotRaconteur_Python_Examples. Try connecting a webcam to your Ubuntu laptop, running SimpleWebcamService.py and SimpleWebcamClient_streaming.py to get a live window streaming your webcam. 
To get a good understanding of Robot Raconteur, we’ll demonstrate how RR works on duckiebot with keyboard teleop. 
