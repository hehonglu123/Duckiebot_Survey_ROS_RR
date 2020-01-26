#!/usr/bin/env python
import rospy
import cv2
import io
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import time

class CameraNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" %(self.node_name))
        # TODO: load parameters

        self.framerate = self.setupParam("~framerate",60.0)
        self.res_w = self.setupParam("~res_w",320)
        self.res_h = self.setupParam("~res_h",200)

        # TODO: load camera info yaml file and publish CameraInfo
        self.pub_img= rospy.Publisher("image_raw",Image,queue_size=1)
        
        self.has_published = False

        # Setup Webcam
        self.bridge = CvBridge()
        self.camera = cv2.VideoCapture(0)
        self.camera.set(3,320)
        self.camera.set(4,240)
        self.uncompress=1
        # TODO setup other parameters of the camera such as exposure and white balance etc

        # Setup timer
        self.timer_img_low = rospy.Timer(rospy.Duration.from_sec(1.0/self.framerate),self.cbTimer)
        rospy.loginfo("[%s] Initialized." %(self.node_name))

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbTimer(self,event):
        if not rospy.is_shutdown():
            self.grabAndPublish(self.pub_img)
            # Maybe for every 5 img_low, change the setting of the camera and capture a higher res img and publish.

    def grabAndPublish(self,publisher):
        # Grab image from stream

        rval,img_data = self.camera.read()
        if rval:
            # Publish raw image
            image_msg = self.bridge.cv2_to_imgmsg(img_data)
                    
            image_msg.header.stamp = rospy.Time.now()
            # Publish 
            publisher.publish(image_msg)



        if not self.has_published:
            rospy.loginfo("[%s] Published the first image." %(self.node_name))
            self.has_published = True

    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == '__main__': 
    rospy.init_node('camera_node',anonymous=False)
    camera_node = CameraNode()
    rospy.on_shutdown(camera_node.onShutdown)
    rospy.spin()
