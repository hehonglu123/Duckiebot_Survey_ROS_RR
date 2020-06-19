#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class Webcam_impl(): 
    def __init__(self): 
        self.camera = cv2.VideoCapture(0)
        self.camera.set(3,320)
        self.camera.set(4,240)

    def CaptureFrame(self): 
        rval,img_data = self.camera.read()
        if rval:
            return img_data 
        else:
            print("error")
 
 
if __name__ == '__main__': 
    picam=Webcam_impl() 
    pub = rospy.Publisher('webcam', Image, queue_size=0) 
    rospy.init_node('picam', anonymous=False) 
    bridge=CvBridge() 
    img=Image() 
    img.width=320
    img.height=240
    print("running")
    while not rospy.is_shutdown(): 
        frame=picam.CaptureFrame() 
        pub.publish(bridge.cv2_to_imgmsg(frame,"bgr8")) 

