import time
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import numpy as np
import cv2
import sys
from collections import namedtuple
import math
from math import floor, atan2, pi, cos, sin, sqrt
from cv_bridge import CvBridge, CvBridgeError

def callback(data):
	bridge=CvBridge()
	try:
		cv_image=bridge.imgmsg_to_cv2(data, "bgr8")
		cv2.namedWindow("Image")
		if (not cv_image is None):
			cv2.imshow("Image",cv_image)
		if cv2.waitKey(50)!=-1:			#if not working, try chaning to ==
			cv2.destroyAllWindows()

	except CvBridgeError as e:
		print(e)

if __name__ == '__main__': 
	rospy.init_node('stream_node', anonymous=False)
	sub = rospy.Subscriber("webcam",Image,callback)
	rospy.spin()
		