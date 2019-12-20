#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
class streaming:
	def __init__(self):
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("picam",Image,self.callback)

	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		cv2.imshow("Image window", cv_image)
		cv2.waitKey(3)
	 
def main(args):
	picam_stream=streaming()
	rospy.init_node('picam_streaming', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)