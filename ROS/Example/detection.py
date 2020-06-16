import cv2
import numpy as np
def detection(image):				#takes in opencv image type, return [c,r] of centroid of stop sign
	lowerb = np.array([0, 0, 160])
	upperb = np.array([120, 120, 255])
	stop_filtered = cv2.inRange(image, lowerb, upperb)		#BGR
	cv2.imshow('image',stop_filtered)			#filtered image display
	cv2.waitKey(0)
	cv2.destroyAllWindows()
	retval, labels, stats, centroids=cv2.connectedComponentsWithStats(stop_filtered)
	print(stats)
	for i in range(len(stats)):
		if stats[i][4]>5000 and stats[i][4]<50000 :
			return(centroids[i])		

