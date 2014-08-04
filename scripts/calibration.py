#!/usr/bin/env python
from number_recognition.srv import *
import rospy
import cv2
import numpy as np
from treatment_error import TreatmentError
from sensor_msgs.msg import Image
from recognizeNumber import recognizeNumber
from cv_bridge import CvBridge, CvBridgeError
from openCV_treatment import toBinary

def nothing(x):
    pass
    
class calibration:

	def __init__(self):
		rospy.init_node('color_calibration')
		self.image_sub = rospy.Subscriber("head_xtion/rgb/image_color",Image,self.callback) 

		self.iLowH = 0;
		self.iHighH = 179;

		self.iLowS = 0; 
		self.iHighS = 255;

		self.iLowV = 0;
		self.iHighV = 255;
		
		cv2.namedWindow('control')
		
		cv2.createTrackbar('LowH','control',0,179,nothing)
		cv2.createTrackbar('HighH','control',0,179,nothing)
		cv2.createTrackbar('LowS','control',0,255,nothing)
		cv2.createTrackbar('HighS','control',0,255,nothing)
		cv2.createTrackbar('LowV','control',0,255,nothing)
		cv2.createTrackbar('HighV','control',0,255,nothing)

		 
	def callback(self, im):
		try:
			bridge = CvBridge()
			cv_image = bridge.imgmsg_to_cv2(im, "bgr8")

		except CvBridgeError, e:
			print e
			raise rospy.ServiceException("Recognition Failed")

		self.iLowH = cv2.getTrackbarPos('LowH','control')
		self.iHighH = cv2.getTrackbarPos('HighH','control')
		self.iLowS = cv2.getTrackbarPos('LowS','control')
		self.iHighS = cv2.getTrackbarPos('HighS','control')
		self.iLowV = cv2.getTrackbarPos('LowV','control')
		self.iHighV = cv2.getTrackbarPos('HighV','control')

		lower_red = np.array([self.iLowH,self.iLowS,self.iLowV])
		upper_red = np.array([self.iHighH,self.iHighS,self.iHighV])

		hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		maskR = cv2.inRange(hsv, lower_red, upper_red)
		resR = cv2.bitwise_and(cv_image,cv_image, mask= maskR)
		
		resR=cv2.cvtColor(resR, cv2.COLOR_BGR2GRAY)
		retval, resR=cv2.threshold(resR,127,255,cv2.THRESH_BINARY)

		cv2.imshow('Calibration', resR)
		cv2.waitKey(3)
		
		lowredl = np.array([0, 0, 255])
		lowredu = np.array([0, 255, 255])

		upperredl = np.array([150, 0, 255])
		upperredu = np.array([179, 255, 255])		
		
		mlow = cv2.inRange(hsv, lowredl, lowredu)
		mup = cv2.inRange(hsv, upperredl, upperredu)
		
		m=cv2.bitwise_or(mlow,mup)
		res=cv2.bitwise_and(cv_image,cv_image, mask= m)
		res=cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
		retval, res=cv2.threshold(res,127,255,cv2.THRESH_BINARY)
		cv2.imshow('Test', res)
		cv2.waitKey(3)		
		
	def lauch(self):
		print "Ready for calibration"		


if __name__ == "__main__":
	c=calibration()
	c.lauch
	rospy.spin()
