#!/usr/bin/env python
from number_recognition.srv import *
import rospy
import sys
from treatment_error import TreatmentError
from recognizeNumber import recognizeNumber
from cv_bridge import CvBridge, CvBridgeError

def handle_read_floor_number(req):
	floor=0
	#Treatment processing...
	print "Recognition..."

	#convert ROS image to opencv matrix
	try:
		bridge = CvBridge()
		cv_image = bridge.imgmsg_to_cv2(req.im, "bgr8")

		#try to recognize
		try:
			floor=recognizeNumber(cv_image)
			return ReadFloorNumberResponse(floor)
		except TreatmentError, e:
			rospy.loginfo("Enable to read floor number : %s"%e)
			raise rospy.ServiceException("Recognition Failed")

	except CvBridgeError, e:
		rospy.loginfo(e)
		raise rospy.ServiceException("Recognition Failed")




def read_floor_number_server():
	rospy.init_node('read_floor_number_server')
	s = rospy.Service('read_floor_number', ReadFloorNumber, handle_read_floor_number)
	print "Ready to treat image."
	rospy.spin()

if __name__ == "__main__":
	read_floor_number_server()
    
    
    
