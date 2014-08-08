#!/usr/bin/env python

import sys
import rospy
from sensor_msgs.msg import Image
from number_recognition.srv import *

def read_floor_number_client(im, arg):
	
	if arg[0]==1:
		rospy.wait_for_service('read_floor_number')

		try:
			read_floor_number = rospy.ServiceProxy('read_floor_number', ReadFloorNumber)
			resp = read_floor_number(im)
			print "floor :  %s"%(resp.floorNumber)

		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		
		finally:
			arg[0]=0
	else:
		pass



if __name__ == "__main__":

	rospy.init_node('client', anonymous=True)
	
	if len(sys.argv)<2:
		rospy.loginfo('Camera topic needed')
		sys.exit(0)
		

	print "Requesting srv"

	#move the camera to focus the number
	rospy.wait_for_service('track_number')
	try:
		track_number = rospy.ServiceProxy('track_number', TrackNumber)
		res=track_number(sys.argv[1])

	except rospy.ServiceException, e:
		rospy.loginfo("Service call failed: %s"%e)
		sys.exit()	
	arg=[0]	
	image_sub = rospy.Subscriber("head_xtion/rgb/image_color",Image,read_floor_number_client, arg)
	while not rospy.is_shutdown():
		try:
			raw_input('Launch recognition ?')
			arg[0]=1
		except KeyboardInterrupt:
			sys.exit(0)
