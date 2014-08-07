#!/usr/bin/env python
from number_recognition.srv import *
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from openCV_treatment import toBinary
from treatment_error import TreatmentError


#return the position of the average of all pixels in im
def average_pixel_pos(im):
	x=0
	y=0
	i=0
	j=0
	cpt=0
	for l in im:
		for pix in l:
			if pix:
				x+=j
				y+=i				
				cpt+=1
			j+=1
		i+=1
		j=0
	x=int(x/cpt)
	y=int(y/cpt)
	if cpt<100 or cpt>500:
		raise TreatmentError('red tracker fail', 'track number')
	return (x,y)




def track_number_callback(im, arg):
	#convert ROS image to opencv matrix
	l, pub, msg = arg
	target_pos=(400, 210)
	try:
		bridge = CvBridge()
		cv_image = bridge.imgmsg_to_cv2(im, "bgr8")

		hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		
		lowredl = np.array([0, 0, 255])
		lowredu = np.array([0, 255, 255])

		upredl = np.array([150, 0, 255])
		upredu = np.array([179, 255, 255])	
			
		mlow = cv2.inRange(hsv, lowredl, lowredu)
		mup = cv2.inRange(hsv, upredl, upredu)
		m=cv2.bitwise_or(mlow,mup)
		
		res=cv2.bitwise_and(cv_image,cv_image, mask= m)
		
		res=cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
		retval, binary=cv2.threshold(res,127,255,cv2.THRESH_BINARY)
		binary=toBinary(binary)
		
		red_pos=average_pixel_pos(binary)
		
		print red_pos
		cv2.circle(binary, red_pos, 10, 125)
		cv2.imshow('Track', binary)
		cv2.waitKey(1) 	
		#move the camera to focus the number
		#...
		
	except CvBridgeError,  e:
		rospy.loginfo(e)
		raise rospy.ServiceException("Tracking Failed")
	except TreatmentError,  e:
		rospy.loginfo(e)
		msg.position=[-0.7, 0.5]
		pub.publish(msg)
		rospy.sleep(5)
#		raise rospy.ServiceException("Tracking Failed")

	else:
		if abs(red_pos[0]-target_pos[0])>20 or abs(red_pos[1]-target_pos[1])>20:
				
			if (red_pos[0]-target_pos[0])>0:
				#move to right
				msg.position[0]-=0.05
			else:
				msg.position[0]+=0.05

			if (red_pos[1]-target_pos[1])>0:
				#move down
				msg.position[1]+=0.05
			else:
				msg.position[1]-=0.05
								
			print msg.position
			pub.publish(msg)
			rospy.sleep(1)
			
		else:
			l[0]=True



def handle_track_number(req):
	#Treatment processing...
	print "Tracking..."
	l=[False]
	pub = rospy.Publisher('/ptu/cmd', JointState, queue_size=1)

	#wait for listener...
	rospy.sleep(1)
	
	msg=JointState()
	msg.name=['pan','tilt']
	msg.velocity=[0.8, 0.8]
	msg.position=[-0.7, 0.5]
	
	#defaul position
	pub.publish(msg)

	
	#suscribe to the topic of the camera
	image_subscriber = rospy.Subscriber(req.cameraTopic,Image, track_number_callback, [l, pub, msg])
	
	while l[0]==False:
		pass 
	image_subscriber.unregister()
	return TrackNumberResponse()

		

def track_number_server():
	rospy.init_node('track_number_server')
	s = rospy.Service('track_number', TrackNumber, handle_track_number)
	print "Ready to track."
	rospy.spin()

if __name__ == "__main__":
    track_number_server()