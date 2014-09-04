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

class track_number:

	def __init__(self):
		rospy.init_node('track_number_server')
		s = rospy.Service('track_number', TrackNumber, self.handle_track_number)
		print "Ready to track."
		self.status=False
		self.localization=3

        #return the position of the average of all pixels in im
	def average_pixel_pos(self, im):
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
		if cpt<25 or cpt>500:
	               	#number not detected
			raise TreatmentError('red tracker fail, number of pixels founded: %d'%cpt, 'track number')

		x=int(x/cpt)
		y=int(y/cpt)
	
		return (x,y)




	def track_number_callback(self, im, arg):

		pub, msg = arg
		target_pos=(380, 210)
		try:
			bridge = CvBridge()
		        #convert ROS image to opencv matrix
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
		
			red_pos=self.average_pixel_pos(binary)
			print red_pos
		
		except CvBridgeError,  e:
			rospy.loginfo(e)
			raise rospy.ServiceException("Tracking Failed")

		except TreatmentError,  e:
			rospy.loginfo(e)
			if self.localization==3:
				msg.position=[-0.9, 0.5]
			elif self.localization==2:
				msg.position=[-1.2, 0.5]
			elif self.localization==1:
				msg.position=[-1.5, 0.5]
			else:
				raise rospy.ServiceException("Tracking Failed")
			self.localization-=1
			pub.publish(msg)
			rospy.sleep(0.5)

		else:
		#move the camera to focus the number
			thresh=20
			if abs(red_pos[0]-target_pos[0])>thresh or abs(red_pos[1]-target_pos[1])>thresh:
				
				if (red_pos[0]-target_pos[0])>thresh:
				#move to right
					msg.position[0]-=0.01
				elif (red_pos[0]-target_pos[0])<thresh:
					msg.position[0]+=0.01
					
				if (red_pos[1]-target_pos[1])>thresh:
				#move down
					msg.position[1]+=0.01
				elif (red_pos[1]-target_pos[1])<thresh:
					msg.position[1]-=0.01
								
				
				print msg.position
				pub.publish(msg)
			
			else:
				self.status=True



	def handle_track_number(self, req):
		try:
	                #Treatment processing...
			print "Tracking..."
			pub = rospy.Publisher('/ptu/cmd', JointState, queue_size=1)

            	        #wait for listener...
	 		rospy.sleep(0.5)
	
			msg=JointState()
			msg.name=['pan','tilt']
			msg.velocity=[0.8, 0.8]
			msg.position=[-0.7, 0.5]
	
                   	#defaul position
			pub.publish(msg)

	
                     	#suscribe to the topic of the camera
			image_subscriber = rospy.Subscriber(req.cameraTopic,Image, self.track_number_callback, [pub, msg])
	
                     	#start a timer
			t=rospy.get_time()
			elaps=0
			while t==0:
				t=rospy.get_time()

	
			while self.status==False and elaps < 15.:

				t_c=rospy.get_time()
				elaps=t_c-t
			
			if elaps >15:
				raise rospy.ServiceException("Taking to mutch time for tracking. Aborted")
			return TrackNumberResponse()
		finally:
			print 'End of tracking'
			image_subscriber.unregister()
			self.status=False
			self.localization=3

		

if __name__ == "__main__":
	t=track_number()
	rospy.spin()

