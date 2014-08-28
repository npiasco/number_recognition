import smach
import cv2
import numpy as np
from openCV_treatment import toBinary
from openCV_treatment import extraction
from image_compare import ImageComparator
from treatment_error import TreatmentError
												
class Color_Extraction(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
								outcomes=['extracted', 'fail', 'fail_after_LC'],
								input_keys=['im_input'],
								output_keys=['im_output'])
		self.isGPRed=False
		self.extraction_step=3
								
	def execute(self, userdata):
	
		im=userdata.im_input
		#Debug visualization
		#cv2.imshow('Input', im)
		#cv2.waitKey(5)

		#working in the HSV color model to extract the red pixel
		hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
		
		#first threshold to red extraction
		lowredl = np.array([0, 0, 255])
		lowredu = np.array([0, 255, 255])

		#second threshold to red extraction
		upredl = np.array([150, 0, 255])
		upredu = np.array([179, 255, 255])		
		
		mlow = cv2.inRange(hsv, lowredl, lowredu)
		mup = cv2.inRange(hsv, upredl, upredu)


		#trying diffrent type of extraction, if the process failed three time, call the luminosity correction process
		if self.extraction_step==3:
			m=cv2.bitwise_or(mlow,mup)
			res=cv2.bitwise_and(im,im, mask= m)
		elif self.extraction_step==2:
			res=cv2.bitwise_and(im,im, mask= mup)
		elif self.extraction_step==1:
			res=cv2.bitwise_and(im,im, mask= mlow)
		elif self.extraction_step==0:
			if self.isGPRed:
				return 'fail_after_LC'
			else:
				self.extraction_step=3
				return 'fail'
		
		#make the thresholded image binary
		res=cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
		retval, res=cv2.threshold(res,127,255,cv2.THRESH_BINARY)
		res=toBinary(res)

		
		self.extraction_step-=1
		userdata.im_output=res
		return 'extracted'


class Number_Extraction(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
								outcomes=['succeed', 'fail'],
								input_keys=['im_input'],
								output_keys=['im_output'])								
	def execute(self, userdata):
	
		im=userdata.im_input
		
		#extracted the interessting part of the image (the one with the number)
		box, boderless=extraction(im)
		extracted=im[box[0]:box[2], box[1]:box[3]]
		
		#resizing the image to make an image with an area of 30000 pixels^2
		area=30000
		y, x=extracted.shape
		print x*y
		try:
			r=float(x)/y
			ny=int(np.sqrt(area/r))
		except ZeroDivisionError:
			print 'Wrong dimension of extraction: x=%d, y=%d'%(x,y)
			return 'fail'
		if x*y<400 and x*2>y:
			print 'Wrong area: x=%d, y=%d'%(x,y)
			return 'fail'

		nx=int(r*ny)

		extracted=cv2.resize(extracted, (nx, ny))
		
		#binary thresholding requested after resize
		extracted=toBinary(extracted)

		#Debuging print
		#cv2.imshow('Extracted', extracted)
		#cv2.waitKey(5)

		userdata.im_output=extracted
		return 'succeed'



class Recognition(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
								outcomes=['succeed', 'fail'],
								input_keys=['im_input'],
								output_keys=['im_output', 'number'])
		#tool used to recognition
		self.ic=ImageComparator()
		
	def execute(self, userdata):
	
		im=userdata.im_input
		
		try:
			num=self.ic.identify(im)
		except TreatmentError,e:
			print e
			userdata.im_output=im
			return 'fail'
			
		else:
			userdata.number=num
			return 'succeed'


class Binary_Treatment(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
								outcomes=['succeed', 'fail'],
								input_keys=['im_input'],
								output_keys=['im_output'])								
		self.binary_step=3
		self.im=None
		
	def execute(self, userdata):
		
		#the kernel used to perform morphological transformation
		kernel=np.array([[0, 0, 0, 1, 0, 0, 0], 
						 [0, 0, 1, 1, 1, 0, 0], 
						 [0, 1, 1, 1, 1, 1, 0],
						 [1, 1, 1, 1, 1, 1, 1],
						 [0, 1, 1, 1, 1, 1, 0],
						 [0, 0, 1, 1, 1, 0, 0],
						 [0, 0, 0, 1, 0, 0, 0]], np.uint8)

               #trying diffrent type of transformation, if the process failed three time, go to step Color_Extraction to try an other extraction
		if self.binary_step==3:
		
			self.im=userdata.im_input
			res=cv2.morphologyEx(self.im, cv2.MORPH_CLOSE, kernel, iterations=1)
			
		if self.binary_step==2:

			res=cv2.morphologyEx(self.im, cv2.MORPH_CLOSE, kernel, iterations=2)

		if self.binary_step==1:

			res=cv2.morphologyEx(self.im, cv2.MORPH_CLOSE, kernel, iterations=3)
			
		elif self.binary_step==0:
			#reset binary counter to treat the image
			self.binary_step=3
			return 'fail'
			
		self.binary_step-=1
		userdata.im_output=res
		
		return 'succeed'































