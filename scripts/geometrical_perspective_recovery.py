import smach
import rospy
import cv2
import sys
import numpy as np
from treatment_error import TreatmentError
from openCV_treatment import toBinary
from track_number import average_pixel_pos

#return the biggest shape of a binary image
def cleanSmallShape(im):
	img=im.copy()
	contours, hierarchy = cv2.findContours(img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	if len(contours)<2:
		raise TreatmentError('No contours detected : %d contours'%(len(contours)), 'GPR')
	for i in xrange(0,len(contours)):
		cnt = contours[i]
		cv2.drawContours(img, [cnt], 0, i+1, -1)

	hist = cv2.calcHist([img],[0],None,[len(contours)],[0,len(contours)])

	ind=1

	for i in xrange(2, len(hist) ):
		if hist[i]>hist[ind]:
			ind=i
	ret,im = cv2.threshold(im,0,255,cv2.THRESH_BINARY)
	dst=np.zeros_like(im)
	if ind==1:
		cv2.drawContours(dst, contours, ind, 255, 1)
	else:
		cv2.drawContours(dst, contours, ind-1, 255, 1)
	return dst
	
#return the orthogonal projection of A on the line defined by the point B and the unit vector v
def orthogonal_projection(A, B, v):
	
	BA=[A[0]-B[0], A[1]-B[1]]	
	ps=np.vdot(BA, v)
	return [int(ps*v[0]+B[0]), int(ps*v[1]+B[1])]

#return the intersection point p between l1 and l2 (defined by polar coordinate), raise an error if lines are parallel
def intersection_point(l1, l2):
	
	p=[0,0]
	a1 = np.cos(l1[1])
	b1 = np.sin(l1[1])

	a2 = np.cos(l2[1])
	b2 = np.sin(l2[1])

	if b1!=0 and b2!=0 and (a2*b1-a1*b2)!=0:
		p[0]=int((l2[0]*b1-l1[0]*b2)/(a2*b1-a1*b2))
		p[1]=int(-a1/b1*p[0]+l1[0]/b1)
		return True, p
	elif a1!=0 and a2!=0 and (a2*b1-a1*b2)!=0:
		p[1]=int((l2[0]*a1-l1[0]*a2)/(a2*b1-a1*b2))
		p[0]=int(-b1/a1*p[0]+l1[0]/a1)
		return True, p
	else:
		raise TreatmentError('Detected lines are parallel', 'GPR')
		return False, p


class Color_Extraction(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
								outcomes=['extracted', 'fail'],
								input_keys=['im_input'],
								output_keys=['im_output'])
		self.extraction_step=2
								
	def execute(self, userdata):
	
		im=userdata.im_input
		cv2.imshow('Input', im)
		cv2.waitKey(5)
		hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

		if self.extraction_step==2:
			#default extraction
			lower_white = np.array([0,0,120])
			upper_white = np.array([50,150,200])
			m = cv2.inRange(hsv, lower_white, upper_white)
			res=cv2.bitwise_and(im,im, mask= m)
			
		elif self.extraction_step==1:
			#extraction on an overexpose image
			lower_white = np.array([0,65,170])
			upper_white = np.array([80,255,255])
			m = cv2.inRange(hsv, lower_white, upper_white)
			res=cv2.bitwise_and(im,im, mask= m)
			
		elif self.extraction_step==0:
			self.extraction_step=2
			return 'fail'
	
		res=cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
		retval, res=cv2.threshold(res,127,255,cv2.THRESH_BINARY)
		res=toBinary(res)
		
		self.extraction_step-=1
		userdata.im_output=res
		return 'extracted'




#recover the true perspective of im, using the sheet of paper display in the lift
class GPR(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
								outcomes=['succeed', 'fail'],
								input_keys=['im_input', 'im_extracted'],
								output_keys=['im_output'])


	def execute(self, userdata):
		
		thresh=userdata.im_extracted
		im=userdata.im_input

		#enhancement, cleaning little shapes
		kernel = np.ones((5,5),np.uint8)
		closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=3)
		opened = cv2.morphologyEx(closed, cv2.MORPH_OPEN, kernel, iterations=3)
		try:
			sheet=cleanSmallShape(opened)
		except TreatmentError, e:
			rospy.loginfo(str(e))
			return 'fail'

			

		#edge recognition of the sheet
		lines = cv2.HoughLines(sheet,1,np.pi/90,28)
		if not isinstance(lines, np.ndarray):
			rospy.loginfo('Sheet edges not detected, no lines detected, in GPR')
			return 'fail'
		if len(lines[0])<8:
			rospy.loginfo('Sheet edges not detected, %d lines detected, in GPR'%(len(lines[0])))
			return 'fail'
		#Debug print
		'''
		for rho,theta in lines[0]:
			a = np.cos(theta)
			b = np.sin(theta)
			x0 = a*rho
			y0 = b*rho
			x1 = int(x0 + 1000*(-b))
			y1 = int(y0 + 1000*(a))
			x2 = int(x0 - 1000*(-b))
			y2 = int(y0 - 1000*(a))

			cv2.line(sheet,(x1,y1),(x2,y2),125,2)

		cv2.imshow("Image window", sheet)
			cv2.waitKey(-1)
		'''


		mhor=[0,0]
		mvert=[0,0]

		alpha=0.7

		for rho,theta in lines[0]:

			if rho<0:
				rho=-rho
				theta=theta-np.pi


			if (theta > np.pi/2-alpha and theta < np.pi/2+alpha) or (theta > np.pi*1.5-alpha and theta < np.pi*1.5+alpha):

				if mhor == [0, 0]:
					mhor=[rho, theta]
				else:
					mhor=[(mhor[0]+rho)/2, (mhor[1]+theta)/2]
			else:

				if mvert == [0, 0]:
					mvert=[rho, theta]
				else:
					mvert=[(mvert[0]+rho)/2, (mvert[1]+theta)/2]

		try:
			res, A=intersection_point(mhor, mvert)
		except TreatmentError, e:
			rospy.loginfo(str(e))
			return 'fail'

		hor=[[0,0], [0,0]]
		vert=[[0,0], [0,0]]

		for rho,theta in lines[0]:

			if rho<0:
				rho=-rho
				theta=theta-np.pi

			a = np.cos(theta)
			b = np.sin(theta)
			x0 = a*rho
			y0 = b*rho
			x1 = int(x0 + 2000*(-b))
			y1 = int(y0 + 2000*(a))
			x2 = int(x0 - 2000*(-b))
			y2 = int(y0 - 2000*(a))
			v=[x2-x1, y2-y1]
			v=[v[0]/np.linalg.norm(v), v[1]/np.linalg.norm(v)]
			H=orthogonal_projection(A, [x1, y1] , v)


			if theta > np.pi/2-alpha and theta < np.pi/2+alpha:
				#Horizontal lines
				if H[1]<A[1]:
					#top
					if hor[0]==[0,0]:
						hor[0]=[rho, theta]
					else:
						hor[0]=[(hor[0][0]+rho)/2, (hor[0][1]+theta)/2]

				else:
					#low
					if hor[1]==[0,0]:
						hor[1]=[rho, theta]				
					else:
						hor[1]=[(hor[1][0]+rho)/2, (hor[1][1]+theta)/2]
			

			else:
				#Vertical lines
				if H[0]<A[0]:	
					#left
					if vert[0]==[0,0]:
						vert[0]=[rho, theta]
					else:
						vert[0]=[(vert[0][0]+rho)/2, (vert[0][1]+theta)/2]

				else:
					#right
					if vert[1]==[0,0]:
						vert[1]=[rho, theta]				
					else:
						vert[1]=[(vert[1][0]+rho)/2, (vert[1][1]+theta)/2]

		#sheet's corners calculation
		try:
			res, u_l_corner=intersection_point(hor[0], vert[0])	
			res, u_r_corner=intersection_point(hor[0], vert[1])	
			res, l_l_corner=intersection_point(hor[1], vert[0])	
			res, l_r_corner=intersection_point(hor[1], vert[1])
		except TreatmentError, e:
			rospy.loginfo(str(e))
			return 'fail'
		'''
		cv2.circle(sheet, (u_l_corner[0],u_l_corner[1]), 10, 125)
		cv2.circle(sheet, tuple(u_r_corner), 20, 125)
		cv2.circle(sheet, tuple(l_l_corner), 30, 125)
		cv2.circle(sheet, tuple(l_r_corner), 40, 125)
		cv2.imshow("Image window", sheet)
		cv2.waitKey(-1)
		'''

		try:
			#recovery of the perspective
			y, x=sheet.shape
			real_size_sheet=[21*4.5, 29.7*4.5]

			n1=np.linalg.norm([ u_r_corner[0]-u_l_corner[0], u_r_corner[1]-u_l_corner[1] ])
			k1=real_size_sheet[0]/n1
			n2=np.linalg.norm([ l_l_corner[0]-u_l_corner[0], l_l_corner[1]-u_l_corner[1] ])
			k2=real_size_sheet[1]/n2
			n3=np.linalg.norm([ u_r_corner[0]-l_r_corner[0], u_r_corner[1]-l_r_corner[1] ])
			k3=real_size_sheet[1]/n3
			n4=np.linalg.norm([ l_l_corner[0]-l_r_corner[0], l_l_corner[1]-l_r_corner[1] ])
			k4=real_size_sheet[0]/n4

			'''
			hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
		
			lowredl = np.array([0, 0, 255])
			lowredu = np.array([0, 255, 255])

			upredl = np.array([150, 0, 255])
			upredu = np.array([179, 255, 255])	
			
			mlow = cv2.inRange(hsv, lowredl, lowredu)
			mup = cv2.inRange(hsv, upredl, upredu)
			m=cv2.bitwise_or(mlow,mup)
		
			res=cv2.bitwise_and(im,im, mask= m)
		
			res=cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
			retval, binary=cv2.threshold(res,127,255,cv2.THRESH_BINARY)
			binary=toBinary(binary)
		
			red_pos=average_pixel_pos(binary)
			print red_pos
		
			x1=int(red_pos[0]/2)
			y1=int(red_pos[1]/2)
			'''
			x1=50
			y1=0
			x2=x1+int( (u_r_corner[0]-u_l_corner[0])/n1*(x/k1) )
			y2=y1+int( (u_r_corner[1]-u_l_corner[1])/n1*(x/k1) )
			x3=x1+int( (l_l_corner[0]-u_l_corner[0])/n2*(y/k2) )
			y3=y1+int( (l_l_corner[1]-u_l_corner[1])/n2*(y/k2) )	
			x4=x3+int( (l_r_corner[0]-l_l_corner[0])/n4*(x/k4) )
			y4=y3+int( (l_r_corner[1]-l_l_corner[1])/n4*(x/k4) )	

			pts1 = np.float32([[x1, y1],[x2, y2], [x3, y3],[x4, y4]])
			pts2 = np.float32([[0,0],[x, 0],[0,y],[x, y]])

			M = cv2.getPerspectiveTransform(pts1,pts2)
			img = cv2.warpPerspective(im,M,(x,y))

			userdata.im_output=img
			return 'succeed'
		except:
			rospy.loginfo("Unexpected error:%s"%(sys.exc_info()[0]))
			return 'fail'





