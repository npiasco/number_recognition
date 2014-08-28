import rospy
import smach
import cv2
import numpy as np

class Luminosity_Correction(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
                                     outcomes=['succeed', 'aborted'],
                                     input_keys=['im_input'],
                                     output_keys=['im_output'])								
	def execute(self, userdata):
	
		im=userdata.im_input
                
                try:
                    #Split the image to get HSV values
                    im = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
                    h,s,v = cv2.split(im)
                
                    #First treatment, decrease the luminosity of none red pixel
                    masq1=v.copy()
                    masq2=v.copy()
                    masq3=v.copy()
                
                    masq1[h<1]=0
                    masq1[h>150]=0

                    masq2[h>1]=0
                    masq3[h<150]=0
                    masq2=cv2.add(masq2,masq3)
                
                    masq1=cv2.addWeighted(masq1,0.2,v,0,0)

                    v=cv2.add(masq1,masq2)

                    #Second treatment, decrease the luminosity of unsaturate pixel
                    masq1=v.copy()
                    masq2=v.copy()

                    threshold=5
                    masq1[s<threshold]=0
                    masq2[s>threshold]=0
                    masq2=cv2.addWeighted(masq2,0.2,v,0,0)

                    v=cv2.add(masq1,masq2)
                
                    #Recreate RGB image
                    im=cv2.merge((h,s,v))
                    im = cv2.cvtColor(im1, cv2.COLOR_HSV2BGR)
                    
                    userdata.im_output=im

                    return 'succeed'
                

                except:
                    rospy.loginfo('Cannot procced to luminosity recovery, aborted')
                    return 'aborted'
