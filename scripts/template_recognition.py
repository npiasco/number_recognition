import cv2
import numpy as np
import os
import sys
import smach
from treatment_error import TreatmentError

#The state machine, using templateRecognition Object to proceed the recognition
class Template_Recognition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeed', 'fail'],
                             input_keys=['im_input'],
                             output_keys=['number'])
        self.tem=templateRecognition()

    def execute(self, userdata):
        im=userdata.im_input
        try:
            num=self.tem.identify(im)
        except TreatmentError:
            return 'fail'
        else:
            userdata.number=num
            return 'succeed'


class templateRecognition:

    def __init__(self):
        #first load the templates image
        self.template=[None, None, None, None]
        #path to the directory of the num's directories
        pathname = os.path.dirname(sys.argv[0])
        fullpath = os.path.abspath(pathname)+'/templates/'
        print fullpath
        self.template[0] = cv2.imread(fullpath+'template-1.jpg',0)
        self.template[1] = cv2.imread(fullpath+'template0.jpg',0)
        self.template[2] = cv2.imread(fullpath+'template1.jpg',0)
        self.template[3] = cv2.imread(fullpath+'template2.jpg',0)

    def identify(self, im):
           #the treatment have to be done on gray image
           im=cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)

           val=np.zeros((6,4))
           # All the 6 methods for comparison in a list
           methods = ['cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED', 'cv2.TM_CCORR',
                      'cv2.TM_CCORR_NORMED', 'cv2.TM_SQDIFF', 'cv2.TM_SQDIFF_NORMED']
           
           #apply the template matching algorithme on the image with 4 diffrents templates (one by number) and 6 diffrents methodes, and put the result into an array
           i=0
           for tem in self.template:
               j=0
               for meth in methods:
            
                   method = eval(meth)

                   # Apply template Matching
                   res = cv2.matchTemplate(im,tem,method)
                   min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

                   # If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take minimum
                   if method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
                       val[j][i] = min_val
                   else:
                       val[j][i] = max_val

                   j+=1
               i+=1
        
           
           #compare the result between the diffrent template for each methode used, and put the result into the array res
           i=0
           res=[0,0,0,0]
           for seq in val:
               seq=list(seq)
               if i in  [4,5]:
                   res[seq.index(min(seq))]+=1
               else:
                   res[seq.index(max(seq))]+=1
                   i+=1
           #if a template get more than 3 best score among the methode, return the number of the corresponding template
           if max(res)>=4:
               return res.index(max(res))-1
           else:
               raise TeatmentError('Impossible to find the right number', 'template_recognition')


        
            

