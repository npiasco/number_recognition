import cv2
import numpy as np
import os
import glob
from openCV_treatment import toBinary
from treatment_error import TreatmentError

class ImageComparator:
	def __init__ (self):
		self.li_m1=list()
		self.li_0=list()
		self.li_1=list()
		self.li_2=list()
		#path to the directory of the num's directories
		path='number_example/'
		dirs=glob.glob(path+'*')
		for folder in dirs:

			im=cv2.imread(folder+'/-1.jpg',0)
			im=toBinary(im)
			self.li_m1.append(im)

			im=cv2.imread(folder+'/0.jpg', 0)
			im=toBinary(im)
			self.li_0.append(im)

			im=cv2.imread(folder+'/1.jpg', 0)
			im=toBinary(im)
			self.li_1.append(im)

			im=cv2.imread(folder+'/2.jpg', 0)
			im=toBinary(im)
			self.li_2.append(im)

	#identify the number on the image im, according to the database loaded in the constructor
	def identify(self, im):
		result=[0, 0, 0, 0]
		result[0]=mult_image_compare(self.li_m1, im)
		result[1]=mult_image_compare(self.li_0, im)
		result[2]=mult_image_compare(self.li_1, im)
		result[3]=mult_image_compare(self.li_2, im)
		number=-1+result.index(max(result))
		print result
		for i in result:
			if result[number+1]-i>0 and result[number+1]-i<5:
				raise TreatmentError('too close to other numbers, number=%d dif=%f'%(number, result[number+1]-i), 'recognition_number')
		return number


#resize im2 to im1 size, then compare each pixel
def image_compare(im1, im2):
	try:
		y1, x1=im1.shape
		y2, x2=im2.shape
		
	except ValueError:
		print "Not a binary image"
		raise ValueError("Not a binary image")
		
	else:
		img2=cv2.resize(im2,( x1, y1))
		img2=toBinary(img2)

		hist = cv2.calcHist([im1],[0],None,[256],[0,256])

		img2[im1==img2]=255
		img2[im1!=img2]=0

		hist2 = cv2.calcHist([img2],[0],None,[256],[0,256])
		return 	float(hist2[255])/hist[255]*100
	



#compare each image in l with im
def mult_image_compare(l, im):
	average=0.;
	for i in l:
		try:
			if average==0.:
				average=image_compare(i, im)
			else:
				average=(average+image_compare(i, im))/2

		except ValueError, e:
			print 'Error in mult_image_compare: ', e
			raise ValueError(e)
			

	return average

        

if __name__=='__main__':

	f='resultCV/final_test2.jpg'
	im=cv2.imread(f, 0)
	im=toBinary(im)

	ic=ImageComparator()
	print ic.identify(im)
