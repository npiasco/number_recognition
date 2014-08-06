import cv2
import numpy as np
from treatment_error import TreatmentError

def toBinary(im):
	img=im.copy()
	img[img!=0]=255
	return img
	
def extraction(im):
	x, y =im.shape
	box=[x, y, 0, 0]
	f_shape=False
	ligne_cpt=0
	i=0
	j=0
	for l in im:
		for pix in l:
			if pix:


				if i < box[0]:
					box[0]=i
				if j < box[1]:
					box[1]=j
				if i > box[2]:
					box[2]=i
				if j > box[3]:
					box[3]=j

				f_shape=True
				ligne_cpt=0
				

			if f_shape:
				ligne_cpt=ligne_cpt+1
			if ligne_cpt > 5*y:
				#avoid the focuse on noise
				
				if (box[2]-box[0])*(box[3]-box[1]) < 100:
					box=[x, y, 0, 0]
					f_shape=False
					ligne_cpt=0
				else:
					break
			j+=1
		i+=1
		j=0
				
	edge=3
	return [box[0]-edge, box[1]-edge, box[2]+edge, box[3]+edge], box

def detectLignes(im):

	y, x = im.shape
	h=0
	v=0	
	pt=int(max(x,y)/4)
	print pt

	try: 
	
		lines = cv2.HoughLines(im,1,np.pi/2,pt )
		for rho,theta in lines[0]:
			print rho
			a = np.cos(theta)
			b = np.sin(theta)
			x0 = a*rho
			y0 = b*rho
			x1 = int(x0 + 1000*(-b))
			y1 = int(y0 + 1000*(a))
			x2 = int(x0 - 1000*(-b))
			y2 = int(y0 - 1000*(a))

			cv2.line(im,(x1,y1),(x2,y2),(155),1)
			if theta==0:
				v=v+1
			else:
				h=h+1
			
		return (h, v, im)

	except TypeError:
		print "No line founded"
		return (0, 0, im)


def main(im):

	#color traitment
	hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

	lower_red = np.array([0,0,200])
	upper_red = np.array([179,255,255])

	maskR = cv2.inRange(hsv, lower_red, upper_red)
	resR = cv2.bitwise_and(im,im, mask= maskR)
	
	#make the image binary
	gray=cv2.cvtColor(resR, cv2.COLOR_BGR2GRAY)
	thresh=toBinary(gray)
	cv2.imshow('MyW', thresh)
	cv2.waitKey(-1)
	#exctracting number
	box, boderless=extraction(thresh)
	extracted=thresh[box[0]:box[2], box[1]:box[3]]
	
	#making an image with an area of 30000 pixels
	area=30000
	y, x=extracted.shape
	print x*y
	try:
		r=float(x)/y
		ny=int(np.sqrt(area/r))
	except ZeroDivisionError:
		raise TreatmentError('Wrong dimension of extraction: x=%d, y=%d'%(x,y), 'extracted number')
		

	nx=int(r*ny)

	extracted=cv2.resize(extracted, (nx, ny))
	extracted=toBinary(extracted)

	
	#Binary treatment
	kernel=np.array([[0, 0, 0, 1, 0, 0, 0], 
					 [0, 0, 1, 1, 1, 0, 0], 
					 [0, 1, 1, 1, 1, 1, 0],
					 [1, 1, 1, 1, 1, 1, 1],
					 [0, 1, 1, 1, 1, 1, 0],
					 [0, 0, 1, 1, 1, 0, 0],
					 [0, 0, 0, 1, 0, 0, 0]], np.uint8)

	close=cv2.morphologyEx(extracted, cv2.MORPH_CLOSE, kernel, iterations=3)

	return close

if __name__ == '__main__':


	f="test0.jpg"
	#loading image
	im=cv2.imread("lift_pictures/" + f, 1)

	im = main(im)
	cv2.imwrite("resultCV/final_"+f,im)
























