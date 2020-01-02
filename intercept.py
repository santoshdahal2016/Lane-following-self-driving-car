import vrep
import sys
import time
import numpy as np
import cv2
import imutils
import math
import matplotlib.pyplot as plt


img = cv2.imread('test.png')


# cv2.imshow('Raw Image',img)
# cv2.waitKey(0)
hsvImage = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
orgImage = np.copy(img)

grayImage = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)


blurredImage = cv2.GaussianBlur(grayImage,(9,9),1)
cannyImage = cv2.Canny(blurredImage,50,150)
# cv2.imshow('Canny Filter',cannyImage)
# cv2.waitKey(0)

#----------find lines--------------------------
lines = cv2.HoughLinesP(cannyImage, 2, np.pi/180,100,np.array([]),minLineLength=4, maxLineGap=5)
#-----------get average lines for the two lanes --------------------
leftLinesParam =[]
rightLinesParam =[]
leftLines =[]
rightLines =[]
leftLane=[]
rightLane=[]
if lines is not None:
	for line in lines:
		x1,y1,x2,y2 =line.reshape(4)
		print(x1,y1,x2,y2)
		cv2.line(img, (x1, y1), (x2, y2), (0,0,255), 4)

		slope,intercept = 0,0
		if (x2-x1)!=0:
			slope = (y2-y1)/(x2-x1)
			intercept = y2 - slope * x2
			print(slope)
			print(intercept)
		# 	if slope<0:
		# 		leftLinesParam.append((slope,intercept))
		# 		leftLines.append((line))
		# 		cv2.line(img, (x1, y1), (x2, y2), (0,200,255), 5)
		# 	elif slope >0:
		# 		rightLinesParam.append((slope,intercept))
		# 		rightLines.append((line))
		# 		cv2.line(img, (x1, y1), (x2, y2), (255,200,0), 5)

		# 	else:
		# 		pass
		# else:
		# 		pass
				
	# cv2.imshow('HoughLinesP finding',img)
	# cv2.waitKey(1)

	cv2.imshow('Left and Right Lane',img)
	cv2.waitKey(0)