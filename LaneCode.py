import vrep
import sys
import time
import numpy as np
import cv2
import imutils
import math
import matplotlib.pyplot as plt

vrep.simxFinish(-1) # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Get the client ID

if clientID!=-1:  #check if client connection successful
	print('Connected to remote API server')
else:
	print('Connection not successful')
	sys.exit('Could not connect')
	
errorCode, cameraHandle = vrep.simxGetObjectHandle(clientID, 'Vision_sensor', vrep.simx_opmode_oneshot_wait)
vrep.simxGetVisionSensorImage(clientID, cameraHandle, 0, vrep.simx_opmode_streaming)
errorCode, leftMotorHandle = vrep.simxGetObjectHandle(clientID,'dr20_leftWheelJoint_',vrep.simx_opmode_oneshot_wait)
errorCode, rightMotorHandle = vrep.simxGetObjectHandle(clientID,'dr20_rightWheelJoint_',vrep.simx_opmode_oneshot_wait)
print('Setting up the camera system...')
lastFrame = None;
err = 0;

def get_image(id,handle):
	err, resolution, image = vrep.simxGetVisionSensorImage(id, handle, 0, vrep.simx_opmode_buffer)
	if err == vrep.simx_return_ok:
		img = np.array(image,dtype=np.uint8)
		img.resize([resolution[1],resolution[0],3])
		lastFrame = img
		return 1,lastFrame
	elif err == vrep.simx_return_novalue_flag:
		return 0, None
	else:
		return err, None

def display_lines(image, lines):
	lineImage = np.zeros_like(image)
	if len(lines[0]) !=0:
		for line in lines:
			x1,y1,x2,y2 = line.reshape(4)
			cv2.line(lineImage,(x1,y1),(x2,y2),(255,0,0),3)
	return lineImage
	
def make_coordinates(image, lineParameters):
	slope, intercept = lineParameters
	y1 = image.shape[0]
	y2 = int( 310)
	x1 = int((y1-intercept)/slope)
	x2 = int((y2-intercept)/slope)
	return np.array([x1,y1,x2,y2])

while(err != 1):
	err, lastFrame = get_image(clientID, cameraHandle)
print('Camera setup successful.')

while True:
	err, img = get_image(clientID, cameraHandle)

	#--------process image--------------------------
	transformedImage = cv2.flip(img,0)
	transformedImage = cv2.cvtColor(transformedImage,cv2.COLOR_BGR2RGB)

	# cv2.imshow('Raw Image',transformedImage)
	# cv2.waitKey(1)
	hsvImage = cv2.cvtColor(transformedImage,cv2.COLOR_BGR2HSV)
	orgImage = np.copy(transformedImage)
	grayImage = cv2.cvtColor(transformedImage,cv2.COLOR_BGR2GRAY)
	lowerYellow = np.array([20,100,100],dtype="uint8")
	upperYellow = np.array([30,255,255],dtype="uint8")
	yellowMask = cv2.inRange(hsvImage,lowerYellow,upperYellow)
	yellowMaskedImage = cv2.bitwise_and(grayImage,yellowMask)
	# cv2.imshow('Yellow mask',yellowMaskedImage)
	# cv2.waitKey(1)
	blurredImage = cv2.GaussianBlur(yellowMaskedImage,(9,9),1)
	cannyImage = cv2.Canny(blurredImage,50,150)
	# cv2.imshow('Canny Filter',cannyImage)
	# cv2.waitKey(1)
	#-------region of interest----------------------
	polygon = np.array([(0,511),(0,330),(511,330),(511,511)],np.int32)
	mask = np.zeros_like(cannyImage)
	cv2.fillPoly(mask,[polygon],255)
	maskedImage = cv2.bitwise_and(cannyImage,mask)
	# cv2.imshow('Masked Image',maskedImage)
	# cv2.waitKey(1)
	#----------find lines--------------------------
	lines = cv2.HoughLinesP(maskedImage, 2, np.pi/180,100,np.array([]),minLineLength=4, maxLineGap=5)
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
			cv2.line(transformedImage, (x1, y1), (x2, y2), (0,0,255), 4)

			slope,intercept = 0,0
			if (x2-x1)!=0:
				slope = (y2-y1)/(x2-x1)
				intercept = y2 - slope * x2
				print(x1,x2,y1,y2)
				print(slope)
				print(intercept)
				if slope<0:
					leftLinesParam.append((slope,intercept))
					leftLines.append((line))
					cv2.line(maskedImage, (x1, y1), (x2, y2), (0,200,255), 5)
				elif slope >0:
					rightLinesParam.append((slope,intercept))
					rightLines.append((line))
					cv2.line(maskedImage, (x1, y1), (x2, y2), (255,200,0), 5)

				else:
					pass
			else:
					pass
					
		# cv2.imshow('HoughLinesP finding',transformedImage)
		# cv2.waitKey(1)

		cv2.imshow('Left and Right Lane',maskedImage)
		cv2.waitKey(1)
		maxPower = 5
		kp=3
		if len(leftLinesParam)!=0 and len(rightLinesParam) !=0:
			avgLeftLineParam = np.average(leftLinesParam,axis=0)
			avgRightLineParam = np.average(rightLinesParam,axis=0)
			leftLane = make_coordinates(orgImage, avgLeftLineParam)
			rightLane = make_coordinates(orgImage, avgRightLineParam)
			avgSlope = avgLeftLineParam[0] + avgRightLineParam[0]
			errorCode=vrep.simxSetJointTargetVelocity(clientID,leftMotorHandle,0, vrep.simx_opmode_oneshot)
			errorCode=vrep.simxSetJointTargetVelocity(clientID,rightMotorHandle,0, vrep.simx_opmode_oneshot)

			if avgSlope <-0.34:
				print(avgSlope,"Moving right")
				errorCode=vrep.simxSetJointTargetVelocity(clientID,leftMotorHandle,1, vrep.simx_opmode_oneshot)
				errorCode=vrep.simxSetJointTargetVelocity(clientID,rightMotorHandle,2, vrep.simx_opmode_oneshot)
			elif avgSlope >0.34:
				print(avgSlope,"Moving left")
				errorCode=vrep.simxSetJointTargetVelocity(clientID,leftMotorHandle,2, vrep.simx_opmode_oneshot)
				errorCode=vrep.simxSetJointTargetVelocity(clientID,rightMotorHandle,1, vrep.simx_opmode_oneshot)
			else:
				print("Moving forward")
				errorCode=vrep.simxSetJointTargetVelocity(clientID,leftMotorHandle,2, vrep.simx_opmode_oneshot)
				errorCode=vrep.simxSetJointTargetVelocity(clientID,rightMotorHandle,2	, vrep.simx_opmode_oneshot)
				
			linesImage = display_lines(orgImage,[leftLane,rightLane])
			tst = cv2.addWeighted(orgImage,0.8,linesImage,1,1)
			cv2.imshow('view',tst)
			cv2.waitKey(1)

		elif len(leftLinesParam)!=0 and len(rightLinesParam) ==0:
			print(slope,"Moving right")
			avgLeftLineParam = np.average(leftLinesParam,axis=0)
			leftLane = make_coordinates(orgImage, avgLeftLineParam)
			slope = avgLeftLineParam[0]
			errorCode=vrep.simxSetJointTargetVelocity(clientID,leftMotorHandle,2, vrep.simx_opmode_oneshot)
			errorCode=vrep.simxSetJointTargetVelocity(clientID,rightMotorHandle,-1, vrep.simx_opmode_oneshot)
			linesImage = display_lines(orgImage,[leftLane])
			tst = cv2.addWeighted(orgImage,0.8,linesImage,1,1)
			cv2.imshow('view',tst)
			cv2.waitKey(1)

		elif len(leftLinesParam)==0 and len(rightLinesParam) !=0:
			print(slope,"Moving left")
			avgRightLineParam = np.average(rightLinesParam,axis=0)
			rightLane = make_coordinates(orgImage, avgRightLineParam)
			slope = avgRightLineParam[0]
			errorCode=vrep.simxSetJointTargetVelocity(clientID,leftMotorHandle,-1, vrep.simx_opmode_oneshot)
			errorCode=vrep.simxSetJointTargetVelocity(clientID,rightMotorHandle,2, vrep.simx_opmode_oneshot)
			linesImage = display_lines(orgImage,[rightLane])
			tst = cv2.addWeighted(orgImage,0.8,linesImage,1,1)
			cv2.imshow('view',tst)
			cv2.waitKey(1)
			
		else:
			print("stopped no line lane found")
			errorCode=vrep.simxSetJointTargetVelocity(clientID,leftMotorHandle,0, vrep.simx_opmode_oneshot)
			errorCode=vrep.simxSetJointTargetVelocity(clientID,rightMotorHandle,0, vrep.simx_opmode_oneshot)
