#!/usr/bin/python

'''
svm_node.py - This node uses the support vector machine for Phase 1.         

RPI Rock Raiders
6/7/15

Last Updated: Bryant Pong: 6/7/15 - 9:16 PM
'''

# Python Imports
import rospy
import sklearn
import sklearn.svm as svm
import cPickle as pickle
import os
import gc
import numpy as np
import matplotlib.pyplot as plt
from object_tracker.msg import *
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError

# Global Variables:
nextImage, nextCameraInfo = None, None
imgSet, infoSet = False, False
svm_file = None
SVM = None
svmPub = None
bridge = CvBridge()
throttle_number = 0
image_count = 0

# Helper function to generate sliding windows:
def slidingWindow(img, winLen, winHgt): 
	
	# Size of the image:
	imgLen = img.shape[1]
	imgHgt = img.shape[0]
				
	# Determine the number of window blocks to cover:
	numBlkLen = int(imgLen / winLen)
	numBlkHgt = int(imgHgt / winHgt)
	imgs = []
	
	# Extract the blocks:
	for m in xrange(numBlkHgt):
		for n in xrange(numBlkLen):
			row = []
			nextCornerRow = m*winHgt
			nextCornerCol = n*winLen
			row.append(img[nextCornerRow:nextCornerRow+winHgt, nextCornerCol:nextCornerCol+winLen])
		imgs.append(row)
	return imgs

'''
This function generates an 8x8x8 color histogram-based descriptor to run the
SVM on. 
'''
def calcDescriptor(img):
	flattenedImg = img.flatten()
	
	numElements = 0
	nextPixel = []
	nextPos = 0

	# Images received are 160 x 160 px
	convertedImg = np.zeros((160**2,3))

	for val in np.nditer(flattenedImg):
		numElements += 1
		nextPixel.append(val)

		if numElements >= 3:
			convertedImg[nextPos] = nextPixel
			nextPos += 1

			numElements = 0
			nextPixel = []
	histogram, _ = np.histogramdd(convertedImg, bins=[8,8,8])

	return histogram.flatten()

# Callback for the image subscriber:
def imgSubCallback(data):
	global SVM, bridge, svmPub, throttle_number
	global image_count
	if not image_count % throttle_number == 0:
		return
	nextImage = data

	# Extract the image:
	img = bridge.imgmsg_to_cv2(nextImage, "bgr8")

	'''
	# Run the sliding window algorithm:
	dim = 160
	slidingWindows = slidingWindow(img, 160, 160) 

	avgRow, avgCol = 0.0, 0.0 
	numPos = 0
	# Run the prediction function of the SVM:
	for i, row in enumerate(slidingWindows):
		for j, sImg in enumerate(row):
			# Get the prediction for this image:
			descriptor = calcDescriptor(sImg)
			pred = SVM.predict(descriptor)[0]
			
			if pred == 1:
				numPos += 1
				avgRow += (i-1) * dim + dim/2
				avgCol += (j-1) * dim + dim/2
	'''

	#if numPos > 0: 
	if True:
		#avgRow /= numPos
		#avgCol /= numPos				
		avgRow = img.shape[0]/2
		avgCol = img.shape[1]/2
		# Construct the Observation Message to publish:
		observe = Observation()  
		observe.header = nextImage.header
		observe.P = nextCameraInfo.P
		observe.point = [avgCol, avgRow]
		svmPub.publish(observe)
		rospy.loginfo("svm: sending out observation")

# Callback for the Camera Info Subscriber:
def camInfoCallback(data):
	global nextCameraInfo, infoSet

	if not infoSet:
		nextCameraInfo = data
		infoSet = True

# Main Node:
def svmNode():

	# Referencing global variables:
	global nextCameraInfo, imgSet, infoSet, svm_file, SVM, svmPub, throttle_number
	
	# Initialize the SVM node:
	rospy.init_node("svm")

	'''
	Create two subscribers to acquire the camera images and transformational
	matrices:      
	'''
	imgSub = rospy.Subscriber("image_rect_color", Image, imgSubCallback)
	matSub = rospy.Subscriber("camera_info", CameraInfo, camInfoCallback)

	svmPub = rospy.Publisher("svminfo", Observation)

	svm_file = rospy.get_param("svm_file", "SVM.pkl")
	throttle_number = rospy.get_param("throttle_number", 3)

	SVM = pickle.load(open(svm_file, "rb"))  

	rospy.spin()	
					
if __name__ == "__main__":

	rospy.loginfo("svm_node: Now loading support vector machine")
	rospy.loginfo("svm_node: Done loading support vector machine")
	svmNode()		 
