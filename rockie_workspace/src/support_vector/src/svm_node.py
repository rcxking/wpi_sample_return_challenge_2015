#!/usr/bin/python

'''
svm_node.py - This node uses the support vector machine for Phase 1.         

RPI Rock Raiders
6/7/15

Last Updated: Bryant Pong: 6/7/15 - 9:16 PM
'''

# Python Imports
import rospy
import sklearn.svm as svm
import cPickle as pickle
import os
import gc
import numpy as np
import matplotlib.pyplot as plt
from object_tracker.msg import *
from sensor_msgs.msg import CameraInfo, Image

# Global Variables:
nextImage, nextCameraInfo = None, None
imgSet, infoSet = False, False

# Helper function to generate sliding windows:
def slidingWindow(img, labels, winLen, winHgt): 
	
	# Size of the image:
	imgLen = img.shape[1]
	imgHgt = img.shape[0]
				
	# Determine if there is a sample in this image:
	sample = False 
	sampleMidpointCol, sampleMidpointRow = None, None
	if len(labels) > 0:
		sample = True
		avg = np.mean(labels, axis=0)
		sampleMidpointCol = avg[1]
		sampleMidpointRow = avg[0]
																			
	# Ensure that this image can be evenly divided by winLen, winHgt:
	if winLen == 0 or winHgt == 0 or imgLen % winLen != 0 or imgHgt % winHgt != 0:
		print("Error window parameters do not divide img evenly!")
		return []	
									
	# Determine the number of window blocks to cover:
	numBlkLen = int(imgLen / winLen)
	numBlkHgt = int(imgHgt / winHgt)
	imgs = []
	lbls = []
	
	# Extract the blocks:
	for m in xrange(numBlkHgt):
		for n in xrange(numBlkLen):
			nextCornerRow = m*winHgt
			nextCornerCol = n*winLen
			if sample:
				if sampleMidpointRow >= nextCornerCol and sampleMidpointRow <= nextCornerCol + winHgt and \
						sampleMidpointCol >= nextCornerRow and sampleMidpointCol <= nextCornerRow + winLen:
					lbls.append(1)
				else:
					lbls.append(-1)
			else:
				lbls.append(-1)
		imgs.append(img[nextCornerRow:nextCornerRow+winHgt, nextCornerCol:nextCornerCol+winLen])
	return imgs, lbls  

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
	global nextImg, imgSet  

	if not imgSet:
		nextImg = data
		imgSet = True

# Callback for the Camera Info Subscriber:
def camInfoCallback(data):
	global nextCameraInfo, infoSet

	if not infoSet:
		nextCameraInfo = data
		infoSet = True

# Main Node:
def svmNode(SVM):

	# Referencing global variables:
	global nextImage, nextCameraInfo, imgSet, infoSet
	
	# Initialize the SVM node:
	rospy.init_node("svm")

	'''
	Create two subscribers to acquire the camera images and transformational
	matrices:      
	'''
	imgSub = rospy.Subscriber("/image_rect_color", Image, imgSubCallback)
	matSub = rospy.Subscriber("/camera_info", CameraInfo, camInfoCallback)

	svmPub = rospy.Publisher("/svminfo", Observation)

	while not rospy.is_shutdown():
		# Is the next set of data ready to go?
		if imgSet and infoSet:
			rospy.loginfo("svm_node: Now sending out prediction")     
		
			# Extract the image:
			img = nextImage.data 							

			# Run the sliding window algorithm:
			slidingWindows, _ = slidingWindow(img, [], 160, 160) 

			avgRow, avgCol = 0.0, 0.0 
			numPos = 0
			counter = 0
			# Run the prediction function of the SVM:
			for sImg in slidingWindows:
				# Get the prediction for this image:
				pred = SVM.predict(sImg)[0]
				
				if pred == 1:
					numPos += 1
					# Determine the region of this image:
					
					if counter == 0 or counter == 1 or counter == 2 or counter == 3:
						avgRow += 80
					elif counter == 4 or counter == 5 or counter == 6 or counter == 7:
						avgRow += 240
					else:
						avgRow += 400  	 

					if counter == 0 or counter == 4 or counter == 8:
						avgCol += 80
					elif counter == 1 or counter == 5 or counter == 9:
						avgCol += 240 
					elif counter == 2 or counter == 6 or counter == 10:
						avgCol += 400
					else:
						avgCol += 560 

				counter += 1
			if numPos > 0: 
				avgRow /= numPos
				avgCol /= numPos				

			# Construct the Observation Message to publish:
			observe = Observation()  
			observe.header = nextImage.header
			observe.P = nextCameraInfo.P
			observe.point = [avgCol, avgRow]
			svmPub.publish(observe)
			imgSet, infoSet = False, False

		rospy.spin()	
					
if __name__ == "__main__":

	rospy.loginfo("svm_node: Now loading support vector machine")
	supVecMac = pickle.load(open("SVM.pkl", "rb"))  
	rospy.loginfo("svm_node: Done loading support vector machine")
	svmNode(supVecMac)		 
