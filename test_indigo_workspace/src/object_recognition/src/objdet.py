#!/usr/bin/python

'''
objdet.py - Module to extract regions of interest from an image.

RPI Rock Raiders
4/25/15

Last Updated: Bryant Pong: 4/25/15 - 6:40 PM 
'''

# Python Includes:
import cv2
import rospy
from std_msgs.msg import String
from object_recognition.srv import *
import sklearn
import numpy as np
import pickle
from matplotlib import pyplot as plt

# List of images to process:
imgList = []  

'''
This function processes images and extracts feature descriptors:
'''
def extractDescriptors():
	pass

'''
Callback to load images:
'''
def openImages(req):
	global imgList

	# Extract the path to the image to extract:
	imgPath = req.path

	img = cv2.imread(imgPath)
	imgList.append(img)

	# DEBUG ONLY - Display the loaded image:
	plt.axis("off")
	img = img[:,:,::-1]
	plt.imshow(img)
	plt.show()
	cv2.waitKey(0) 

	return None

# Main function:
def main():

	# Initialize ROS:
	sub = rospy.Service('imgProc', Image, openImages)
	pub = rospy.Publisher('objdet', String, queue_size=10)
	rospy.init_node('objdet', anonymous=True)

	while not rospy.is_shutdown():
		hello_str = "hello world %s" % rospy.get_time()
		rospy.loginfo(hello_str)
		pub.publish(hello_str)


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
