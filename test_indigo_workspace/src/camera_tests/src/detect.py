#!/usr/bin/python

# Python Imports:
import cv2 
import numpy as np
import os
from matplotlib import pyplot as plt

# This function loads all images into a list:
def openImages(imageFolder):
	
	# List of image objects:
	imgs = [] 

	for img in os.listdir(imageFolder):
		nextImg = cv2.imread(imageFolder + str(img)) 

		# Convert to grayscale:
		nextImg = cv2.cvtColor(nextImg, cv2.COLOR_BGR2GRAY)
		imgs.append(nextImg)

	return imgs  
	
# This function performs Gaussian smoothing on an image:
def gaussSmooth(img, kernSize, sigma):
	kSize = (int(kernSize*sigma+1), int(kernSize*sigma+1))
	return cv2.GaussianBlur(img, kSize, sigma)

# This function performs Harris corner detection on an image:
def harris(smoothImg, sigma):
	# Calculate the Derivative kernels:
	kx,ky = cv2.getDerivKernels(1,1,3)
	kx = np.transpose(kx/2)
	ky /= 2

	# Calculate the image derivatives in the X/Y directions:
	smoothImgDX = cv2.filter2D(smoothImg, -1, kx)
	smoothImgDY = cv2.filter2D(smoothImg, -1, ky) 

	# Calculate the harris measure:
	harrisSigma = 2 * sigma
	harrisKernel = (4*harrisSigma+1, 4*harrisSigma+1)
	harrisDXSQ = cv2.GaussianBlur(smoothImgDX, harrisKernel, harrisSigma)
	HarrisDYSQ = cv2.GaussianBlur(smoothImgDY, harrisKernel, harrisSigma)
	harrisDXDY = cv2.GaussianBlur(	
	 

# This function displays an image:
def showImg(img):
	plt.gray()
	plt.axis("off")
	plt.imshow(img)
	plt.show() 

# Main function:
def main(imageFolder):

	# Get the list of image objects:
	images = openImages(imageFolder)   

	# Perform feature detection on the next image:
	for image in images:	
		showImg(image)	
		smoothImg = gaussSmooth(image, 4, 2)
		showImg(smoothImg)
		keypoints = harris(smoothImg, 2, 3, 0.04, cv2.BORDER_DEFAULT) 
		showImg(keypoints)

# Main function runner:
if __name__ == "__main__":
	main("./images/")
