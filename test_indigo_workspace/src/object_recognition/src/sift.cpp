/*
 * sift.cpp - This program performs SIFT on two input images and
 * attempts to find all the regions of interest in both images.  
 *
 * RPI Rock Raiders
 * 4/16/15
 *
 * Last Updated: Bryant Pong - 4/18/15 - 11:38 PM
 */

// ROS Headers:
#include "ros/ros.h"

// OpenCV Headers:
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp> // For SIFT Algorithm

// STL Headers:
#include <iostream> 
#include <vector>

// Helper function to load    

// Main function:
int main(int argc, char **argv) {

	// Initialize ROS:
	ros::init(argc, argv, "SIFTNode");

	// Create a ROS Node Handle for the SIFT node:
	ros::NodeHandle n;

	// Create the SIFT Object:
	cv::SiftFeatureDetector detector;

	if(argc != 2) {
		std::cout << "Usage: rosrun object_recognition sift_node <image>" << std::endl;
		return -1;
	} // End if  

	cv::Mat image1, image2;
	image1 = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);

	if(!image1.data) {
		std::cout << "Could not open image" << std::endl;
		return -1;
	} // End if

	// Display the original image:
	cv::namedWindow("Original Image", cv::WINDOW_NORMAL);
	cv::resizeWindow("Original Image", 640, 480);
	cv::imshow("Original Image", image1);

	// Apply Gaussian Blurring to the image:
	cv::Mat blurredImg;
	cv::GaussianBlur(image1, blurredImg, cv::Size(51,51), 0, 0);

	// Display the blurred image:
	cv::namedWindow("Blurred Image", cv::WINDOW_NORMAL);
	cv::resizeWindow("Blurred Image", 640, 480);
	cv::imshow("Blurred Image", blurredImg);

	// Next use SIFT to detect keypoint in the image:
	
	// A Keypoint vector that holds all the keypoints found by SIFT:
	std::vector<cv::KeyPoint> keypoints;
	detector.detect(blurredImg, keypoints);

	// DEBUG ONLY - Display the keypoints on the image:
	cv::Mat outputImg;
	cv::drawKeypoints(blurredImg, keypoints, outputImg); 		 
	cv::namedWindow("output", cv::WINDOW_NORMAL);
	cv::resizeWindow("output", 640, 480);
	cv::imshow("output", outputImg);

	cv::waitKey(0);

	return 0;
} // End function main()
