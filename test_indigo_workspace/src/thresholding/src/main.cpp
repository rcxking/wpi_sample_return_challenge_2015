//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include "thresholding/ProcessImage.h"
//Include headers for OpenCV Image processing
#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>

#include <stdlib.h>
#include <stdio.h>
#include <iostream>

//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;

// Global image variable to process:
cv::Mat img2Proc; 

//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Image Processed";

int iLowH = 0;
int iHighH = 179;

int iLowS = 0; 
int iHighS = 255;

int iLowV = 0;
int iHighV = 255;

int houghResolution = 10;
int minRadius = 5;
int maxRadius = 20;
int cannyThreshold = 40;
int accumulatorThreshold = 40;

int thresh = 30;
int max_thresh = 255;
cv::RNG rng(12345);

//Use method of ImageTransport to create image publisher
image_transport::Publisher pub;

void anonomy_ycrcb(cv_bridge::CvImagePtr cv_ptr) {

	cv::Mat gray;
	cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
	cv::cvtColor(gray, gray, CV_GRAY2BGR);

	cv::Mat real_col;
	cv::cvtColor(cv_ptr->image, real_col, CV_BGR2YCrCb);

	cv::Scalar average = cv::mean(real_col);

	cv::Mat standout;
	cv::Mat avg;

	cv::inRange(real_col, average-cv::Scalar(thresh,thresh,thresh), average+cv::Scalar(thresh,thresh,thresh), real_col);


	cv::erode(real_col, real_col, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)) );
	cv::dilate(real_col, real_col, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)) );

	cv::dilate(real_col, real_col, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)) );
	cv::erode(real_col, real_col, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)) );

	cv::namedWindow( "original", CV_WINDOW_AUTOSIZE);
	cv::imshow("original", cv_ptr->image);
	cv::imshow("updated", real_col);


	cv::waitKey(3);
	pub.publish(cv_ptr->toImageMsg());
} 

void hsv_anonomy(cv_bridge::CvImagePtr cv_ptr) {
	cv::Mat gray;
	cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
	cv::cvtColor(gray, gray, CV_GRAY2BGR);

	cv::Mat real_col;
	cv::cvtColor(cv_ptr->image, real_col, CV_BGR2HSV);

	cv::Scalar average = cv::mean(real_col);

	cv::Mat standout;
	cv::Mat avg;

	average[1] = 127;
	average[2] = 127;

	cv::inRange(real_col, average-cv::Scalar(thresh,127,127), average+cv::Scalar(thresh,127,127), real_col);

	cv::namedWindow( "original", CV_WINDOW_AUTOSIZE);
	cv::imshow( "original", cv_ptr->image);
	cv::imshow("updated", real_col);


	cv::waitKey(3);
	pub.publish(cv_ptr->toImageMsg());
}

void color_extract(cv_bridge::CvImagePtr cv_ptr) {
	cv::Mat gray;
	cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
	cv::cvtColor(gray, gray, CV_GRAY2BGR);

	cv::Mat real_col;
	cv_ptr->image.convertTo(real_col, CV_32F);
	gray.convertTo(gray, CV_32F);
	real_col = real_col-1.0*(gray-255/2);
	real_col.convertTo(real_col, CV_8U);

	cv::Mat standout;
	cv::Mat avg;

	cv::namedWindow( "original", CV_WINDOW_AUTOSIZE);
	cv::imshow( "original", cv_ptr->image);
	cv::imshow("updated", real_col);

	cv::waitKey(3);
	pub.publish(cv_ptr->toImageMsg());
}

//void anomony(cv_bridge::CvImagePtr cv_ptr) {
void anomony(const cv::Mat& img) {

	/*
	cv::Mat gray;
	cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
	cv::cvtColor(gray, gray, CV_GRAY2BGR);

	cv::Mat real_col;
	cv_ptr->image.convertTo(real_col, CV_32F);
	gray.convertTo(gray, CV_32F);
	real_col = real_col.mul(1-gray/255+.5);
	real_col.convertTo(real_col, CV_8U);

	cv::Scalar average = cv::mean(real_col);
	cv::Mat standout;
	cv::Mat avg;

	cv::inRange(real_col, average-cv::Scalar(thresh,thresh,thresh), average+cv::Scalar(thresh,thresh,thresh), real_col);

	cv::namedWindow( "original", CV_WINDOW_AUTOSIZE);
	cv::imshow( "original", cv_ptr->image);
	cv::imshow("updated", real_col);
	cv_ptr->image = real_col;

	cv::waitKey(3);
	pub.publish(cv_ptr->toImageMsg());
	*/

	cv::Mat gray;
	cv::cvtColor(img, gray, CV_BGR2GRAY);
	cv::cvtColor(gray, gray, CV_GRAY2BGR);

	cv::Mat real_col;
	gray.convertTo(gray, CV_32F);
	real_col = real_col.mul(1-gray/255+.5);
	real_col.convertTo(real_col, CV_8U);

	cv::Scalar average = cv::mean(real_col);
	cv::Mat standout;
	cv::Mat avg;

	cv::inRange(real_col, average-cv::Scalar(thresh, thresh, thresh), average+cv::Scalar(thresh,thresh,thresh), real_col);

	cv::namedWindow("original", cv::WINDOW_NORMAL);
	cv::imshow("original", img);
	cv::imshow("updated", real_col);

	cv::waitKey(0);
}

// color detection
void color_detection(cv_bridge::CvImagePtr cv_ptr)
{
	cv::Mat fullImageHSV;
	cv::cvtColor(cv_ptr->image, fullImageHSV, CV_BGR2HSV);

	cv::Mat imgThresholded;

	cv::inRange(fullImageHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded);

	cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)) );
	cv::dilate(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)) );

	cv::dilate(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)) );
	cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)) );

	cv::imshow("Thresholded Image", imgThresholded);

	//Display the image using OpenCV
	cv::imshow("Original", cv_ptr->image);
	//Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
	cv::waitKey(3);
	/**
	 * The publish() function is how you send messages. The parameter
	 * is the message object. The type of this object must agree with the type
	 * given as a template parameter to the advertise<>() call, as was done
	 * in the constructor in main().
	 */
	//Convert the CvImage to a ROS image message and publish it on the "camera/image_processed" topic.
	pub.publish(cv_ptr->toImageMsg());
}

//This function is called everytime a new image is published
// void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
void imageCallback(const cv::Mat &original_image)
{
	//Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing

	/*	
	cv_bridge::CvImagePtr cv_ptr;

	try
	{
		//Always copy, returning a mutable CvImage
		//OpenCV expects color images to use BGR channel order.
		//cv_ptr = cv_bridge::toCvCopy(&original_image, enc::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		//if there is an error during conversion, display it
		ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
		return;
	}
	*/

	anomony(original_image);
}

// Callback that opens the image for processing:
bool processImgCallback(thresholding::ProcessImage::Request &req,
		thresholding::ProcessImage::Response &res) {

	std::cout << "imagepath: " << req.imagename << std::endl;

	img2Proc = cv::imread(req.imagename, CV_LOAD_IMAGE_COLOR);

	// DEBUG ONLY - Display the initial image:
	cv::namedWindow("DEBUG WINDOW", cv::WINDOW_NORMAL);
	cv::resizeWindow("DEBUG WINDOW", 640, 480);
	cv::imshow("DEBUG WINDOW", img2Proc);

	cv::waitKey(0);

	imageCallback(img2Proc);

	return true;
} // End callback processImgCallback 

/**
 * This tutorial demonstrates simple image conversion between ROS image message and OpenCV formats and image processing
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_processor");
	ros::NodeHandle nh;
	//Create an ImageTransport instance, initializing it with our NodeHandle.
	image_transport::ImageTransport it(nh);
	//OpenCV HighGUI call to create a display window on start-up.
	cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
	cv::createTrackbar("Threshold:", WINDOW, &thresh, max_thresh);

	// Create an image_transport service client to take in a string message that designates the image to process:
	ros::ServiceServer service = nh.advertiseService("processimage", processImgCallback);

	//pub = it.advertise("camera/image_processed", 1);
	/**
	 * In this application all user callbacks will be called from within the ros::spin() call. 
	 * ros::spin() will not return until the node has been shutdown, either through a call 
	 * to ros::shutdown() or a Ctrl-C.
	 */
	ros::spin();
	//ROS_INFO is the replacement for printf/cout.
	ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");
}
