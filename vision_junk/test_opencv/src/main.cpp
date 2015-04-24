//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
//Include headers for OpenCV Image processing
#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
 
//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;
 
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

	int thresh = 100;
	int max_thresh = 255;
	cv::RNG rng(12345);

 
//Use method of ImageTransport to create image publisher
image_transport::Publisher pub;

void circle_detection(cv_bridge::CvImagePtr cv_ptr);
/* 
//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
  //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
      //Always copy, returning a mutable CvImage
      //OpenCV expects color images to use BGR channel order.
      cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
      //if there is an error during conversion, display it
      ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
      return;
  }

color_extract(cv_ptr);
}
*/
void histf(cv_bridge::CvImagePtr cv_ptr) {
    cv::Mat hsv;
    cv::Mat src = cv_ptr->image;
    cv::cvtColor(src, hsv, CV_BGR2HSV);

    // Quantize the hue to 30 levels
    // and the saturation to 32 levels
    int hbins = 30, sbins = 32;
    int histSize[] = {hbins, sbins};
    // hue varies from 0 to 179, see cvtColor
    float hranges[] = { 0, 180 };
    // saturation varies from 0 (black-gray-white) to
    // 255 (pure spectrum color)
    float sranges[] = { 0, 256 };
    const float* ranges[] = { hranges, sranges };
    cv::MatND hist;
    // we compute the histogram from the 0-th and 1-st channels
    int channels[] = {0, 1};

    cv::calcHist( &hsv, 1, channels, cv::Mat(), // do not use mask
             hist, 2, histSize, ranges,
             true, // the histogram is uniform
             false );
    double maxVal=0;
    cv::minMaxLoc(hist, 0, &maxVal, 0, 0);

    int scale = 10;
    cv::Mat histImg = cv::Mat::zeros(sbins*scale, hbins*10, CV_8UC3);

    for( int h = 0; h < hbins; h++ )
        for( int s = 0; s < sbins; s++ )
        {
            float binVal = hist.at<float>(h, s);
            int intensity = cvRound(binVal*255/maxVal);
            cv::rectangle( histImg, cv::Point(h*scale, s*scale),
                        cv::Point( (h+1)*scale - 1, (s+1)*scale - 1),
                        cv::Scalar::all(intensity),
                        CV_FILLED );
        }

    cv::namedWindow( "Source", 1 );
    cv::imshow( "Source", src );

    cv::namedWindow( "H-S Histogram", 1 );
    cv::imshow( "H-S Histogram", histImg );
	cv::waitKey(3);
	pub.publish(cv_ptr->toImageMsg());
}


void anonomy_ycrcb(cv_bridge::CvImagePtr cv_ptr) {

  cv::Mat gray;
  cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
  cv::cvtColor(gray, gray, CV_GRAY2BGR);

  cv::Mat real_col;
//  cv_ptr->image.convertTo(real_col, CV_32F);
//  gray.convertTo(gray, CV_32F);
//  real_col = real_col-.75*(gray-255/2);
//  real_col.convertTo(real_col, CV_8U);
  
  cv::cvtColor(cv_ptr->image, real_col, CV_BGR2YCrCb);

  cv::Scalar average = cv::mean(real_col);

  cv::Mat standout;
  cv::Mat avg;

  average[0] = 127;
  //average[2] = 127;

  cv::inRange(real_col, average-cv::Scalar(127,thresh,thresh), average+cv::Scalar(127,thresh,thresh), real_col);


  cv::erode(real_col, real_col, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)) );
  cv::dilate(real_col, real_col, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)) );

  cv::dilate(real_col, real_col, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)) );
  cv::erode(real_col, real_col, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)) );

	
	cv::namedWindow( "original", CV_WINDOW_AUTOSIZE);
	cv::imshow( "original", cv_ptr->image);
	cv::imshow("updated", real_col);


	cv::waitKey(3);
	pub.publish(cv_ptr->toImageMsg());
}


void hsv_anonomy(cv_bridge::CvImagePtr cv_ptr) {
  cv::Mat gray;
  cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
  cv::cvtColor(gray, gray, CV_GRAY2BGR);

  cv::Mat real_col;
//  cv_ptr->image.convertTo(real_col, CV_32F);
//  gray.convertTo(gray, CV_32F);
//  real_col = real_col-.75*(gray-255/2);
//  real_col.convertTo(real_col, CV_8U);
  
  cv::cvtColor(cv_ptr->image, real_col, CV_BGR2HSV);

  cv::Scalar average = cv::mean(real_col);

  cv::Mat standout;
  cv::Mat avg;
/*
    // Quantize the hue to 30 levels
    // and the saturation to 32 levels
    int hbins = 180;
    int histSize[] = {hbins, sbins};
    // hue varies from 0 to 179, see cvtColor
    float hranges[] = { 0, 180 };
    // saturation varies from 0 (black-gray-white) to
    // 255 (pure spectrum color)
    float sranges[] = { 0, 256 };
    const float* ranges[] = { hranges, sranges };
    cv::MatND hist;
    // we compute the histogram from the 0-th and 1-st channels
    int channels[] = {0, 1};

    cv::calcHist( &hsv, 1, channels, cv::Mat(), // do not use mask
             hist, 2, histSize, ranges,
             true, // the histogram is uniform
             false );
*/
 
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

  //cv::Scalar average = cv::mean(real_col);

  cv::Mat standout;
  cv::Mat avg;

  //average[1] = 127;
  //average[2] = 127;

  //cv::inRange(real_col, average-cv::Scalar(thresh,127,127), average+cv::Scalar(thresh,127,127), real_col);

	cv::namedWindow( "original", CV_WINDOW_AUTOSIZE);
	cv::imshow( "original", cv_ptr->image);
	cv::imshow("updated", real_col);


	cv::waitKey(3);
	pub.publish(cv_ptr->toImageMsg());
}

void anomony(cv_bridge::CvImagePtr cv_ptr) {
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


	cv::waitKey(3);
	pub.publish(cv_ptr->toImageMsg());
}


void edges(cv_bridge::CvImagePtr cv_ptr)
{
	cv::Mat gray;
	cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
	cv::blur( gray, gray, cv::Size(3,3) );

	cv::Mat threshold_output;
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	//detect edges usign threshold
	cv::threshold( gray, threshold_output, thresh, 255, cv::THRESH_BINARY);
	//find contours
	cv::findContours (threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0) );

	std::vector< std::vector<cv::Point> > contours_poly( contours.size() );
	std::vector< cv::Rect> boundRect( contours.size() );
	std::vector<cv::Point2f> center(contours.size() );
	std::vector<float> radius(contours.size());

	for (int i =0; i < contours.size(); i++)
	{
		approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true);
		boundRect[i] = boundingRect( cv::Mat(contours_poly[i]));
		minEnclosingCircle( (cv::Mat)contours_poly[i], center[i], radius[i] );
	}

	cv::Mat drawing = cv::Mat::zeros( threshold_output.size(), CV_8UC3);
	for(int i = 0; i < contours.size(); i++)
	{
		cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
		drawContours( drawing, contours_poly, i , color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
//		circle(drawing, center[i], (int)radius[i], color, 2, 8, 0);
	}

	cv::namedWindow( "Contours", CV_WINDOW_AUTOSIZE);
	cv::imshow( "threshold", threshold_output);
	cv::imshow("Contours", drawing);


	cv::waitKey(3);
	pub.publish(cv_ptr->toImageMsg());
}

//better circle
void slow_circle(cv_bridge::CvImagePtr cv_ptr)
{
	//filter image
	cv::Mat filtered;
	cv::pyrMeanShiftFiltering(cv_ptr->image, filtered, 10/*spatialWindowRadius*/, 10/*colorWindowRadius*/, 2);

	//increase sharpness
	cv::Mat grayImg;
	cv::Mat_<float> blurred;
	cv::Mat_<float> laplaccian;
	cv::Mat grayscale;
	cv::cvtColor(filtered, grayImg, cv::COLOR_BGR2GRAY);
	grayImg.convertTo(grayImg, CV_32F);

	cv::GaussianBlur(grayImg, blurred, cv::Size(5,5), 0);

	cv::Laplacian(blurred, laplaccian, CV_32F);

	cv::Mat sharpened = 1.5f * grayImg - 0.5f * blurred - .4/*weight*/ * grayImg.mul(.2/*scale*/ * laplaccian);

	//circle detection
	std::vector<cv::Vec3f> circles;

	//apply the Hough Transform to find the circles
	sharpened.convertTo(sharpened, CV_8U);
	cv::HoughCircles( sharpened, circles, CV_HOUGH_GRADIENT, 1/*0.1f * houghResolution*/, sharpened.rows/8 /*distance between*/, cannyThreshold, accumulatorThreshold, minRadius, maxRadius);


//		ROS_INFO("Number of circles: %ld", circles.size());
	//Draw the cirles detected
	for (size_t i = 0; i < circles.size(); i++)
	{
		cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		//circle center
		cv::circle( cv_ptr->image, center, 3, cv::Scalar(0,255,0), -1, 8, 0);
		//circle outline
		cv::circle( cv_ptr->image, center, radius, cv::Scalar(0,0,255), 3, 8, 0);
	}

		

		cv::namedWindow("Hough Circle Transform Demo", CV_WINDOW_AUTOSIZE);

		cv::imshow( "Hough Circle Transform Demo", cv_ptr->image);

//		cv::imshow("Gray Scale", grayscale_image);	
	
		cv::waitKey(3);
		pub.publish(cv_ptr->toImageMsg());



}

void circle_detection(cv_bridge::CvImagePtr cv_ptr)
{
		//color detection
		cv::Mat fullImageHSV;
		cv::cvtColor(cv_ptr->image, fullImageHSV, CV_BGR2HSV);
 
		cv::Mat imgThresholded;

		cv::inRange(fullImageHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded);


/*		cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)) );
		cv::dilate(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)) );

		cv::dilate(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)) );
		cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)) );
*/
		cv::imshow("Thresholded Image", imgThresholded);


		//circle detection
		cv::Mat grayscale_image;
		cv::Mat image;

		cv::cvtColor(cv_ptr->image, grayscale_image, CV_BGR2GRAY);
		//imgThresholded.copyTo(grayscale_image);

		//reduce the noise so we avoid false circle detection
		cv::GaussianBlur( grayscale_image, grayscale_image, cv::Size(9, 9), 2, 2 );
		std::vector<cv::Vec3f> circles;

		//apply the Hough Transform to find the circles
		cv::HoughCircles( grayscale_image, circles, CV_HOUGH_GRADIENT, .1*houghResolution, grayscale_image.rows/8, cannyThreshold, accumulatorThreshold, minRadius, maxRadius);


//		ROS_INFO("Number of circles: %ld", circles.size());
		//Draw the cirles detected
		for (size_t i = 0; i < circles.size(); i++)
		{
			cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
			int radius = cvRound(circles[i][2]);
			//circle center
			cv::circle( cv_ptr->image, center, 3, cv::Scalar(0,255,0), -1, 8, 0);
			//circle outline
			cv::circle( cv_ptr->image, center, radius, cv::Scalar(0,0,255), 3, 8, 0);
		}

		

		cv::namedWindow("Hough Circle Transform Demo", CV_WINDOW_AUTOSIZE);

		cv::imshow( "Hough Circle Transform Demo", cv_ptr->image);

		cv::imshow("Gray Scale", grayscale_image);	
	
		cv::waitKey(3);
		pub.publish(cv_ptr->toImageMsg());

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


    //Invert Image
    //Go through all the rows
/*    for(int i=0; i<cv_ptr->image.rows; i++)
    {
        //Go through all the columns
        for(int j=0; j<cv_ptr->image.cols; j++)
        {
            //Go through all the channels (b, g, r)
            for(int k=0; k<cv_ptr->image.channels(); k++)
            {
                //Invert the image by subtracting image data from 255               
                cv_ptr->image.data[i*cv_ptr->image.rows*4+j*3 + k] = 255-cv_ptr->image.data[i*cv_ptr->image.rows*4+j*3 + k];
            }
        }
    }
*/     
 
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
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
  //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
      //Always copy, returning a mutable CvImage
      //OpenCV expects color images to use BGR channel order.
      cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
      //if there is an error during conversion, display it
      ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
      return;
  }

histf(cv_ptr);
//color_extract(cv_ptr);
//hsv_anonomy(cv_ptr);
//anomony(cv_ptr);
//anonomy_ycrcb(cv_ptr);
}


/**
* This tutorial demonstrates simple image conversion between ROS image message and OpenCV formats and image processing
*/
int main(int argc, char **argv)
{
    /**
    * The ros::init() function needs to see argc and argv so that it can perform
    * any ROS arguments and name remapping that were provided at the command line. For programmatic
    * remappings you can use a different version of init() which takes remappings
    * directly, but for most command-line programs, passing argc and argv is the easiest
    * way to do it.  The third argument to init() is the name of the node. Node names must be unique in a running system.
    * The name used here must be a base name, ie. it cannot have a / in it.
    * You must call one of the versions of ros::init() before using any other
    * part of the ROS system.
    */
        ros::init(argc, argv, "image_processor");
    /**
    * NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.
    */
        ros::NodeHandle nh;
    //Create an ImageTransport instance, initializing it with our NodeHandle.
        image_transport::ImageTransport it(nh);
    //OpenCV HighGUI call to create a display window on start-up.
    cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);




	  //Create trackbars in "Control" window
 		cvCreateTrackbar("LowH", 	WINDOW, &iLowH, 179); //Hue (0 - 179)
 		cvCreateTrackbar("HighH", WINDOW, &iHighH, 179);

  	cvCreateTrackbar("LowS", WINDOW, &iLowS, 255); //Saturation (0 - 255)
 		cvCreateTrackbar("HighS", WINDOW, &iHighS, 255);

  	cvCreateTrackbar("LowV", WINDOW, &iLowV, 255); //Value (0 - 255)
 		cvCreateTrackbar("HighV", WINDOW, &iHighV, 255);

		cv::createTrackbar("Canny threshold",       WINDOW, &cannyThreshold,       255);
    cv::createTrackbar("Accumulator Threshold", WINDOW, &accumulatorThreshold, 200);
 
    cv::createTrackbar("Hough resolution",      WINDOW, &houghResolution,      100);
    cv::createTrackbar("Min Radius",            WINDOW, &minRadius,             50);
    cv::createTrackbar("Max Radius",            WINDOW, &maxRadius,            200);
    
		cv::createTrackbar("Threshold:", WINDOW, &thresh, max_thresh);

		/**
    * Subscribe to the "camera/image_raw" base topic. The actual ROS topic subscribed to depends on which transport is used. 
    * In the default case, "raw" transport, the topic is in fact "camera/image_raw" with type sensor_msgs/Image. ROS will call 
    * the "imageCallback" function whenever a new image arrives. The 2nd argument is the queue size.
    * subscribe() returns an image_transport::Subscriber object, that you must hold on to until you want to unsubscribe. 
    * When the Subscriber object is destructed, it will automatically unsubscribe from the "camera/image_raw" base topic.
    */
    image_transport::Subscriber sub = it.subscribe("camera/image_raw", 1, imageCallback);
    //OpenCV HighGUI call to destroy a display window on shut-down.
    //cv::destroyWindow(WINDOW);
    /**
    * The advertise() function is how you tell ROS that you want to
    * publish on a given topic name. This invokes a call to the ROS
    * master node, which keeps a registry of who is publishing and who
    * is subscribing. After this advertise() call is made, the master
    * node will notify anyone who is trying to subscribe to this topic name,
    * and they will in turn negotiate a peer-to-peer connection with this
    * node.  advertise() returns a Publisher object which allows you to
    * publish messages on that topic through a call to publish().  Once
    * all copies of the returned Publisher object are destroyed, the topic
    * will be automatically unadvertised.
    *
    * The second parameter to advertise() is the size of the message queue
    * used for publishing messages.  If messages are published more quickly
    * than we can send them, the number here specifies how many messages to
    * buffer up before throwing some away.
    */
        pub = it.advertise("camera/image_processed", 1);
    /**
    * In this application all user callbacks will be called from within the ros::spin() call. 
    * ros::spin() will not return until the node has been shutdown, either through a call 
    * to ros::shutdown() or a Ctrl-C.
    */
        ros::spin();
    //ROS_INFO is the replacement for printf/cout.
    ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");
 
}
