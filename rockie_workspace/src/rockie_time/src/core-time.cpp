/*
 * core-time.cpp - This node keeps track of the elapsed time that has passed
 * since the robot started. 
 *
 * RPI Rock Raiders
 * 2/26/15
 *
 * Last Updated: Bryant Pong: 2/27/15 - 7:14 PM
 */

// ROS Libraries:
#include "ros/ros.h"
#include "std_msgs/Float64.h"

// C++ STL Libraries:
#include <sstream>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>

// This variable keeps track of the time when Rockie starts up:
clock_t startTime = 0;

/** FUNCTION PROTOTYPES **/
double calcElapsedTime(void);
/** END SECTION FUNCTION PROTOTYPES **/

// This function calculates the elapsed time (in minutes):
double calcElapsedTime(void) {
	clock_t curElapsedTime = clock();		

	return double(curElapsedTime - startTime) / CLOCKS_PER_SEC * 10 / 60.0; 
} // End function calcElapsedTime()

// Main function:
int main(int argc, char **argv) {

	// Initialize the core-time node:
	ros::init(argc, argv, "coretime"); 	

	// Create the Node Handle for core-time:
	ros::NodeHandle coreTime;

	// Use the clock() function to initial the starting time:
	startTime = clock();

	// The name of this node:
	char *nodeName = (char *) malloc(sizeof(char) * 9);
	strcpy(nodeName, "core-time");  

	// DEBUG ONLY: Print out the startTime:
	printf("DEBUG ONLY (%s module) - startTime is %ld\n", nodeName, startTime);

	// Create a ROS Publisher to publish the elapsed time:
	ros::Publisher time_pub = coreTime.advertise<std_msgs::Float64>("elapsedTime", 10);
	
	// Keep publishing elapsed time: 
	while(ros::ok()) {
		// Create a Float64 message to send the elapsed time in minutes:
		std_msgs::Float64 elTime;

		printf("DEBUG ONLY - elapsedTime is %lf\n", calcElapsedTime());
		elTime.data = calcElapsedTime();

		// Publish the elapsed time (in minutes):
		time_pub.publish(elTime);

		ros::spinOnce();	
		
	} // End while

	// Free dynamically allocated memory here:
	free(nodeName);

	return 0;
} // End function main()
