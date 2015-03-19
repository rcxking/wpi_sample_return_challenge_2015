/*
 * core-main.cpp - This is the main "master" node for Rockie. This node will
 * listen to all other nodes and determine what the next course of action to 
 * take.
 *
 * RPI Rock Raiders
 * 2/26/15
 *
 * Last Updated: Bryant Pong - 3/12/15 - 6:48 PM
 */

/*
 * ROS Includes:
 */
#include "ros/ros.h"

// Type definitions:
#include "../include/rockie_core/rockie-types.h"

/*
 * C++ STL Headers:
 * cstdio - Output functions (i.e printf)   
 * cstdlib - Memory functions (malloc, free) 
 * cstring - C String functions 
 */
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <queue> 

/** Struct definitions **/
struct task {
	uint priority;
	std::string taskName;
};

/*
 * This struct contains the finite state representation of the robot's
 * state machine:
 */
struct globalfnm {
	
};
/** END SECTION Struct definitions **/



// Main function:
int main(int argc, char **argv) {

	// Initialize the core-main node:
	ros::init(argc, argv, "coremain");

	// Create the ROS Node Handle for core-main:
	ros::NodeHandle coreMain;		

	// The name of this node:
	char *nodeName = (char *) malloc(sizeof(char) * 9);
	strcpy(nodeName, "core-main");

	// This priority queue stores tasks to complete by the robot:
	//std::priority_queue< 

	printf("DEBUG ONLY - Now starting %s node\n", nodeName);
	
	// Free Dynamically allocated memory:
	free(nodeName);
			
	return 0;	
} // End function main()
