/*
 * localization2d.cpp
 *
 * This program performs localization on a 2D world found in "world.txt".
 *
 * RPI Rock Raiders
 * 1/8/15
 *
 * Last Updated: Bryant Pong: 1/9/15 - 11:37 PM 
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../include/kalmanfilter.h"

/** Constants **/

/*
 * These variables control the probability that the robot has "slipped"; i.e.
 * the robot did not travel in the direction that it was supposed to. 
 */
const float pMovesGCommand = 1.0;
const float pNoMovesGCommand = 0.0;

// The number of characters to read in for the next input:
size_t nextInputLength = 100;

/** END SECTION CONSTANTS **/

/** Global Variables **/

// The dimensions of the world map:
int worldLength, worldHeight;

// The starting X and Y location of the robot:
int startX, startY;  

// The starting direction of the robot:
char *startingDirection;  

// The current pose variables of the robot:
int currX, currY;
char *currDir;

// If the robot had perfect movements, the pose is here:
int perX, perY;
char *perDir;  

// This 2D array contains a representation of the world:
char **world;

// Looping Variables:
int i, j;

// This buffer holds the user input:
char *userInput = (char *) malloc(sizeof(int) * 100); 

/** END SECTION GLOBAL VARIABLES **/

/** Function Prototypes **/
void printWorld(void);
void outputCurrentLocation(void);
void getMove(void);
void executeMove(void);
/** END SECTION FUNCTION PROTOTYPES **/

/*
 * This function prints out the world.
 */
void printWorld(void) {
	for(i = 0; i < worldHeight; i++) {
		printf("%s", world[i]);
	} // End for
} // End function printWorld()

/*
 * This function outputs the current location of the robot.
 * The world follows the following coordinate system:
 *
 * (0, height)      (length, height)
 * -------------------
 * |                 |
 * |                 |
 * -------------------
 * (0, 0)            (length, 0)   
 */
void outputCurrentLocation(void) {

	for(i = 0; i < worldHeight; i++) {
		for(j = 0; j < worldLength; j++) {

			// Output a "C" for the current location of the robot:   
			if( ((worldHeight - 1 - i) == startY) && (j == startX)) {
				printf("C");
			} else {
				printf("%c", world[i][j]);
			} // End if-else
		} // End for
		printf("\n");
	} // End for

} // End function outputCurrentLocation()

/*
 * This function gets the next move from the user and stores it into the
 * global string userInput.
 */
void getMove(void) {
	printf("Please enter your next move: \n");
	int bytesRead = getline(&userInput, &nextInputLength, stdin);
	if(bytesRead == -1) {
		fprintf(stderr, "ERROR in getMove(): getline failed\n");
		exit(1);
	} // End if

#ifdef DEBUG
	printf("DEBUG ONLY - You Entered: %s\n", userInput);
#endif
} // End function getMove()

/*
 * Given the user input, execute the move if valid.
 *
 * The valid moves are:
 * 1) forward <squares to move>
 * 2) backward <squares to move>
 * 3) turn left (turns 90 degrees counterclockwise)
 * 4) turn right (turns 90 degrees clockwise)      
 */
void executeMove(void) {
	/*
	 * First, we need to parse the command.  All commands are two part commands
	 * and will be parsed using the strtok() function. 
	 */
	
	// This array will hold the next part of the command:
	char *token = (char *) malloc(sizeof(char) * 20);

	

	// Free any dynamically allocated memory:
	free(token);
} // End function executeMove()

// Main function:
int main(int argc, char **argv) {

	// First read in the text file containing the world:  
	FILE *ptr_world;
	char readbuffer[1024];

	// Read in the world from the text file: 
	ptr_world = fopen("world.txt", "r");
	if(!ptr_world) {
		fprintf(stderr, "ERROR in reading world.txt\n");
		exit(1);
	} // End if

	/*
	 * The first two lines in world.txt give the length and the width of the
	 * world.  The third and fourth line tell the initial pose of the robot
	 * (X, Y, N/E/S/W).    
	 */

	startingDirection = (char *) malloc(sizeof(char) * 10);

	if(fgets(readbuffer, 1024, ptr_world) == NULL) {
		fprintf(stderr, "ERROR in reading world.txt line 1\n");
		exit(1);
	} // End if 

	// Get the world's length:
	worldLength = atoi(readbuffer);

	if(fgets(readbuffer, 1024, ptr_world) == NULL) {
		fprintf(stderr, "ERROR in reading world.txt line 2\n");
		exit(1);
	} // End if

	// Get the world's height:
	worldHeight = atoi(readbuffer);

	if(fgets(readbuffer, 1024, ptr_world) == NULL) {
		fprintf(stderr, "ERROR in reading world.txt line 3\n");
		exit(1);
	} // End if

	// Get the robot's starting X location:
	startX = atoi(readbuffer);

	if(fgets(readbuffer, 1024, ptr_world) == NULL) {
		fprintf(stderr, "ERROR in reading world.txt line 4\n");
		exit(1);
	} // End if

	// Get the robot's starting Y location:
	startY = atoi(readbuffer);

	if(fgets(readbuffer, 1024, ptr_world) == NULL) {
		fprintf(stderr, "ERROR in reading world.txt line 5\n");
		exit(1);
	} // End if

	// Get the robot's starting direction:
	strcpy(startingDirection, readbuffer); 

#ifdef DEBUG
	printf("DEBUG ONLY: Now printing out starting parameters:\n");
	printf("World Length: %d\n", worldLength);
	printf("World Height: %d\n", worldHeight);
	printf("Robot Starting X Location: %d\n", startX);
	printf("Robot Starting Y Location: %d\n", startY);
	printf("Robot Starting Direction: %s\n", startingDirection);
#endif

	// Allocate memory for the world array:
	world = (char **) malloc(sizeof(char *) * worldHeight);
	for(i = 0; i < worldHeight; i++) {
		world[i] = (char *) malloc(sizeof(char) * worldLength);
	} // End for

	for(i = 0; i < worldHeight; i++) {
		if(fgets(readbuffer, 1024, ptr_world) == NULL) {
			fprintf(stderr, "ERROR in reading from world.txt - map\n");
			exit(1);
		} // End if

#ifdef DEBUG
		printf("%s", readbuffer);
#endif
		strcpy(world[i], readbuffer);

	} // End for

	// Good policy to close the reading buffer: 
	fclose(ptr_world);

#ifdef DEBUG
	printf("DEBUG ONLY: Now printing out starting parameters:\n");
	printf("World Length: %d\n", worldLength);
	printf("World Height: %d\n", worldHeight);
	printf("Robot Starting X Location: %d\n", startX);
	printf("Robot Starting Y Location: %d\n", startY);
	printf("Robot Starting Direction: %s\n", startingDirection);
	printf("World is:\n");
	printWorld();
#endif

	// Now that we have the world, create a REPL to allow the user to "move" the robot:
	printf("Now starting Robot Control REPL: \n"); 
	strcpy(userInput, "");

	// Initialize the robot's current pose to the starting variables:
	currDir = (char *) malloc(sizeof(char) * 10);
	strcpy(currDir, startingDirection);
	currX = startX;
	currY = startY;

	// Initialize the robot's "perfect" pose to the starting variables:
	perDir = (char *) malloc(sizeof(char) * 10);
	strcpy(perDir, startingDirection);
	perX = startX;
	perY = startY;
	
	// Have the user quit the REPL using "q" or "Q":
	while(((strcmp(userInput, "q\n") != 0) && 
		   (strcmp(userInput, "Q\n") != 0))) {

		// First output where the robot SHOULD be:
		printf("I should be at the following pose:\n");
		printf("(%d, %d) facing: %s\n", perX, perY, perDir);

		// Secondly, output the current position of the robot:
		outputCurrentLocation();

		// Next, get the next move from the user:
		getMove();  

		// Parse the next move and execute it if the move is valid:
		executeMove();

	} // End while

	printf("Goodbye!\n");

	// Good policy to deallocate dynamically allocated memory:
	free(currDir);
	free(startingDirection);
	for(i = 0; i < worldHeight; i++) {
		free(world[i]);
	} // End for
	free(world);
	free(userInput);

	return 0;
} // End function main()
