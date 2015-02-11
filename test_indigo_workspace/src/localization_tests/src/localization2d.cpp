/*
 * localization2d.cpp
 *
 * This program performs localization on a 2D world found in "world.txt".
 *
 * RPI Rock Raiders
 * 1/8/15
 *
 * Last Updated: Bryant Pong: 1/30/15 - 6:39 PM 
 */

// C Headers
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// C++ Headers
#include <vector>

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

// This array holds the different directions the robot can face:
const char* directions[8] = {"north", "northeast", "east", "southeast", "south", "southwest", "west", "northwest"};
const int numDirections = 8;

// This array holds the command history:
char **commandHistory;   
const int commandHistorySize = 9001;
int currentCommand = 0;

/** END SECTION GLOBAL VARIABLES **/

/** Function Prototypes **/
void printWorld(void);
void outputCurrentLocation(void);
void getMove(void);
bool isValidMove(char **command);
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

bool isValidMove(char **command) {

	/*
	 * Next, look at the first command and determine the appropriate action to
	 * take.  The appropriate commands are:
	 *
	 * 1) forward <number of spaces to travel>
	 * 2) backward <number of spaces to travel>
	 * 3) left <increments of 45 degrees>
	 * 4) right <increments of 45 degrees>  
	 */


	if(strcmp(command[0], "forward") == 0) {
#ifdef DEBUG
		printf("DEBUG ONLY - FORWARD COMMAND\n");
#endif

		// A forward command's 2nd argument contains the # of spaces to move forward:
		const int forwardSpaces = atoi(command[1]);   

	} else if(strcmp(command[0], "backward") == 0) {
#ifdef DEBUG
		printf("DEBUG ONLY - BACKWARD COMMAND\n");
#endif

	} else if(strcmp(command[0], "left") == 0) {
#ifdef DEBUG
		printf("DEBUG ONLY - TURN LEFT COMMAND\n");
#endif

		/*
		 * There are 8 different directions for turning:
		 *
		 * 1) North:
		 * 2) Northeast
		 * 3) East
		 * 4) Southeast
		 * 5) South
		 * 6) Southwest
		 * 7) West
		 * 8) Northwest
		 *
		 * This statement determines what direction the robot now faces after turning left:
		 */
		int curDirection = -1;

		if(strcmp(perDir, "north") == 0) {
			
		} else if(strcmp(perDir, "northeast") == 0) {
			
		} else if(strcmp(perDir, "east") == 0) {
		} else if(strcmp(perDir, "southeast") == 0) {
		} else if(strcmp(perDir, "south") == 0) {
		} else if(strcmp(perDir, "southwest") == 0) {
		} else if(strcmp(perDir, "west") == 0) {
		} else if(strcmp(perDir, "northwest") == 0) {
		} else {
			// We should never reach here; this indicates that the direction was corrupted:
			return false;
		} // End if-elseif-else
	
		// Turning left will always succeed:
		return true;

	} else if(strcmp(command[0], "right") == 0) {
#ifdef DEBUG
		printf("DEBUG ONLY - TURN RIGHT COMMAND\n");
#endif

		// Turning right will always succeed:
		return true;

	} else {
		return false;
	} // End if-elseif-else
}

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

	// Commands are delimited by a space:
	const char *delim = " ";

	// These pointers store the next part of the command:	
	char *nextPart;
	char *save;

	// This array temporarily holds the command tokens:
	char **tempCmdArray = (char **) calloc(sizeof(char *), 5);
	for(i = 0; i < 5; i++) {
		tempCmdArray[i] = (char *) calloc(sizeof(char), 20);
	} // End for
	int nextTempCmd = 0;

	for(nextPart = strtok_r(userInput, delim, &save); nextPart; 
			nextPart = strtok_r(NULL, delim, &save)) {
#ifdef DEBUG
		printf("DEBUG ONLY - Next Token is: %s\n", nextPart);
#endif
		strcpy(tempCmdArray[nextTempCmd], nextPart);
		nextTempCmd++;
	} // End for  

	// DEBUG ONLY - Print out the temporary command array:
#ifdef DEBUG
	printf("DEBUG ONLY - Now printing out tempCmdArray:\n");
	for(i = 0; i < nextTempCmd; i++) {
		printf("%s\n", tempCmdArray[i]);
	} // End for
#endif

	/*
	 * Next, look at the first command and determine the appropriate action to
	 * take.  The appropriate commands are:
	 *
	 * 1) forward <number of spaces to travel>
	 * 2) backward <number of spaces to travel>
	 * 3) left <increments of 45 degrees>
	 * 4) right <increments of 45 degrees>  
	 */

	if(strcmp(tempCmdArray[0], "forward") == 0) {
#ifdef DEBUG
		printf("DEBUG ONLY - FORWARD COMMAND\n");
#endif

	} else if(strcmp(tempCmdArray[0], "backward") == 0) {

	} else if(strcmp(tempCmdArray[0], "left") == 0) {

	} else if(strcmp(tempCmdArray[0], "right") == 0) {

	} else {

	} // End if-elseif-else

	// Good policy to free memory:
	for(i = 0; i < 5; i++) {
		free(tempCmdArray[i]);
	} // End for
	free(tempCmdArray);

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

	// Allocate memory for the command history:
	commandHistory = (char **) calloc(sizeof(char *), 1024);
	for(i = 0; i < commandHistorySize; i++) {
		commandHistory[i] = (char *) calloc(sizeof(char), 20);
	} // End for


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

		// If the user inputs "history", display the command history:
		if(strcmp(userInput, "history\n") == 0) {
			printf("Command History\n");
			for(i = 0; i < currentCommand; i++) {
				printf("%s", commandHistory[i]);
			} // End for
		} else {
			// Otherwise:

			// Add the command to the commandHistory:
			strcpy(commandHistory[currentCommand], userInput);

			// Parse the next move and execute it if the move is valid:
			executeMove();

			// Increment the currentCommand counter:
			currentCommand++;
		} // End if-else
	} // End while

	printf("Goodbye!\n");

	// Good policy to deallocate dynamically allocated memory:
	free(currDir);
	free(startingDirection);
	for(i = 0; i < worldHeight; i++) {
		free(world[i]);
	} // End for
	free(world);
	for(i = 0; i < commandHistorySize; i++) {
		free(commandHistory[i]);
	} // End for
	free(commandHistory);
	free(userInput);

	return 0;
} // End function main()
