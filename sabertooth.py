#!/usr/bin/python

'''
sabertooth.py - This script provides a REPL to allow a user to send commands
to a Sabertooth 2x25 motor driver.

RPI Rock Raiders
6/10/15

Last Updated: Bryant Pong: 6/10/15 - 4:20 PM     
'''

while True:
	nextCmd = str(raw_input("Please enter your next command.  Commands are in <addr> <cmd> <data>: ")).split()
	print("Your command (serialized) is: " + str(nextCmd))

	chksum = (int(nextCmd[0]) + int(nextCmd[1]) + int(nextCmd[2])) & 127
	
	print("Your command is: " + str(nextCmd[0]) + " " + str(nextCmd[1]) + \
	    " " + str(nextCmd[2]) + " " + str(chksum))	 
