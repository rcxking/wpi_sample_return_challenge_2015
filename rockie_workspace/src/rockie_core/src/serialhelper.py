#!/usr/bin/python

'''
serial.py - This file contains functions to handle communicating with the PSOC
board responsible for the embedded control side of the robot.

RPI Rock Raiders
5/31/15

Last Updated: Bryant Pong: 6/2/15 - 1:34 PM
'''

# Python includes:
import serial

'''
This function creates a Serial object to connect to the PSOC.

Parameters:
Baud Rate: 115200
Data Bits: 8
Parity: None
Stop Bits: 1
'''
def createSerial(port):

	serialObject = -1

	try:
		serialObject = serial.Serial(port, 115200, 8, 'N', 1) 
	except serial.SerialException:
		print("Error in creating serial connection to PSOC.  Maybe the port is wrong?") 
	return serialObject

'''
Movement commands to the PSOC:

Supported Commands:
'''
 

