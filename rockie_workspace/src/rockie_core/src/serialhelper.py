#!/usr/bin/python

'''
serial.py - This file contains functions to handle communicating with the PSOC
board responsible for the embedded control side of the robot.

RPI Rock Raiders
5/31/15

Last Updated: Bryant Pong: 5/31/15 - 4:24 PM    
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
	return serial.Serial(port, 115200, 8, 'N', 1) 


