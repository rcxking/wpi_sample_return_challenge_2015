#!/usr/bin/python

'''
drive.py - This file contains functions to interface with the        
'''

# Python Imports:
import serial
import rospy
from driving.srv import Drive

'''
Create a Serial object to talk to the Arduino.

Parameters:
Baud: 9600
Parity: None
Stop Bits: 1
'''
ser = serial.Serial("/dev/ttyACM0", 115200) 

'''
Functions to control the motors:   
'''
def stop(motorNum):
	ser.write(str(motorNum) + "-0")

def forward(motorNum, powerLvl):
	ser.write(str(motorNum) + "-" + str(powerLvl)) 			

def drive_handler(req):
	print("Sending out command: " + str(req.cmd)) 	
	
	numBytes = ser.write(str(req.cmd))

	if numBytes < len(str(req.cmd)):
		print("ERROR: did not send out all data")
		return False

	# Read the response back from the Arduino:
	resp = ser.readline()
	print("Arduino responds with: " + str(resp))
	
	return True	  

def drive():
		
	# Initialize ROS:
	rospy.init_node("drivenode")
	s = rospy.Service('drive_service', Drive, drive_handler)
	print("Ready to send driving commands:") 
	rospy.spin()
	
if __name__ == "__main__":
	drive() 
