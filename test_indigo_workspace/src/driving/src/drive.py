#!/usr/bin/python

'''
drive.py - This file contains functions to interface with the Arduino        
'''

# Python Imports:
import serial
import rospy
import time
from driving.srv import Drive

'''
Create a Serial object to talk to the Arduino.

Parameters:
Baud: 115200
Parity: None
Stop Bits: 1
'''
ser = serial.Serial("/dev/ttyACM0", 115200) 

'''
Functions to control the motors:   

Motor Pinouts:
0 - Front Left Motor
1 - Rear Left Motor
2 - Front Right Motor
3 - Rear Right Motor
'''
def stop(motorNum):
	ser.write(str(motorNum) + "-0")

	# Read the response back from the Arduino:
	resp = ser.readline()
	print("Arduino responds with: " + str(resp))
	resp = ser.readline()
        print("Arduino responds with: " + str(resp))


def forward(motorNum, powerLvl):
	ser.write(str(motorNum) + "-" + str(powerLvl)) 			

	# Read the response back from the Arduino:
	resp = ser.readline()
	print("Arduino responds with: " + str(resp))
	resp = ser.readline()
        print("Arduino responds with: " + str(resp))

def down():
	ser.write("1--50")
	resp = ser.readline()
	print("Arduino responds with : " + str(resp))
	resp = ser.readline()
        print("Arduino responds with: " + str(resp))
	time.sleep(8)
	ser.write("1-0")
        resp = ser.readline()
        print("Arduino responds with : " + str(resp))
        resp = ser.readline()
        print("Arduino responds with: " + str(resp))

def up():
        ser.write("1-50")
        resp = ser.readline()
        print("Arduino responds with : " + str(resp))
        resp = ser.readline()
        print("Arduino responds with: " + str(resp))
        time.sleep(8)
        ser.write("1-0")
        resp = ser.readline()
        print("Arduino responds with : " + str(resp))
        resp = ser.readline()
        print("Arduino responds with: " + str(resp))


def stopall():
	'''
	request = "";
	for i in xrange(7):
		request += (str(i) + "-0,")
	request += "7-0"
	'''

	request = "0-0,1-0"

	ser.write(request)

	for i in xrange(4):
		resp = ser.readline()
		print("Arduino responded with: " + str(resp))

def startall(motorPwr):

	ser.write("0-"+str(motorPwr))
	#ser.write("0-"+str(motorPwr)+",1-"+str(motorPwr)+",2-"+str(motorPwr)+\
	#          ",3-"+str(motorPwr))

	for i in xrange(2):
		resp = ser.readline()
		print("Arduino responded with: " + str(resp))

def drive_handler(req):
	print("Sending out command: " + str(req.cmd)) 	

	# Determine which command to send:
	if req.cmd == "startall":
		startall(req.power)
	elif req.cmd == "stopall":
		stopall()
	elif req.cmd == "forward":
		forward(req.motor, req.power)
	elif req.cmd == "stop":
		stop(req.motor)
	elif req.cmd == "up":
		up()
	elif req.cmd == "down":
		down()
			 	  	
	return True	  

def drive():
		
	# Initialize ROS:
	rospy.init_node("drivenode")
	s = rospy.Service('drive_service', Drive, drive_handler)
	print("Ready to send driving commands:") 
	rospy.spin()
	
if __name__ == "__main__":
	drive() 
