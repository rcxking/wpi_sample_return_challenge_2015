#!/usr/bin/python

'''
serialnode.py - This node communicates with the PSOC and provides services
to send to the PSOC.

RPI Rock Raiders
6/4/15

Last Updated: Bryant Pong: 6/11/15 - 2:20 PM
'''

# Python Imports:
import rospy
import time
import serial 
from serial_node.srv import * 

# Serial objects:
ARDUINOPORT = "/dev/ttyACM1"
ARDUINOBAUD = 9600

MOTORPORT = "/dev/ttyUSB0"
MOTORBAUD = 9600 

arduinoserial = serial.Serial(ARDUINOPORT, ARDUINOBAUD, 8, 'N', 1) 
motorserial = serial.Serial(MOTORPORT, MOTORBAUD, 8, 'N', 1)

'''
This is a helper function to send commands directly to the Sabertooth
motor drivers:

Data is in the format:
1) address
2) command
3) data

'''
def writeData(addr, cmd, data):
	chksum = (addr+cmd+data) & 127
	motorserial.write(chr(addr))
	motorserial.write(chr(cmd))
	motorserial.write(chr(data))
	motorserial.write(chr(chksum))

# cmd is an INTEGER 
def writeArduinoData(cmd):
	arduinoserial.write(chr(cmd))
	# Get the response back:
	return arduinoserial.readline()

'''
This service sends motor velocity commands to all four motors of the chassis.

This service expects the custom service "wheelvel.srv".
'''
def drive_service(req):
	# Get the target velocities of the motors:
	vels = [int(req.front_left*127), int(req.front_right*127), int(req.rear_right*127)]
	dirs = [4 if vel >=0 else 5 for vel in vels]
	motors = [128, 129, 130]

	# Send the velocities.  Back left motor is dead:
	for i in xrange(3):
		writeData(motors[i], dirs[i], abs(vels[i])) 

	return True
	
'''
This service controls the steering of the robot:

This service uses the custom service "Steer.srv".         
'''	   
def steer_service(req):

	# Steer the robot (true = steer):
	if req.turned:
		# Turn Left
		writeData(135, 0, 100)
		writeData(135, 4, 100)

		leftMoving, rightMoving = True, True
		while leftMoving or rightMoving:
			time.sleep(0.0001)
			# Left and Right Limit Switches:
			limitSwitches = int(writeArduinoData(42)[0].encode('hex'), 16)
			if limitSwitches & 0x08 == 8 and limitSwitches & 0x04 == 0 and rightMoving == True:
				writeData(135, 4, 0)
				rightMoving = False
			
			if limitSwitches & 0x02 == 2 and limitSwitches & 0x01 == 0 and leftMoving == True:
				writeData(135, 0, 0)
				leftMoving = False			
	else:
		# TODO: Straighten the robot 	
		writeData(135, 1, 100)
		writeData(135, 5, 100)

		leftMoving, rightMoving = True, True
		while leftMoving or rightMoving:
			time.sleep(0.0001)
			limitSwitches = int(writeArduinoData(42)[0].encode('hex'), 16)
			if limitSwitches & 0x08 == 0 and limitSwitches & 0x04 == 4 and rightMoving == True:
                                writeData(135, 5, 0)
                                rightMoving = False

                        if limitSwitches & 0x02 == 0 and limitSwitches & 0x01 == 1 and leftMoving == True:
                                writeData(135, 1, 0)
                                leftMoving = False
		   
	return True

'''
This service controls the pause service of the robot:

This service uses the custom service "Pause.srv".
'''
def pause_service(req):
	rospy.loginfo("req.paused: " + str(req.paused))
	if req.paused:
		resp = writeArduinoData(43)
		if resp != 100:
			return False
	else:
		resp = writeArduinoData(44)	
		if resp != 100:
			return False 
	return True

'''
This service controls the red and green lights.

This service uses the custom service "Lights.srv"  
'''
def lights_service(req):
	if req.light == 0:
		# Green light:
		if req.on:
			x = writeArduinoData(45)
		else:
			x = writeArduinoData(46)
	else:
		# Red Light
		if req.on:
			x = writeArduinoData(47)
		else:
			x = writeArduinoData(48)

	return True

def serial_server():
	rospy.init_node("serial_node_server")
	# Start all services:
	driveService = rospy.Service("driveservice", WheelVel, drive_service)
	steerService = rospy.Service("steerservice", Steer, steer_service)
	pauseService = rospy.Service("pauseservice", Pause, pause_service)
	lightsService = rospy.Service("lightsservice", Lights, lights_service) 

	rospy.spin()
	
if __name__ == "__main__":
	serial_server()

