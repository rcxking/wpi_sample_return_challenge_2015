#!/usr/bin/python

'''
serialnode.py - This node communicates with the PSOC and provides services
to send to the PSOC.

RPI Rock Raiders
6/4/15

Last Updated: Bryant Pong: 6/11/15 - 10:51 AM
'''

# Python Imports:
import rospy
import serial 
from serial_node.srv import * 

# Serial objects:
ARDUINOPORT = "/dev/ttyACM0"
ARDUINOBAUD = 9600

MOTORPORT = "/dev/ttyUSB0"
MOTORBAUD = 9600 

arduinoserial = serial.Serial(ARDUINOPORT, ARDUINOBAUD, 8, 'N', 1) 
motorserial = serial.Serial(MOTORPORT, MOTORBAUD, 8, 'N', 1)

'''
Addressing of the robot's peripherals: 
'''
addrs = [128, 129, 130, 131] 

'''
This is a helper function to send commands directly to the Sabertooth
motor drivers:

Data is in the format:
1) address
2) command
3) data
4) checksum 
'''
def writeData(addr, cmd, data):
	serial.write(chr(addr))
	serial.write(chr(cmd))
	serial.write(chr(data))

	chksum = (addr+cmd+data) & 127

	serial.write(chr(chksum))

'''
This service calls the PSOC and sets the joint angles of
the arm.      

This service expects the custom service "armservice.srv" 
'''
def arm_service(req):
	# Extract the desired joint angles:
	armRevolute = req.revolute
	armPrismatic = req.prismatic
	armGripper = req.gripper 	 

	# TODO: Send the serial requests to the PSOC   

'''
This service sends the PSOC an open/close gripper command.

This service expects the custom service "gripper.srv" 
'''
def gripper_service(req):
	
	# Open the gripper:
	if req.close:
		# TODO: close the gripper
		return
	else:
		# TODO: Open the gripper
		return 

'''
This service sends motor velocity commands to all four motors of the chassis.

This service expects the custom service "wheelvel.srv".
'''
def drive_service(req):
	# Get the target velocities of the motors:
	frontLeft = req.vel1
	frontRight = req.vel2
	backLeft = req.vel3
	backRight = req.vel4

	# Send the velocities.  Back left motor is dead:
	
	
'''
This service controls the steering of the robot:

This service uses the custom service "Steer.srv".         
'''	   
def steer_service(req):

	# Steer the robot (true = steer):
	if req.turned:
		# TODO: Turn the robot
		return True
	else:
		# TODO: Straighten the robot 					   
		return True

'''
This service controls the pause service of the robot:

This service uses the custom service "Pause.srv".
'''
def pause_service(req):
	rospy.loginfo("req.paused: " + str(req.paused))
	if req.paused:
		writeData("13-1\n")
	else:
		writeData("13-0\n")	
	return True

'''
This service turns on or off any of the 3 lights on the light beacon.

This service uses the custom service "Lights.srv"      
'''
def lights_service(req):
	
	'''
	Lights mapping:
	5 - Red - Send 0
	6 - Greeh - Send 1
	'''	

	if req.light == 0:
		# Red Light:
		if req.on:
			writeData("5-127")
		else:
			writeData("5-0")
	else:
		# Green Light:
		if req.on:
			writeData("7-127")
		else:
			writeData("7-0")

def serial_server():
	rospy.init_node("serial_node_server")
	# Start all services:
	armService = rospy.Service("armservice", ArmService, arm_service) 	
	gripperService = rospy.Service("gripperservice", Gripper, gripper_service)
	driveService = rospy.Service("driveservice", WheelVel, drive_service)
	steerService = rospy.Service("steerservice", Steer, steer_service)
	lightsService = rospy.Service("lightsservice", Lights, lights_service)
	pauseService = rospy.Service("pauseservice", Pause, pause_service)

	rospy.spin()
	
if __name__ == "__main__":
	serial_server()

