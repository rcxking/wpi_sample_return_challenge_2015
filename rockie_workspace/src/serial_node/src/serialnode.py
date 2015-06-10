#!/usr/bin/python

'''
serialnode.py - This node communicates with the PSOC and provides services
to send to the PSOC.

RPI Rock Raiders
6/4/15

Last Updated: Bryant Pong: 6/10/15 - 10:40 AM
'''

# Python Imports:
import rospy
import serial 
from serial_node.srv import * 

# Serial object:
serial = serial.Serial("/dev/ttyS0", 115200, 8, 'N', 1) 

'''
This is a helper function to send serial data to the PSOC:  
'''
def writeData(data):
	serial.write(str(data))
	resp = serial.readline()	

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

	# Send the velocities:
	writeData("1-"+str(frontLeft))
	writeData("2-"+str(frontRight))
	writeData("3-"+str(backLeft))
	writeData("4-"+str(backRight))

'''
This service controls the steering of the robot:

This service uses the custom service "Steer.srv".         
'''	   
def steer_service(req):

	# Steer the robot (true = steer):
	if req.turned:
		# TODO: Turn the robot
		return
	else:
		# TODO: Straighten the robot 					   
		return

'''
This service controls the pause service of the robot:

This service uses the custom service "Pause.srv".
'''
def pause_service(req):
	if req.paused:
		writeData("0-0")
		resp = writeData("13-1") 


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
	gripperService = rospy.Service("gripperservice", GripperService, gripper_service)
	driveService = rospy.Service("driveservice", WheelVel, drive_service)
	steerService = rospy.Service("steerservice", SteerService, steer_service)
	lightsService = rospy.Service("lightsservice", Lights, lights_service)

	rospy.spin()
	
if __name__ == "__main__":
	serial_server()

