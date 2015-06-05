#!/usr/bin/python

'''
serialnode.py - This node communicates with the PSOC and provides services
to send to the PSOC.

RPI Rock Raiders
6/4/15

Last Updated: Bryant Pong: 6/4/15 - 10:33 PM 
'''

# Python Imports:
import rospy
import serial 
from serial_node.srv import * 

# Serial object:
serial = None

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
	else:
		# TODO: Open the gripper

'''
This service sends motor velocity commands to all four motors of the chassis.

This service expects the custom service "wheelvel.srv".
'''
def drive_service(req):
	# Get the 

def serial_server():
	rospy.init_node("serial_node_server")
	# Start all services:
	armService = rospy.Service("armservice", ArmService, arm_service) 	
	


if __name__ == "__main__":

	global serial

	try:
		serial = serial.Serial("/dev/ttyS0", 115200, 8, 'N', 1)
	except serial.SerialException:
		rospy.loginfo("Serial Node: ERROR in creating serial object to PSOC") 
		return

	serial_server()

