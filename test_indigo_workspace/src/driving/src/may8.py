#!/usr/bin/python

'''
may8.py - Script to move the robot for May 8th deadline.

Tasks:
1) Drive forward for at least 5 meters
2) Pick up the precached sample
3) Drive backwards for at least 5 meters

RPI Rock Raiders
5/6/15

Last Updated: Bryant Pong: 5/7/15 - 2:40 PM       
'''

# Python imports:
import rospy
import time
from driving.srv import *

# Callback to communicate with the drive_service service:
def drive_callback(cmd, motor, pwr):
	rospy.wait_for_service("drive_service")
	try:
		drive_service = rospy.ServiceProxy("drive_service", Drive)
		resp = drive_service(cmd, motor, pwr)
		return True
	except rospy.ServiceException, e:
		print("Service call failed: %s" % e)

# Main function:
def main():
	# Call the callback to drive and pick up the sample:

	#for i in xrange(100, 125, 5):
	drive_callback("startall", 0, 70)
	time.sleep(25)
	drive_callback("stopall", 0, 0)
	drive_callback("down", 0, 0)
	drive_callback("forward", 13, 1)
	drive_callback("up", 0, 0)
	#for i in xrange(90, 120, 5):
	drive_callback("startall", 0, -70)
	time.sleep(20)
	drive_callback("stopall", 0, 0)
	drive_callback("forward", 13, 0)

# Main function runner:
if __name__ == "__main__":
	main()
