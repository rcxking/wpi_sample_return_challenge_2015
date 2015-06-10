#!/usr/bin/python

'''
xbee.py - This node listens for the XBee on the Pause Button for a
Pause signal and publishes a String containing the status of the Pause.

RPI Rock Raiders
6/9/15

Last Updated: Bryant Pong: 6/9/15 - 2:24 AM            
'''

# Python Imports
import serial
import rospy
from std_msgs.msg import String

# Globals:
PORT = "/dev/ttyUSB2"
BAUD = 9600
TIMEOUT = 0.25

# Run the ROS Node:
def xbeenode():

	ser = None

	try:
		ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
	except serial.SerialException:
		rospy.loginfo("ERROR: xbee.py: Cannot create serial port object! \
		Have you checked if PORT is set correctly?")
		return

	pub = rospy.Publisher("pause", String, queue_size=1)
	rospy.init_node("pauselistener")
	rate = rospy.Rate(200)

	while not rospy.is_shutdown():

		nextData = ser.read()
		if str(nextData) != '':
			rospy.loginfo("No pause Command Received")
			pub.publish("no")
		else:
			rospy.loginfo("Yes Pause Received")
			pub.publish("yes")
		
		rate.sleep()

if __name__ == "__main__":
	try:
		xbeenode()
	except rospy.ROSInterruptException:
		pass

