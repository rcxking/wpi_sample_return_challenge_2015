#!/usr/bin/python

'''
xbee.py - This node listens for the XBee on the Pause Button for a
Pause signal and publishes a String containing the status of the Pause.

RPI Rock Raiders
6/9/15

Last Updated: Bryant Pong: 6/9/15 - 9:25 AM            
'''

# Python Imports
import serial
import rospy
from std_msgs.msg import String

# Globals:
PORT = "/dev/ttyUSB0"
BAUD = 115200 
TIMEOUT = 0.25

ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)

# Run the ROS Node:
def xbeenode():

	pub = rospy.Publisher("pause", String, queue_size=10)
	rospy.init_node("pauselistener")
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():

		nextData = ser.read()
		if str(nextData) != '':
			rospy.loginfo("Pause Command Received")
			pub.publish("yes")
		else:
			rospy.loginfo("No Pause Received")
			pub.publish("no")
		
		rate.sleep()

if __name__ == "__main__":
	try:
		xbeenode()
	except rospy.ROSInterruptException:
		pass

