#!/usr/bin/python

'''
State machine to run for stage 1.

RPI Rock Raiders
5/31/15

Last Updated: Bryant Pong: 6/2/15 - 1:23 PM
'''

# ROS Libraries:
import roslib
import rospy

# Finite State Machine Libraries
import smach
import smach_ros

# For OpenCV:
import cv2
import numpy as np

# For Neural Network / Object Recognition:
import theano
import theano.tensor as T
import lasagne

# Operating System / Data Libraries:
import os
import cPickle as pickle

# Serial Communications to PSOC:
import serial
import serialhelper

# Miscellaneous Libraries:
import warnings
import matplotlib.pyplot as plt

'''
Global Objects:
'''
# Serial Object:
serialObj = None

# Neural Network Objects:
l_in = None
l_conv1 = None
l_pool1 = None
l_conv2 = None
l_pool2 = None
l_conv3 = None
l_conv4 = None
l_pool3 = None



# Global flag for whether the sample has been found:
sampleFound = False  

'''
This state performs system checks on the robot before beginning the run.   
'''
class StartupSequence(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=["egressEnter", "serialFailure"])    

	def execute(self, userdata):
		rospy.loginfo("Executing Startup Sequence") 

		# Create the Serial Port to communicate with the PSOC:
		rospy.loginfo("Now creating PSOC Serial Object") 
		global serialObj
		serialObj = serialhelper.createSerial("/dev/ttyUSB0") 
		if serialObj != -1:
			rospy.loginfo("Successfully created PSOC Serial Object")		
		else:
			return "serialFailure" 

		# 
		#nnparams = pickle.load(open("nnparams.dat", "rb"))	 

		# Create the neural network:
			  
		return "egressEnter"

'''
In the event of a serial port exception raised, this state provides some 
advice on looking for debugging issues:    
'''
class SerialFailure(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=["end"])

	def execute(self, userdata):
		rospy.loginfo("It looks like I could not connect to the PSOC")
		rospy.loginfo("Ensure that the following parameters have been set correctly:")
		rospy.loginfo("1) Is the Serial port set correctly?  (i.e. /dev/ttyUSB0")
		rospy.loginfo("2) Baud Rate: 115200")
		rospy.loginfo("3) Data Bits: 8")
		rospy.loginfo("4) Parity: None")
		rospy.loginfo("5) Stop Bits: 1")  
		    
		return "end"

class Egress(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=["transitEnter"])

	def execute(self, userdata):
		rospy.loginfo("Executing Egress")
		return "transitEnter"

class Transit(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=["searchEnter"])
		
	def execute(self, userdata):
		rospy.loginfo("Executing Transit")
		return "searchEnter"
		
class Search(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=["searchTransitEnter"])

	def execute(self, userdata):
		rospy.loginfo("Executing Search")
		return "searchTransitEnter"

class SearchTransit(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=["sampleRecogEnter"])
	
	def execute(self, userdata):
		rospy.loginfo("Executing Search Transit")
		return "sampleRecogEnter"
		
class SampleRecognition(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=["sampleNotFound", "sampleFound"]) 			

	def execute(self, userdata):
		rospy.loginfo("Executing Sample Recognition")

		global sampleFound

		# Has the sample been found?
		if sampleFound:  
			return "sampleFound"
		else:
			return "sampleNotFound"

class RetrieveSample(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=["endTransitEnter"])
	
	def execute(self, userdata):
		rospy.loginfo("Executing Retrieve Sample")
		return "endTransitEnter"

class EndTransit(smach.State);
	def __init__(self):
		smach.State.__init__(self, outcomes=["end"])
	
	def execute(self, userdata):
		rospy.loginfo("Executing End Transit")
		return "end" 

def main():
	rospy.init_node('rockie_state_machine')

	sm = smach.StateMachine(outcomes=['complete'])
	with sm:
		smach.StateMachine.add("STARTUPSEQUENCE", StartupSequence(), transitions={"egressEnter":"EGRESS"})
		smach.StateMachine.add("SERIALFAILURE", SerialFailure(), transitions={"end":"complete"})
		smach.StateMachine.add("EGRESS", Egress(), transitions={"transitEnter":"TRANSIT"})
		smach.StateMachine.add("TRANSIT", Transit(), transitions={"searchEnter":"SEARCH"})
		smach.StateMachine.add("SEARCH", Search(), transitions={"searchTransitEnter":"SEARCHTRANSIT"})
		smach.StateMachine.add("SEARCHTRANSIT", SearchTransit(), transitions={"sampleRecogEnter":"SAMPLERECOGNITION"})
		smach.StateMachine.add("SAMPLERECOGNITION", SampleRecognition(), transitions={"sampleNotFound":"SEARCH","sampleFound":"RETRIEVESAMPLE"})
		smach.StateMachine.add("RETRIEVESAMPLE", RetrieveSample(), transitions={"endTransitEnter":"ENDTRANSIT"})
		smach.StateMachine.add("ENDTRANSIT", EndTransit(), transitions={"end":"complete"})

	outcome = sm.execute()

if __name__ == "__main__":
	main()

