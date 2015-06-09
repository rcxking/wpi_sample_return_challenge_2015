#!/usr/bin/python

'''
State machine to run for stage 1.

RPI Rock Raiders
5/31/15

Last Updated: Bryant Pong: 6/8/15 - 2:01 PM
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

# Operating System / Data Libraries:
import os
import time
import cPickle as pickle

# Miscellaneous Libraries:
import warnings
import matplotlib.pyplot as plt

# Serial Messages for Services:
from serial_node.srv import *  

'''
Global Objects:
'''

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
						
		rospy.loginfo("Now sending motors to home position") 
		rospy.wait_for_service("steerservice")
		try:
			steerservice = rospy.ServiceProxy('steerservice', SteerService)	 
			steerservice(False)
		except rospy.ServiceException, e:
			print("Service call failed: %s" % e)
		return "egressEnter"

class Egress(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=["transitEnter"])

	def execute(self, userdata):
		rospy.loginfo("Executing Egress")

		rospy.loginfo("Exiting off  
		return "transitEnter"

class Transit(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=["searchEnter"])
		
	def execute(self, userdata):
		rospy.loginfo("Executing Transit")
					
		rospy.loginfo("Done executing Transit.  Now beginning search for object.")	
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

