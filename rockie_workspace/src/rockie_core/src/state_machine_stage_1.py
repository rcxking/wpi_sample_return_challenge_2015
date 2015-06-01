#!/usr/bin/python

'''
State machine to run for stage 1.

RPI Rock Raiders
5/31/15

Last Updated: Bryant Pong: 6/1/15 - 12:39 PM
'''

# Python Imports:
import roslib
import rospy
import smach
import smach_ros

'''
This state performs system checks on the robot before beginning the run.   
'''
class StartupSequence(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=["egressEnter"])    

	def execute(self, userdata):
		rospy.loginfo("Executing Startup Sequence") 
		return "egressEnter"

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

		sampleFound = True
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
		#smach.StateMachine.add("FOO", Foo(), transitions={'outcome1':'BAR','outcome2':'outcome4'})
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

