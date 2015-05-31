#!/usr/bin/python

'''
State machine to run for stage 1.

RPI Rock Raiders
5/31/15

Last Updated: Bryant Pong: 5/31/15 - 6:26 PM
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
		smach.State.__init__(self, outcomes=['egressEnter'])    

	def execute(self,  

class Foo(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])
		self.counter = 0
	
	def execute(self, userdata):
		rospy.loginfo("Executing state FOO")
		if self.counter < 3:
			self.counter += 1
			return 'outcome1'
		else:
			return 'outcome2'

class Bar(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['outcome1'])

	def execute(self, userdata):
		rospy.loginfo('Executing state BAR')
		return 'outcome1'

def main():
	rospy.init_node('smach_example_state_machine')

	sm = smach.StateMachine(outcomes=['outcome4'])
	with sm:
		smach.StateMachine.add("FOO", Foo(), transitions={'outcome1':'BAR','outcome2':'outcome4'})
		smach.StateMachine.add("BAR", Bar(), transitions={'outcome1':'FOO'})

	outcome = sm.execute()

if __name__ == "__main__":
	main()

