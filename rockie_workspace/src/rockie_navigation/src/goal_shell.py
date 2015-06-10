#!/usr/bin/python

import rospy
import actionlib
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback

class GoalShell:
  def __init__(self):
    print "Initializing goal shell"
    self.goal_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    print "Waiting for server"
    self.goal_client.wait_for_server()

  def spin(self):
    print "Starting goal shell"
    while not rospy.is_shutdown():
      frame_id = raw_input("frame_id: ")
      x = float(raw_input("x: "))
      y = float(raw_input("y: "))
      yaw = float(raw_input("yaw: "))

      goal = MoveBaseGoal()
      goal.target_pose.header.stamp = rospy.Time.now()
      goal.target_pose.header.frame_id = frame_id
      goal.target_pose.pose.position.x = x
      goal.target_pose.pose.position.y = y
      q = quaternion_from_euler(0, 0, yaw)
      goal.target_pose.pose.orientation = Quaternion(*q)
      self.goal_client.send_goal(goal)
      self.goal_client.wait_for_result(rospy.Duration.from_sec(120))

if __name__ == "__main__":
  rospy.init_node("goal_shell")
  goal_shell = GoalShell()
  goal_shell.spin()
