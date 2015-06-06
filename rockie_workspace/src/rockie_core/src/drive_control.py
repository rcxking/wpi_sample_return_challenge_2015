#!/usr/bin/python
'''
rockie drive control node

This node is responsible for taking the desired velocity from the planner
and turning that into a desired speed for the robot
'''

import rospy
import numpy as np

from geometry_msgs.msg import Twist
from serial_node.srv import WheelVel
from serial_node.srv import Steer
from std_msgs.msg import Bool


class DriveControl:
  def __init__(self):
    self.loadParams()

    self.wheel_vel_proxy = rospy.ServiceProxy('wheel_vel', WheelVel)
    self.wheel_vel_proxy = rospy.ServiceProxy('steer', Steer)
    rospy.Subscriber("cmd_vel", Twist, self.driveCommandCallback)
    rospy.Subscriber("turning", Bool, self.turnInPlaceCallback)

    self.turn_in_place = False


    self.radial_velocity = 0.0 # m/s
    self.linear_velocity = 0.0 # m/s
    self.actual_speeds = np.array([0.0, 0.0, 0.0, 0.0])

  def startController(self):
    rospy.Timer(rospy.Duration(self.control_rate), self.controlLoop)

  def loadParams(self):
    self.min_radius = rospy.get_param('min_differential_steering_radius', 0.2)
    self.ff_gain = rospy.get_param('feed_forward_gain', 0.2)
    self.p_gain = rospy.get_param('proportional_gain', 0.2)
    self.wheel_base_width = rospy.get_param('wheel_base_width', 0.5)
    self.wheel_base_height = rospy.get_param('wheel_base_height', 0.5)
    self.control_rate = rospy.get_param('control_rate', 20)

  def driveCommandCallback(self, msg):
    rospy.loginfo("got drive command")

  def turnInPlaceCallback(self, msg):
    rospy.loginfo("got turn status message")
    self.turn_in_place = msg.data

  def controlLoop(self):
    pass

  def differentialSpeeds(self):
    radius = self.robot_width / 2.0
    left = self.linear_velocity - self.radial_velocity * radius
    right = self.linear_velocity + self.radial_velocity * radius
    return np.array([left, left, right, right])

  def controlEfforts(self, desired_speeds):
    return self.ff_gain * desired_speeds + self.p_gain * (self.desired_speeds - self.actual_speeds)

  def desiredTurningRadius(self):
    return self.linear_velocity / self.radial_velocity

if __name__ == "__main__":
  rospy.init_node('drive_control')
  d = DriveControl()
  rospy.spin()
	
