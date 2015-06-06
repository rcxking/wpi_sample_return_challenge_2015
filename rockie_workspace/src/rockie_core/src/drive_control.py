#!/usr/bin/python
'''
rockie drive control node

This node is responsible for taking the desired velocity from the planner
and turning that into a desired speed for the robot
'''

import rospy
import numpy as np
import Time

from geometry_msgs.msg import Twist
from serial_node.srv import WheelVel
from serial_node.srv import Steer
from std_msgs.msg import Bool

class DriveControl:
  def __init__(self):
    self.loadParams()

    self.wheel_vel_proxy = rospy.ServiceProxy('wheel_vel', WheelVel)
    self.steer_proxy = rospy.ServiceProxy('steer', Steer)
    rospy.Subscriber("cmd_vel", Twist, self.driveCommandCallback)
    rospy.Subscriber("turn_in_place_status", Bool, self.turnInPlaceStatusCallback)

    self.turn_in_place = False
    self.desired_turn_in_place = False
    self.last_command_time = rospy.Time(0)

    self.desired_angular_velocity = 0.0 # m/s
    self.desired_linear_velocity = 0.0 # m/s
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
    self.drive_controller_timeout = rospy.get_param('drive_controller_timeout', 0.5)

  def driveCommandCallback(self, msg):
    self.desired_linear_velocity = msg.linear.x
    self.desired_angular_velocity = msg.angular.z
    rospy.loginfo("got drive command linear: %d angular: %d", self.desired_linear_velocity, self.desired_angular_velocity)
    self.last_command_time = rospy.Time.now()

  def turnInPlaceStatusCallback(self, msg):
    rospy.loginfo("got turn status message")
    self.turn_in_place = msg.data

  def controlLoop(self):
    if desiredTurningRadius() < self.min_radius:
      self.requestTurnInPlace(True)
    else:
      self.requestTurnInPlace(False)

    # only send motor commands when turning state matches desired state
    if self.desired_turn_in_place == self.turn_in_place:
      self.wheel_vel_proxy(*self.desiredWheelSpeeds())

  def desiredWheelSpeeds(self):
    if rospy.Time.now() - self.last_command_time > drive_controller_timeout:
      return np.zeros(4)
    elif not desired_turn_in_place == turn_in_place:
      # wheels still turning
      return np.zeros(4)
    elif desired_turn_in_place:
      return self.turningSpeeds()
    else:
      return self.differentialSpeeds()

  def differentialSpeeds(self):
    radius = self.wheel_base_width / 2.0
    left = self.desired_linear_velocity - self.desired_angular_velocity * radius
    right = self.desired_linear_velocity + self.desired_angular_velocity * radius
    return np.array([left, left, right, right])

  def turningSpeeds(self):
    radius = np.sqrt(self.wheel_base_width**2 + self.wheel_base_height**2)
    left = -self.desired_angular_velocity * radius
    right = self.desired_angular_velocity * radius
    return np.array([left, left, right, right])
    
  def controlEfforts(self, desired_speeds):
    return self.ff_gain * desired_speeds + self.p_gain * (self.desired_speeds - self.actual_speeds)

  def desiredTurningRadius(self):
    return self.desired_linear_velocity / self.desired_angular_velocity

  def requestTurnInPlace(self, turn_in_place):
    if not self.turn_in_place == turn_in_place:
      try:
        self.steer_proxy(turn_in_place)
        self.desired_turn_in_place = turn_in_place
      except:
        rospy.logwarn("Tried to turn in place and failed")
  

if __name__ == "__main__":
  rospy.init_node('drive_control')
  d = DriveControl()
  rospy.spin()
