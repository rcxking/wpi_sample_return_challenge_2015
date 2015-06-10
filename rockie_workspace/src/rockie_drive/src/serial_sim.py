#!/usr/bin/python

import rospy
import numpy as np

from geometry_msgs.msg import Twist
from serial_node.srv import WheelVel
from serial_node.srv import Steer
from std_msgs.msg import Bool

class SerialSim:
  def __init__(self):
    self.loadParams()

    self.turning = False

    self.wheel_vel_serv = rospy.Service('wheel_vel', WheelVel, self.wheelVelCallback)
    self.steer_serv = rospy.Service('steer', Steer, self.steerCallback)

    # True is turning
    self.steer_pub = rospy.Publisher("turn_in_place_status", Bool)

    self.start()

  def start(self):
    rospy.Timer(rospy.Duration(1.0/self.turn_status_rate), self.turnPubCallback)

  def loadParams(self):
    self.turn_status_rate = rospy.get_param('turn_status_rate', 5)

  def wheelVelCallback(self, req):
    print "front_left:", req.front_left, \
          "front_right:", req.front_right, \
          "rear_left:", req.rear_left, \
          "rear_right:", req.rear_right
    return True

  def steerCallback(self, req):
    self.turning = req.turned
    return True

  def turnPubCallback(self, t):
    self.steer_pub.publish(self.turning)

if __name__ == "__main__":
  rospy.init_node('serial_sim')
  s = SerialSim()
  rospy.spin()
