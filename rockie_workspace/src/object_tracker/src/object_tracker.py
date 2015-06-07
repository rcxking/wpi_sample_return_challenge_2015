#!/usr/bin/python

'''
object_tracker.py 
Node to cluster lines for object detection


Bryant Pong / Micah Corah      
CSCI-4962
'''

import numpy as np
import rospy
import cluster_util as cu

class ObjectTracker:
  def __init__(self):
    self.loadParams()

    rospy.Subscriber('observation', Observation)
    #self.service = rospy.ServiceProxy('service', Steer)

  def loadParams(self):
    self.param = rospy.get_param('param', 0.2)

if __name__ == "__main__":
  rospy.init_node('object_tracker')
  o = ObjectTracker()
  rospy.spin()
