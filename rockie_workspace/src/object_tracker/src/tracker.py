#!/usr/bin/python

'''
tracker.py 
Node to cluster lines for object detection


Bryant Pong / Micah Corah      
CSCI-4962
'''

import numpy as np
import rospy
import cluster_util as cu
from object_tracker.msg import Observation

# extract intrinsic matrix from p matrix
def pToIntrinsic(p):
  P = numpy.array(p).reshape(3,4)
  return P[0:3,0:3]

class ObjectTracker:
  def __init__(self):
    self.loadParams()

    rospy.Subscriber('observation', Observation, self.observationCallback)
    #self.service = rospy.ServiceProxy('service', Steer)

  def loadParams(self):
    self.param = rospy.get_param('param', 0.2)

  def observationCallback(self, msg):
    I = pToIntrinsic(msg.P)
    t = msg.header.stamp
    frame_id = msg.header.frame_id
    point = np.array(msg.point)
    pass

if __name__ == "__main__":
  rospy.init_node('tracker')
  o = ObjectTracker()
  rospy.spin()
