#!/usr/bin/python

'''
tracker.py 
Node to cluster lines for object detection


Bryant Pong / Micah Corah      
CSCI-4962

TODO
test code
implement blacklist
'''

import numpy as np
import rospy
import cluster_util as cu
import tf
from object_tracker.msg import Observation
from object_tracker.msg import Objects 
from object_tracker.srv import Blacklist 
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

# extract intrinsic matrix from p matrix
def pToIntrinsic(p):
  P = np.array(p).reshape(3,4)
  return P[0:3,0:3]

def tuplesToArrays(tuples):
  points = np.column_stack([point for point, direction in tuples])
  directions = np.column_stack([direction for point, direction in tuples])
  return points, directions

def toPoints(points):
  return [Point(points[0,i],
                points[1,i],
                points[2,i]) for i in range(points.shape[1])]

def transformPoint(T, point):
  tp = np.hstack((point, [1]))
  tpp = np.dot(T,tp)
  return tpp[0:3]

class ObjectTracker:
  RECENT = 0
  ALL = 1

  def __init__(self):

    self.back_log = []
    self.recent_observations = []
    self.recent_next = 0
    self.all_observations = []


    self.tf_listener = tf.TransformListener()
    rospy.Subscriber('observation', Observation, self.observationCallback, queue_size = 20)
    rospy.Service('observation_blacklist', Blacklist, self.blacklistCallback)
    self.recent_pub = rospy.Publisher('recent_objects', Objects)
    self.all_pub = rospy.Publisher('all_objects', Objects)
    self.viz_pub = rospy.Publisher('object_visualizations', Marker)
    #self.service = rospy.ServiceProxy('service', Steer)

    self.loadParams()

    self.start()

  def start(self):
    rospy.Timer(rospy.Duration(self.backlog_rate), self.backlogCallback)
    rospy.Timer(rospy.Duration(self.full_search_rate), self.fullSearchCallback)

  def loadParams(self):
    self.base_frame = rospy.get_param('base_frame', 'map')
    self.backlog_rate = rospy.get_param('backlog_rate', 0.5)
    self.full_search_rate = rospy.get_param('full_search_rate', 0.1)
    self.recent_length = rospy.get_param('recent_length', 20)
    self.sigma_observation = rospy.get_param('sigma_observation', 0.5)
    self.resolve_observation_attempts = rospy.get_param('resolve_observation_attempts', 10)
    self.min_observations = rospy.get_param('min_observations', 10)
    self.bounds = (-100, 100, -100, 100, -20, 20)
    self.recent_color = ColorRGBA(255, 0, 0, 0.7)
    self.all_color = ColorRGBA(0, 0, 255, 0.7)

  def observationCallback(self, msg):
    try:
      self.processObservation(msg)
    except tf.Exception:
      # the transform for the given time may not have been published yet
      rospy.logwarn("could not look up transform from %s to %s", self.base_frame, msg.header.frame_id)
      self.back_log.append([msg,1])

  def blacklistCallback(self, req):
    pass

  def backlogCallback(self, t):
    for i, msg_tries in enumerate(self.back_log):
      try:
        self.processObservation(msg_tries[0])
        self.back_log.pop(msg_tries)
      except:
        msg_tries[1] += 1
        if msg_tries[1] >= self.resolve_observation_attempts:
          self.back_log.pop(i)
          rospy.logwarn("failed to insert observation after %d attempts", self.resolve_observation_attempts)
        
  def fullSearchCallback(self, t):
    self.allObservations()

  def processObservation(self, msg):
    I = pToIntrinsic(msg.P)
    t = msg.header.stamp
    frame_id = msg.header.frame_id

    p = np.array([msg.point[0], msg.point[1], 1])
    point = np.array(msg.point)
    camera = np.zeros(3)

    # look up transform at given time
    T = self.waitForTransform(msg.header)
    rospy.logwarn("successfully found observation")
    point = transformPoint(T, camera)
    direction = transformPoint(T, point) - point
    ray = (point, direction)
    self.pushRay(ray)
    rospy.logwarn("successfully inserted observation")
    self.recentObservations()

  def pushRay(self, line):
    self.all_observations.append(line)
    if len(self.recent_observations) >= self.recent_length:
      self.recent_observations[self.recent_next] = line
      self.recent_next = (self.recent_next + 1) % self.recent_length
    else:
      self.recent_observations.append(line)

  def recentObservations(self):
    if len(self.recent_observations) > self.min_observations:
      objects = self.getObservations(self.recent_observations)
      if objects.shape[1] > 0:
        print objects.shape
        self.recent_pub.publish(self.toObjects(objects))
        self.viz_pub.publish(self.toMarker(objects, ObjectTracker.RECENT))

  def allObservations(self):
    if len(self.all_observations) > self.min_observations:
      objects = self.getObservations(self.all_observations)
      if objects.shape[1] > 0:
        print objects.shape
        self.recent_pub.publish(self.toObjects(objects))
        self.all_pub.publish(self.toObjects(objects))
        self.viz_pub.publish(self.toMarker(objects, ObjectTracker.ALL))

  def getObservations(self, tuples):
    points, directions = tuplesToArrays(tuples)
    print "##############################################"
    print "running estimator"
    print len(tuples), points.shape, directions.shape
    print "##############################################"
    return cu.estimatePoints(points = points,
                             directions = directions,
                             threshold = self.sigma_observation,
                             bounds = self.bounds
                            )

  def toObjects(self, points):
    objects = Objects
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = self.base_frame
    objects.header = header
    objects.objects = toPoints(points)
    return objects

  def toMarker(self, points, observation_type):
    marker = Marker()
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = self.base_frame
    marker.header = header
    marker.ns = "object_tracker"
    marker.id = observation_type
    marker.type = Marker.SPHERE_LIST
    marker.action = Marker.ADD
    marker.scale = Vector3(*[self.sigma_observation for i in range(3)])
    marker.points = toPoints(points)
    color = None
    if observation_type == ObjectTracker.RECENT:
      color = self.recent_color
    elif observation_type == ObjectTracker.ALL:
      color = self.all_color
    else:
      raise ValueError
    marker.colors = [color for i in range(len(marker.points))]
    return marker

  def waitForTransform(self, header):
    self.tf_listener.waitForTransform(self.base_frame, header.frame_id,
      rospy.Time.now(), rospy.Duration(2.0))
    return self.tf_listener.asMatrix(self.base_frame, header)

if __name__ == "__main__":
  rospy.init_node('tracker')
  o = ObjectTracker()
  rospy.spin()
