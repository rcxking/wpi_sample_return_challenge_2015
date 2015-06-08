#!/usr/bin/python

'''
cluster_util.py 
math for line clustering

Bryant Pong / Micah Corah      
CSCI-4962
'''

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
from sklearn.cluster import KMeans
from sklearn.cluster import MeanShift

'''
returns point nearest to two skew lines
'''
def nearestPoint(p1, d1, p2, d2):
  # least squares solution to nearest points on each line
  b = np.array([[2*np.dot(p2,d1) - 2*np.dot(p1,d1)],
                [2*np.dot(p1,d2) - 2*np.dot(p2,d2)]])
  a = np.array([[2*np.dot(d1,d1), -2*np.dot(d2,d1)],
                [-2*np.dot(d2,d1), 2*np.dot(d2,d2)]])

  # coefficients for nearest points
  # p+s*d
  ss = np.linalg.solve(a, b)

  # nearest points
  n1 = p1 + ss[0] * d1
  n2 = p2 + ss[1] * d2

  # nearest point is average
  return (n1 + n2) / 2

'''
angle between to vectors
'''
def innerAngle(d1, d2):
  d1p = unit(d1)
  d2p = unit(d2)
  return math.acos(np.dot(d1p, d2p))

'''
unit vector in direction v
'''
def unit(v):
  return v / np.linalg.norm(v)


'''
returns distance between two skew lines
see
http://2000clicks.com/mathhelp/GeometryPointsAndLines3D.aspx
'''
def lineDistance(p1, d1, p2, d2):
  n = np.cross(d1, d2)
  np.dot(p1 - p2, n) / np.linalg.norm(n)

def inBounds(p, bounds):
  return p[0] > bounds[0] and p[0] < bounds[1] \
         and p[1] > bounds[2] and p[1] < bounds[3] \
         and p[2] > bounds[4] and p[2] < bounds[5]
         
def kMeans(points):
  # perform kmeans clustering of data
  kmeans = KMeans(4)
  kmeans.fit(points.T)
  labels = kmeans.predict(points.T)
  return labels

def nearAny(point, points, threshold):
  print "cluster shape:",points.shape
  for ii in range(points.shape[1]):
    if np.linalg.norm(points[:,ii] - point) < threshold:
      return True
  return False

def connectedClusters(points, threshold):
  print "Points:", points
  clusters = []
  for ii in range(points.shape[1]):
    nearby_clusters = []
    point = points[:,ii]
    for jj in range(len(clusters)):
      for index in clusters[jj]:
        if np.linalg.norm(points[:,index]- point) < threshold:
          nearby_clusters.append(jj)
          break
    nearby_clusters.sort()
    if len(nearby_clusters) == 0:
      # new cluster
      clusters.append([ii])
    else:
      clusters[nearby_clusters[0]].append(ii)
      # append neighboring clusters
      # going in reverse order to indices don't change
      for jj in range(len(nearby_clusters)-1, 0, -1):
        clusters[nearby_clusters[0]].extend(clusters[nearby_clusters[jj]])
        clusters.pop(jj)
  labels = np.zeros(points.shape[1], dtype=np.int)
  print "Clusters found:", len(clusters)
  for ii in range(len(clusters)):
    labels[clusters[ii]] = ii
  return labels

def meanShift(points):
  # perform meanshift clustering of data
  meanshift = MeanShift()
  meanshift.fit(points.T)
  labels = meanshift.labels_
  centers = meanshift.cluster_centers_
  return np.array(labels)

def analyzeClusters(points, labels, plot_data = False, ground_truth = None):
  colors = ['b', 'c', 'g', 'y', 'm', 'r', 'k']

  max_label = np.max(labels)

  clusters = []
  # compute clusters
  centers = []
  for ii in range(max_label+1):
    indices = np.nonzero(np.equal(labels, ii))
    cluster = points[:,indices]
    center = np.mean(cluster, 2)
    centers.append(center)
    clusters.append(cluster)
  centers = np.hstack(centers)
  print "Centers:", centers
  if not ground_truth is None:
    print "Ground truth:", ground_truth

  # plotting
  if plot_data:
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for ii in range(len(clusters)):
      cluster = clusters[ii]
      ax.scatter(cluster[0,:], cluster[1,:], cluster[2,:], c=colors[ii], marker='x')
    ax.scatter(centers[0,:], centers[1,:], centers[2,:], c='m', marker='o')
    plotGroundTruth(ax, ground_truth)
    plt.show()
  return centers

# return intersection if good
def computeIntersection(p1, d1, p2, d2, threshold, bounds):
  if lineDistance(p1, d1, p2, d2) < threshold \
      and innerAngle(d1, d2) > np.pi / 180:
    nearest = nearestPoint(p1, d1, p2, d2)
    # throw out points that are outside of the region of interest
    if inBounds(nearest, bounds):
      #print "Found correspondence:", nearest
      return nearest
  return None

def projectPoints(p1, d1, points):
  d1p = unit(d1)
  vs = points - p1[:,np.newaxis]
  return np.dot(d1p.T, vs)

def bestIntersection(p1, d1, intersections, threshold):
  d1p = unit(d1)
  dists = projectPoints(p1, d1p, intersections)
  assert(dists.size ==  intersections.shape[1])
  dists.sort()

  '''
  get number of points fitting in a window of length threshold starting at each
  point
  '''
  counts = np.zeros(dists.shape)
  for ii in range(dists.size):
    dist = dists[ii]
    n = 1
    while ii+n < dists.size and dists[ii+n] - dist < threshold:
      n += 1
    counts[ii] = n
  max_index = np.argmax(counts)
  count = counts[max_index]
  est_dist = np.mean(dists[max_index:max_index+count])
  intersection = p1 + (d1p * est_dist)
  return intersection, count

def interSectionPerLine(points, 
                        directions,
                        threshold = 0.1,
                        bounds = (0,1,0,1,0,1),
                        plot_data = False,
                        ground_truth = None):
  print "intersectionPerLine"
  num_point = points.shape[1]

  intersections_by_line = []
  counts_by_line = []
  # get intersection per each line
  for ii in range(num_point):
    intersections = []
    p1 = points[:,ii]
    d1 = directions[:,ii]
    for jj in range(num_point):
      if not jj == ii:
        p2 = points[:,jj]
        d2 = directions[:,jj]
        intersection = computeIntersection(p1, d1, p2, d2, threshold, bounds)
        if not intersection is None:
          intersections.append(intersection)
    intersections = np.array(intersections).T
    intersection, count = bestIntersection(p1, d1, intersections, threshold)
    #assert(inBounds(intersection, bounds))
    intersections_by_line.append(intersection)
    counts_by_line.append(count)
  intersections_by_line = np.array(intersections_by_line).T

  max_count = max(counts_by_line)
  if plot_data:
    # plot figure for correspondence
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for ii in range(len(counts_by_line)):
      weight = counts_by_line[ii] / max_count
      ax.scatter(intersections_by_line[0,ii], intersections_by_line[1,ii], intersections_by_line[2,ii], c=str(weight), marker='x')
    plotGroundTruth(ax, ground_truth)
    plt.show()

  return intersections_by_line, counts_by_line

def intersectionByDistance(points, 
            directions,
            threshold = 0.1,
            bounds = (0,1,0,1,0,1),
            plot_data = False,
            ground_truth = None
            ):
  print "intersectionByDistance"
  num_point = points.shape[1]

  correspondence_points = []
  for ii in range(num_point):
    for jj in range(ii+1, num_point):
      p1 = points[:,ii]
      d1 = directions[:,ii]
      p2 = points[:,jj]
      d2 = directions[:,jj]
      # interested in nearby skew lines
      correspondence = computeIntersection(p1, d1, p2, d2, threshold, bounds)
      if not correspondence is None:
        correspondence_points.append(correspondence)
  correspondence_points = np.array(correspondence_points).T
  print correspondence_points.shape

  if plot_data:
    # plot figure for correspondence
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(correspondence_points[0,:], correspondence_points[1,:], correspondence_points[2,:], c='b', marker='x')
    plotGroundTruth(ax, ground_truth)
    plt.show()
  return correspondence_points

def plotGroundTruth(ax, ground_truth = None):
  if not ground_truth is None:
    ax.scatter(ground_truth[0,:], ground_truth[1,:], ground_truth[2,:], c='r', marker='o')

def plotLines(points, directions, ground_truth = None):
  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')

  #points
  ax.scatter(points[0,:], points[1,:], points[2,:], c='b', marker='x')

  #lines
  for i in range(points.shape[1]):
    point = points[:,i]
    direction = directions[:,i]
    norm_d = direction / np.linalg.norm(direction)
    ends = np.column_stack([(point - norm_d), (point + norm_d)])
    ax.plot(ends[0,:], ends[1,:], ends[2,:])
  plotGroundTruth(ax, ground_truth)
  plt.show()

def filterDistance(points, threshold, plot_data=False, ground_truth=None):
  print "filterDistance"
  filtered = []
  good = np.zeros(points.shape[1])
  for ii in range(points.shape[1]):
    if not good[ii]:
      for jj in range(ii+1, points.shape[1]):
        if np.linalg.norm(points[:,ii] - points[:,jj]) < threshold:
          good[ii] = 1
          good[jj] = 1
          filtered.append(points[:,ii])
          filtered.append(points[:,jj])
  filtered = np.array(filtered).T

  if plot_data:
    # plot figure for correspondence
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(filtered[0,:], filtered[1,:], filtered[2,:], c='b', marker='x')
    plotGroundTruth(ax, ground_truth)
    plt.show()
  return filtered

'''
Generate simulation data
'''
def simulateData(
    number_points_of_interest = 4,
    number_observation = 10,
    sigma_observation = 0.06,
    false_positives_per_observation = 2,
    ):
  points_of_interest = np.random.ranf((3,number_points_of_interest))
  print "Points of interest"
  print points_of_interest

  observation_points = np.hstack(
      [points_of_interest + sigma_observation * np.random.standard_normal(points_of_interest.shape)
        for i in range(number_observation)])

  observation_directions = np.hstack(
      [np.random.standard_normal(points_of_interest.shape)
        for i in range(number_observation)])


  number_false_positive = int(false_positives_per_observation * \
      number_observation * number_points_of_interest)

  false_positive_points = np.hstack(
      [np.random.ranf((3,1))
        for i in range(number_observation)])

  false_positive_directions = np.hstack(
      [np.random.standard_normal((3,1))
        for i in range(number_observation)])

  points = np.hstack([observation_points, false_positive_points])
  directions = np.hstack([observation_directions, false_positive_directions])
  return points, directions, points_of_interest

def estimatePoints(
    points,
    directions,
    ground_truth,
    threshold = 0.05,
    intersection_per_line = True,
    filter_distance = True,
    kmeans = False,
    plot_data = False
    ):
  intersections = None
  counts = None
  if intersection_per_line:
    intersections, counts = interSectionPerLine(points, directions, \
        ground_truth = ground_truth, threshold = threshold,
        plot_data = plot_data)
  else:
    intersections = intersectionByDistance(points, directions, \
        ground_truth = ground_truth, threshold = threshold,
        plot_data = plot_data
        )

  if filter_distance:
    intersections = filterDistance(intersections, threshold,
        plot_data=plot_data,ground_truth=ground_truth)
  labels = None

  if kmeans:
    labels = kMeans(intersections)
  else:
    labels = connectedClusters(intersections, threshold*2)
  centers = analyzeClusters(intersections, labels, plot_data = plot_data,
      ground_truth = ground_truth)
  return centers

def calculateError(ground_truth, estimates):
  error_vec = np.zeros(ground_truth.shape[1])
  for ii in range(ground_truth.shape[1]):
    error_vec[ii] = min([np.linalg.norm(estimates[:,jj] - ground_truth[:,ii]) for jj in range(estimates.shape[1])])
  return np.mean(error_vec)

'''
Test line clustering algorithm
Considering lines contained in the unit cube
1. generate points of interest
2. generate normally distributed observations and directions around points of
interest
3. generate large number of false positives uniformly in unit cube
4. run clustering on observations and false positives
'''
def testCluster(
    number_points_of_interest = 4,
    number_observation = 10,
    sigma_observation = 0.06,
    false_positives_per_observation = 2,
    intersection_per_line = True,
    filter_distance = True,
    kmeans = False,
    plot_data = False
    ):

  points, directions, ground_truth = simulateData( \
      number_points_of_interest = number_points_of_interest,
      number_observation = number_observation,
      sigma_observation = sigma_observation,
      false_positives_per_observation = false_positives_per_observation)
  if plot_data:
    plotLines(points, directions, ground_truth)

  estimates = estimatePoints(
      points, directions, ground_truth,
      threshold = sigma_observation,
      intersection_per_line = intersection_per_line,
      filter_distance = filter_distance,
      kmeans = kmeans,
      plot_data = plot_data
      )

  error = calculateError(ground_truth, estimates)
  print "Mean minimum distance to nearest estimate:", error
  return error, float(estimates.shape[1]) / number_points_of_interest

if __name__ == "__main__":
  testCluster(
      plot_data=True,
      intersection_per_line = True,
      filter_distance = True,
      kmeans = False
      )
