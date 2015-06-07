import cluster_util
import numpy as np

cases = [
    {
      'intersection_per_line' : False,
      'filter_distance' : False,
      'kmeans' : False,
    },
    {
      'intersection_per_line' : False,
      'filter_distance' : False,
      'kmeans' : True,
    },
    {
      'intersection_per_line' : False,
      'filter_distance' : True,
      'kmeans' : False,
    },
    {
      'intersection_per_line' : False,
      'filter_distance' : True,
      'kmeans' : True,
    },
    {
      'intersection_per_line' : True,
      'filter_distance' : False,
      'kmeans' : False,
    },
    {
      'intersection_per_line' : True,
      'filter_distance' : False,
      'kmeans' : True,
    },
    {
      'intersection_per_line' : True,
      'filter_distance' : True,
      'kmeans' : False,
    },
    {
      'intersection_per_line' : True,
      'filter_distance' : True,
      'kmeans' : True,
    }
  ]

num = 100

results = np.zeros((len(cases), num))
points_ratio = np.zeros((len(cases), num))

for ii in range(len(cases)):
  for jj in range(num):
    results[ii,jj], points_ratio[ii,jj] = cluster_util.testCluster(**cases[ii])
    
print "Results:"
print results
print "Means:"
print np.mean(results,1)
print "Mean ratio of estimated centers to actual centers:"
print np.mean(points_ratio,1)
