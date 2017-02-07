#!/usr/bin/python

############################################################
# ExtractLines.py
#
# This script reads in range data from a csv file, and
# implements a split-and-merge to extract meaningful lines
# in the environment.
############################################################

# Imports
import numpy as np
from PlotFunctions import *
import pdb


############################################################
# functions
############################################################

#-----------------------------------------------------------
# ExtractLines
#
# This function implements a split-and-merge line
# extraction algorithm
#
# INPUT: RangeData - (x_r, y_r, theta, rho)
#                x_r - robot's x position (m)
#                y_r - robot's y position (m)
#              theta - (1D) np array of angle 'theta' from data (rads)
#                rho - (1D) np array of distance 'rho' from data (m)
#           params - dictionary of parameters for line extraction
#
# OUTPUT: (alpha, r, segend, pointIdx)
#         alpha - (1D) np array of 'alpha' for each fitted line (rads)
#             r - (1D) np array of 'r' for each fitted line (m)
#        segend - np array (N_lines, 4) of line segment endpoints.
#                 each row represents [x1, y1, x2, y2]
#      pointIdx - (N_lines,2) segment's first and last point index

def ExtractLines(RangeData, params):

  #Extract useful variables from RangeData
  x_r = RangeData[0]
  y_r = RangeData[1]
  theta = RangeData[2]
  rho = RangeData[3]

  FitLine(theta, rho)

  ### Split Lines ###
  N_pts = len(rho)
  startIdx = 0;
  endIdx = N_pts
  alpha, r, pointIdx = SplitLinesRecursive(theta, rho, startIdx, endIdx, params)

  ### Merge Lines ###
  alpha, r, pointIdx = MergeColinearNeigbors(theta, rho, alpha, r, pointIdx, params)
  N_lines = alpha.shape[0]

  ### Compute endpoints/lengths of the segments ###
  segend = np.zeros((N_lines, 4))
  seglen = np.zeros(N_lines)
  for i in range(N_lines):
      rho1 = r[i]/np.cos(theta[pointIdx[i,0]]-alpha[i])
      rho2 = r[i]/np.cos(theta[pointIdx[i,1]-1]-alpha[i])
      x1 = rho1*np.cos(theta[pointIdx[i,0]])
      y1 = rho1*np.sin(theta[pointIdx[i,0]])
      x2 = rho2*np.cos(theta[pointIdx[i,1]-1])
      y2 = rho2*np.sin(theta[pointIdx[i,1]-1])
      segend[i,:] = np.hstack((x1, y1, x2, y2))
      seglen[i] = np.linalg.norm(segend[i,0:2] - segend[i,2:4])

  ### Filter Lines ###
  #Find and remove line segments that are too short
  goodSegIdx = np.where((seglen >= params['MIN_SEG_LENGTH']) &
  (pointIdx[:,1] - pointIdx[:,0] >= params['MIN_POINTS_PER_SEGMENT']))[0]
  pointIdx = pointIdx[goodSegIdx, :]
  alpha = alpha[goodSegIdx]
  r = r[goodSegIdx]
  segend = segend[goodSegIdx, :]

  #change back to scene coordinates
  segend[:,(0,2)] = segend[:,(0,2)] + x_r
  segend[:,(1,3)] = segend[:,(1,3)] + y_r

  return alpha, r, segend, pointIdx



#-----------------------------------------------------------
# SplitLineRecursive
#
# This function executes a recursive line-slitting algorithm,
# which recursively sub-divides line segments until no further
# splitting is required.
#
# INPUT:  theta - (1D) np array of angle 'theta' from data (rads)
#           rho - (1D) np array of distance 'rho' from data (m)
#      startIdx - starting index of segment to be split
#        endIdx - ending index of segment to be split
#        params - dictionary of parameters
#
# OUTPUT: alpha - (1D) np array of 'alpha' for each fitted line (rads)
#             r - (1D) np array of 'r' for each fitted line (m)
#           idx - (N_lines,2) segment's first and last point index

def SplitLinesRecursive(theta, rho, startIdx, endIdx, params):

  ##### TO DO #####
  # Implement a recursive line splitting function
  # It should call 'FitLine()' to fit individual line segments
  # In should call 'FindSplit()' to find an index to split at
  #################
  alpha, r = FitLine(theta[startIdx:endIdx], rho[startIdx:endIdx])

  # Index to split at is referenced from startIdx. splitIdx = FindSplit + startIdx
  splitIdx = FindSplit(theta[startIdx:endIdx], rho[startIdx:endIdx], alpha, r, params) + startIdx

  # Check whether we've found the last possible split
  if FindSplit(theta[startIdx:endIdx], rho[startIdx:endIdx], alpha, r, params) == -1:
    # If so, return the line fit parameters for this last data set
    return alpha, r, [startIdx, endIdx]

  # Attempt to continue splitting the lines
  alpha_1, r_1, idx_1 = SplitLinesRecursive(theta, rho, startIdx, splitIdx, params)
  alpha_2, r_2, idx_2 = SplitLinesRecursive(theta, rho, splitIdx, endIdx, params)

  # Compile the complete array results and return up stack
  alpha = np.append(np.array([alpha_1]),np.array([alpha_2]))
  r = np.append(np.array([r_1]),np.array([r_2]))
  idx = np.row_stack((idx_1,idx_2))

  return alpha, r, idx



#-----------------------------------------------------------
# FindSplit
#
# This function takes in a line segment and outputs the best
# index at which to split the segment
#
# INPUT:  theta - (1D) np array of angle 'theta' from data (rads)
#           rho - (1D) np array of distance 'rho' from data (m)
#         alpha - 'alpha' of input line segment (1 number)
#             r - 'r' of input line segment (1 number)
#        params - dictionary of parameters
#
# OUTPUT: SplitIdx - idx at which to split line (return -1 if
#                    it cannot be split)

def FindSplit(theta, rho, alpha, r, params):

  ##### TO DO #####
  # Implement a function to find the split index (if one exists)
  # It should compute the distance of each point to the line.
  # The index to split at is the one with the maximum distance
  # value that exceeds 'LINE_POINT_DIST_THRESHOLD', and also does
  # not divide into segments smaller than 'MIN_POINTS_PER_SEGMENT'
  # return -1 if no split is possiple
  #################

  # If array is too short to possibly have a split point, just return -1
  if len(theta) < 2*params['MIN_POINTS_PER_SEGMENT']:
    return -1

  # Get possible points to split at (at distance MIN_POINTS_PER_SEGMENT from edge of array)
  dist = np.absolute(rho*np.cos(theta-alpha)-r)
  dist_zeroed = dist
  dist_zeroed[0:params['MIN_POINTS_PER_SEGMENT']] = 0
  dist_zeroed[len(dist)-params['MIN_POINTS_PER_SEGMENT']:len(dist)]= 0

  #dist[params['MIN_POINTS_PER_SEGMENT']:(len(theta)-params['MIN_POINTS_PER_SEGMENT'])]
  splitIdx = np.argmax(dist_zeroed)

  # Check if this max value satisfies the requirement of being greater than LINE_POINT_DIST_THRESHOLD
  if dist[splitIdx] < params['LINE_POINT_DIST_THRESHOLD']:
    return -1

  return splitIdx

#-----------------------------------------------------------
# FitLine
#
# This function outputs a best fit line to a segment of range
# data, expressed in polar form (alpha, r)
#
# INPUT:  theta - (1D) np array of angle 'theta' from data (rads)
#           rho - (1D) np array of distance 'rho' from data (m)
#
# OUTPUT: alpha - 'alpha' of best fit for range data (1 number) (rads)
#             r - 'r' of best fit for range data (1 number) (m)

def FitLine(theta, rho):

  n = len(theta)

  num1 = np.sum(rho**2*np.sin(2*theta)) 
  den1 = np.sum(rho**2*np.cos(2*theta)) 

  num2 = 0
  den2 = 0
  for i in range(n):
    for j in range(n):
      den2 += rho[i]*rho[j]*np.cos(theta[i]+theta[j])
      num2 += rho[i]*rho[j]*np.cos(theta[i])*np.sin(theta[j])
  den2 = -den2/n
  num2 = -num2*2/n

  alpha = 0.5*np.arctan2((num1+num2),(den1+den2)) + np.pi/2
  r = np.mean(rho*np.cos(theta-alpha))

  # If r is <0, make line equation have positive r and be sure to offset alpha by pi
  if r<0:
    r = -r
    alpha = alpha+np.pi 
    # Wrap alpha to -pi < alpha < pi
    if alpha>np.pi:
      alpha = alpha - 2*np.pi

  return alpha, r



#---------------------------------------------------------------------
# MergeColinearNeigbors
#
# This function merges neighboring segments that are colinear and outputs
# a new set of line segments
#
# INPUT:  theta - (1D) np array of angle 'theta' from data (rads)
#           rho - (1D) np array of distance 'rho' from data (m)
#         alpha - (1D) np array of 'alpha' for each fitted line (rads)
#             r - (1D) np array of 'r' for each fitted line (m)
#      pointIdx - (N_lines,2) segment's first and last point indices
#        params - dictionary of parameters
#
# OUTPUT: alphaOut - output 'alpha' of merged lines (rads)
#             rOut - output 'r' of merged lines (m)
#      pointIdxOut - output start and end indices of merged line segments

def MergeColinearNeigbors(theta, rho, alpha, r, pointIdx, params):

  ##### TO DO #####
  # Implement a function to merge colinear neighboring line segments
  # HINT: loop through line segments and try to fit a line to data
  #       points from two adjacent segments. If this line cannot be
  #       split, then accept the merge. If it can be split, do not merge.
  #################

  haveMerged = True
  alphaOut = alpha.copy()
  rOut = r.copy()
  pointIdxOut = pointIdx.copy()

  while haveMerged:
    haveMerged = False

    # Loops backwards over line segments, merging possible sets
    for i in range(len(pointIdxOut)-1,0,-1):
      startIdx = pointIdxOut[i-1][0]
      endIdx = pointIdxOut[i][1]
      # Fit a line to the combined data set
      alpha_combined, r_combined = FitLine(theta[startIdx:endIdx], rho[startIdx:endIdx])
      # Index to split at is referenced from startIdx. splitIdx = FindSplit + startIdx
      splitIdx = FindSplit(theta[startIdx:endIdx], rho[startIdx:endIdx], alpha_combined, r_combined, params)
      # If merge is possible
      if splitIdx == -1:
        # Merge data sets
        pointIdxOut[i-1,1] = pointIdxOut[i,1]
        np.delete(pointIdxOut,i)
        np.delete(alphaOut,i)
        np.delete(rOut,i)
        alphaOut[i-1] = alpha_combined
        rOut[i-1] = r_combined
        haveMerged = True

  return alphaOut, rOut, pointIdxOut



#----------------------------------
# ImportRangeData
def ImportRangeData(filename):

  data = np.genfromtxt('./RangeData/'+filename, delimiter=',')
  x_r = data[0,0]
  y_r = data[0,1]
  theta = data[1:,0]
  rho = data[1:,1]
  return (x_r, y_r, theta, rho)
#----------------------------------



############################################################
# Main
############################################################
def main():
  # parameters for line extraction (feel free to adjust these)
  MIN_SEG_LENGTH = 0.05; # minimum length of each line segment (m)
  LINE_POINT_DIST_THRESHOLD = 0.02; # max distance of pt from line to split
  MIN_POINTS_PER_SEGMENT = 4; # minimum number of points per line segment

  # Data files are formated as 'rangeData_<x_r>_<y_r>_N_pts.csv
  # where x_r is the robot's x position
  #       y_r is the robot's y position
  #       N_pts is the number of beams (e.g. 180 -> beams are 2deg apart)

  filename = 'rangeData_5_5_180.csv'
  #filename = 'rangeData_4_9_360.csv'
  #filename = 'rangeData_7_2_90.csv'

  #Import Range Data
  RangeData = ImportRangeData(filename)

  params = {'MIN_SEG_LENGTH': MIN_SEG_LENGTH,
            'LINE_POINT_DIST_THRESHOLD': LINE_POINT_DIST_THRESHOLD,
            'MIN_POINTS_PER_SEGMENT': MIN_POINTS_PER_SEGMENT}

  alpha, r, segend, pointIdx = ExtractLines(RangeData, params)

  ax = PlotScene()
  ax = PlotData(RangeData, ax)
  ax = PlotRays(RangeData, ax)
  ax = PlotLines(segend, ax)

  plt.show(ax)

############################################################

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        import traceback
        traceback.print_exc()
