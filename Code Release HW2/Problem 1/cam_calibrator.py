#!/usr/bin/python

import rospy
import sensor_msgs

import time
import os

import cv2
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib import cm
import numpy as np

import pdb

from camera_calibration.calibrator import MonoCalibrator, ChessboardInfo, Patterns

class CameraCalibrator:
  def __init__(self):
    self.calib_flags = 0
    self.pattern = Patterns.Chessboard

  def loadImages(self, cal_img_path, name, n_corners, square_length, display_flag):
    self.name = name
    self.cal_img_path = cal_img_path

    self.boards = []
    self.boards.append(ChessboardInfo(n_corners[0], n_corners[1], float(square_length)))
    self.c = MonoCalibrator(self.boards, self.calib_flags, self.pattern)

    if display_flag:
      fig = plt.figure('Corner Extraction', figsize = (12,5))
      gs = gridspec.GridSpec(1,2)
      gs.update(wspace=0.025, hspace=0.05)

    for i, file in enumerate(os.listdir(self.cal_img_path)):
      img = cv2.imread(self.cal_img_path + '/' + file, 0)     # Load the image
      img_msg = self.c.br.cv2_to_imgmsg(img, 'mono8')         # Convert to ROS Image msg
      drawable = self.c.handle_msg(img_msg)                   # Extract chessboard corners using ROS camera_calibration package

      if display_flag:
        ax = plt.subplot(gs[0,0])
        plt.imshow(img, cmap='gray')
        plt.axis('off')
        
        ax = plt.subplot(gs[0,1])
        plt.imshow(drawable.scrib)
        plt.axis('off')

        plt.subplots_adjust(left=0.02, right=0.98, top=0.98, bottom=0.02)
        fig.canvas.set_window_title('Corner Extraction (Chessboard {0})'.format(i+1))

        plt.show(block=False)
        plt.waitforbuttonpress()

    # Useful parameters
    self.d_square = square_length                             # Length of a chessboard square
    self.h_pixels, self.w_pixels = img.shape                  # Image pixel dimensions
    self.n_chessboards = len(self.c.good_corners)             # Number of examined images
    self.n_corners_y, self.n_corners_x = n_corners            # Dimensions of extracted corner grid
    self.n_corners_per_chessboard = n_corners[0]*n_corners[1]

  def undistortImages(self, A, k = np.zeros(2), scale = 0):
    Anew_no_k, roi = cv2.getOptimalNewCameraMatrix(A, np.zeros(4), (self.w_pixels, self.h_pixels), scale)
    mapx_no_k, mapy_no_k = cv2.initUndistortRectifyMap(A, np.zeros(4), None, Anew_no_k, (self.w_pixels, self.h_pixels), cv2.CV_16SC2)
    Anew_w_k, roi = cv2.getOptimalNewCameraMatrix(A, np.hstack([k, 0, 0]), (self.w_pixels, self.h_pixels), scale)
    mapx_w_k, mapy_w_k = cv2.initUndistortRectifyMap(A, np.hstack([k, 0, 0]), None, Anew_w_k, (self.w_pixels, self.h_pixels), cv2.CV_16SC2)

    if k[0] != 0:
      n_plots = 3
    else:
      n_plots = 2

    fig = plt.figure('Image Correction', figsize = (6*n_plots, 5))
    gs = gridspec.GridSpec(1, n_plots)
    gs.update(wspace=0.025, hspace=0.05)

    for i, file in enumerate(os.listdir(self.cal_img_path)):
      img_dist = cv2.imread(self.cal_img_path + '/' + file, 0)
      img_undist_no_k = cv2.undistort(img_dist, A, np.zeros(4), None, Anew_no_k)
      img_undist_w_k = cv2.undistort(img_dist, A, np.hstack([k, 0, 0]), None, Anew_w_k)

      ax = plt.subplot(gs[0,0])
      ax.imshow(img_dist, cmap='gray')
      ax.axis('off')

      ax = plt.subplot(gs[0,1])
      ax.imshow(img_undist_no_k, cmap='gray')
      ax.axis('off')

      if k[0] != 0:
        ax = plt.subplot(gs[0,2])
        ax.imshow(img_undist_w_k, cmap='gray')
        ax.axis('off')

      plt.subplots_adjust(left=0.02, right=0.98, top=0.98, bottom=0.02)
      fig.canvas.set_window_title('Image Correction (Chessboard {0})'.format(i+1))

      plt.show(block=False)
      plt.waitforbuttonpress()

  def plotBoardPixImages(self, u_meas, v_meas, X, Y, R, t, A, k = np.zeros(2)):
    # Expects X, Y, R, t to be lists of arrays, just like u_meas, v_meas

    fig = plt.figure('Chessboard Projection to Pixel Image Frame', figsize = (8,6))
    plt.clf()

    for p in range(self.n_chessboards):
      plt.clf()
      ax = plt.subplot(111)
      ax.plot(u_meas[p], v_meas[p], 'r+', label='Original')
      u, v = self.transformWorld2PixImageUndist(X[p], Y[p], np.zeros(X[p].size), R[p], t[p], A)
      ax.plot(u, v, 'b+', label='Linear Intrinsic Calibration')

      box = ax.get_position()
      ax.set_position([box.x0, box.y0 + box.height * 0.15, box.width, box.height*0.85])
      if k[0] != 0:
        u_br, v_br = self.transformWorld2PixImageDist(X[p], Y[p], np.zeros(X[p].size), R[p], t[p], A, k)
        ax.plot(u_br, v_br, 'g+', label='Radial Distortion Calibration')

      ax.axis([0, self.w_pixels, 0, self.h_pixels])
      plt.gca().set_aspect('equal', adjustable='box')
      plt.title('Chessboard {0}'.format(p+1))
      ax.legend(loc='lower center', bbox_to_anchor=(0.5, -0.3), fontsize='medium', fancybox=True, shadow=True)

      plt.show(block=False)
      plt.waitforbuttonpress()

  def plotBoardLocations(self, X, Y, R, t):
    # Expects X, U, R, t to be lists of arrays, just like u_meas, v_meas

    ind_corners = [0, self.n_corners_x-1, self.n_corners_x*self.n_corners_y-1, self.n_corners_x*(self.n_corners_y-1),]
    s_cam = 0.02
    d_cam = 0.1
    xyz_cam = [[0, -s_cam, s_cam, s_cam, -s_cam],
               [0, -s_cam, -s_cam, s_cam, s_cam],
               [0, -d_cam, -d_cam, -d_cam, -d_cam]]
    ind_cam = [[0,1,2],[0,2,3],[0,3,4],[0,4,1]]
    verts_cam = []
    for i in range(len(ind_cam)):
      verts_cam.append([zip([xyz_cam[0][j] for j in ind_cam[i]],
                            [xyz_cam[1][j] for j in ind_cam[i]],
                            [xyz_cam[2][j] for j in ind_cam[i]])])

    fig = plt.figure('Estimated Chessboard Locations', figsize = (12,5))
    axim = fig.add_subplot(1, 2, 1)
    ax3d = fig.add_subplot(1, 2, 2, projection='3d')

    boards = []
    verts = []
    for p in range(self.n_chessboards):

      M = []
      W = np.column_stack((R[p],t[p]))
      for i in range(4):
        M_tld = W.dot(np.array([X[p][ind_corners[i]], Y[p][ind_corners[i]], 0, 1]))
        M_tld *= np.sign(M_tld[2])
        M_tld[2] *= -1
        M.append(M_tld[0:3])

      M = (np.array(M).T).tolist()
      verts.append([zip(M[0],M[1],M[2])])
      boards.append(Poly3DCollection(verts[p]))

    for i, file in enumerate(os.listdir(self.cal_img_path)):

      img = cv2.imread(self.cal_img_path + '/' + file, 0)
      axim.imshow(img, cmap='gray')
      axim.axis('off')

      ax3d.clear()

      for j in range(len(ind_cam)):
        cam = Poly3DCollection(verts_cam[j])
        cam.set_color('green')
        cam.set_alpha(0.2)
        ax3d.add_collection3d(cam)
        
      for p in range(self.n_chessboards):
        if p == i:
          boards[p].set_color('blue')
          boards[p].set_alpha(1.0)
        else:
          boards[p].set_color('red')
          boards[p].set_alpha(0.1)

        ax3d.add_collection3d(boards[p])
        ax3d.text(verts[p][0][0][0], verts[p][0][0][1], verts[p][0][0][2], '{0}'.format(p+1))
        plt.show(block=False)

      view_max = 0.2
      ax3d.set_xlim(-view_max,view_max)
      ax3d.set_ylim(-view_max,view_max)
      ax3d.set_zlim(-5*view_max,0)
      ax3d.set_xlabel('X axis')
      ax3d.set_ylabel('Y axis')
      ax3d.set_zlabel('Z axis')

      plt.tight_layout()
      fig.canvas.set_window_title('Estimated Board Locations (Chessboard {0})'.format(i+1))

      plt.show(block=False)

      raw_input('<Hit Enter To Continue>')

  def writeCalibrationYaml(self, A, k):
    self.c.intrinsics = A
    self.c.distortion = np.hstack((k, np.zeros(3))).reshape((5,1))
    self.c.name = self.name
    self.c.R = np.eye(3)
    self.c.P = np.column_stack((np.eye(3), np.zeros(3)))
    self.c.size = [self.w_pixels, self.h_pixels]

    filename = self.name + '_calibration.yaml'
    with open(filename, 'w') as f:
      f.write(self.c.yaml())

    print('Calibration exported successfully to ' + filename)

  def getMeasuredPixImageCoord(self):
    u_meas = []
    v_meas = []
    for chessboards in self.c.good_corners:
      u_meas.append(chessboards[0][:,0][:,0])
      v_meas.append(self.h_pixels - chessboards[0][:,0][:,1])   # Flip Y-axis to traditional direction

    return u_meas, v_meas   # Lists of arrays (one per chessboard)

  def genCornerCoordinates(self, u_meas, v_meas):
    
    X = []
    Y = []

    # DEFINED FROM BOTTOM LEFT
    # for i in range(self.n_corners_y):
    #   line = np.multiply(self.d_square,np.arange(self.n_corners_x))
    #   reversedLine = line[::-1]
    #   X.append(reversedLine)

    # for i in range(self.n_corners_y):
    #   Y.append(np.multiply(i*self.d_square,np.ones(self.n_corners_x)))

    # DEFINED FROM TOP LEFT
    for i in range(self.n_corners_y):
      line = np.multiply(self.d_square,np.arange(self.n_corners_x))
      X.append(line)

    for i in range(self.n_corners_y):
      Y.append(np.multiply(i*self.d_square,np.ones(self.n_corners_x)))

    X = np.concatenate(X)
    Y = np.concatenate(Y)

    return X, Y

  def estimateHomography(self, u_meas, v_meas, X, Y):
    
    # Generate L Matrix
    L = np.zeros((2*len(u_meas),9))
    iter = np.arange(len(u_meas))

    M_i_T = np.column_stack((X,Y,np.ones((len(X)))))

    u_meas_mat = np.column_stack((u_meas,u_meas,u_meas))
    v_meas_mat = np.column_stack((v_meas,v_meas,v_meas))

    FirstStack = np.column_stack((M_i_T,np.zeros((M_i_T.shape[0],3)),-np.multiply(u_meas_mat,M_i_T)))
    SecondStack = np.column_stack((np.zeros((M_i_T.shape[0],3)),M_i_T,-np.multiply(v_meas_mat,M_i_T)))

    L = np.vstack((FirstStack,SecondStack))

    # Use SVD to find lowest singular vector - solution to min norm (Lx)
    U, s, V = np.linalg.svd(L, full_matrices=False)

    # Extract solution vector
    smallestV = V[-1]

    # Reconstruct H
    H = np.zeros((3,3))
    H[0] = smallestV[0:3]
    H[1] = smallestV[3:6]
    H[2] = smallestV[6:9]

    return H

  def getCameraIntrinsics(self, H):

    V_LENGTH = 6 # Length of the V concatenated from every image
    V_mat = np.empty((0,V_LENGTH)) # Initialize to empty matrix for concatenation

    # Loop over all images
    for i in range(H.shape[2]):
      # Get the current H
      H_cur = H[:,:,i]
      # Generate V Matrix
      V_11 = self.getVij(H_cur,1,1)
      V_12 = self.getVij(H_cur,1,2)
      V_22 = self.getVij(H_cur,2,2)
      # Concatenate results onto V_mat
      V_mat = np.vstack((V_mat,V_12,(V_11-V_22)))

    # Use SVD to find lowest singular vector - solution to min norm (Vx)
    U, s, V = np.linalg.svd(V_mat, full_matrices=False)

    # Extract solution vector
    b = V[-1]
    
    # NOTE: b = (B11;B12;B22;B13;B23;B33)^T :
    A = self.solveForIntrinsics(b)

    return A

  # Gets the Vij term of each homography matrix. i and j are in form of equation, NOT for indexes
  # Usage:  V_11 = self.getVij(H_cur,1,1)
  def getVij(self,H,i,j):
    # Adjust i and j to account for indexing
    i -= 1
    j -= 1

    H = H.T
    V_ij = np.array([H[i,0]*H[j,0] , \
          H[i,0]*H[j,1] + H[i,1]*H[j,0], \
          H[i,1]*H[j,1], \
          H[i,2]*H[j,0] + H[i,0]*H[j,2], \
          H[i,2]*H[j,1] + H[i,1]*H[j,2], \
          H[i,2]*H[j,2]])

    return V_ij

  def solveForIntrinsics(self,b_sol):
    # NOTE: b = (B11;B12;B22;B13;B23;B33)^T :
    B = np.zeros((4,4))
    B[1,1] = b_sol[0]
    B[1,2] = b_sol[1]
    B[2,2] = b_sol[2]
    B[1,3] = b_sol[3]
    B[2,3] = b_sol[4]
    B[3,3] = b_sol[5]

    A = np.zeros((3,3))

    # Solve for intrinsic parameters
    v0 = (B[1,2]*B[1,3] - B[1,1]*B[2,3])/(B[1,1]*B[2,2] - B[1,2]**2)
    lam = B[3,3] - (B[1,3]**2 + v0*(B[1,2]*B[1,3] - B[1,1]*B[2,3]))/B[1,1]
    Beta = np.sqrt(lam*B[1,1]/(B[1,1]*B[2,2] - B[1,2]**2))
    alpha = np.sqrt(lam/B[1,1])
    gamma = -B[1,2]*alpha**2*Beta/lam 
    u0 = gamma*v0/alpha - B[1,3]*alpha**2/lam

    A[0,0] = alpha 
    A[0,1] = gamma
    A[0,2] = u0 #u01
    A[1,1] = Beta
    A[1,2] = v0 #
    A[2,2] = 1

    return A

  def getExtrinsics(self, H, A):
    lam = 1/np.linalg.norm(np.dot(np.linalg.inv(A),H[:,0]))
    r1 = lam*np.dot(np.linalg.inv(A),H[:,0])
    r2 = lam*np.dot(np.linalg.inv(A),H[:,1])
    r3 = np.cross(r1,r2)

    t = lam*np.dot(np.linalg.inv(A),H[:,2])

    R = np.column_stack((r1,r2,r3))

    # Use SVD to find lowest singular vector - solution to min norm (Vx)
    U, s, V = np.linalg.svd(R, full_matrices=False)

    R_best = np.dot(U,V)

    return R, t

  def transformWorld2NormImageUndist(self, X, Y, Z, R, t):
    """
    Note: The transformation functions should only process one chessboard at a time!
    This means X, Y, Z, R, t should be individual arrays
    """
    worldCoords = np.row_stack((X,Y,Z,np.ones((len(X)))))
    cameraCoords = np.dot(np.column_stack((R,t)),worldCoords)

    X_C = cameraCoords[0]
    Y_C = cameraCoords[1]
    Z_C = cameraCoords[2]
    
    x = X_C/Z_C
    y = Y_C/Z_C


    return x, y

  def transformWorld2PixImageUndist(self, X, Y, Z, R, t, A):

    worldCoords = np.row_stack((X,Y,Z,np.ones((len(X)))))
    pixCoords = np.dot(np.dot(A,np.column_stack((R,t))),worldCoords)

    pdb.set_trace()
    
    u = pixCoords[0]/pixCoords[2]
    v = pixCoords[1]/pixCoords[2]

    return u, v

  def transformWorld2NormImageDist(self, X, Y, Z, R, t, k):
    

    return x_br, y_br

  def transformWorld2PixImageDist(self, X, Y, Z, R, t, A, k):


    return u_br, v_br

  def estimateLensDistortion(self, u_meas, v_meas, X, Y, R, t, A):
    

    return k