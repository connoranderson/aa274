#!/usr/bin/python

import matplotlib.pyplot as plt
import numpy as np
import pdb

from cam_calibrator import CameraCalibrator


def main():
  cc = CameraCalibrator()

  cal_img_path = './webcam_12'  # Location of calibration images
  name = 'webcam'               # Name of the camera
  n_corners = [7, 9]            # Corner grid dimensions
  square_length = 0.0205        # Chessboard square length in meters

  display_flag = False
  cc.loadImages(cal_img_path, name, n_corners, square_length, display_flag)

  u_meas, v_meas = cc.getMeasuredPixImageCoord()
  
  # Add your code here!

  NUM_IMAGES = 12
  IMAGE_LENGTH = len(u_meas[0])

  # Get coordinates for world frame
  X, Y = cc.genCornerCoordinates(u_meas, v_meas)
  Z = np.zeros(len(X))

  # Generate homography matrices (H) for each image. Store in 3d matrix
  H_full = np.zeros((NUM_IMAGES,3,3))

  # Loop over all images
  for i in range(NUM_IMAGES):
    H = cc.estimateHomography(u_meas[i],v_meas[i],X,Y)
    H_full[i] = H

  A = cc.getCameraIntrinsics(H_full)

  # Generate homography matrices (H) for each image. Store in 3d matrix
  R_full = np.zeros((NUM_IMAGES,3,3))
  t_full = np.zeros((NUM_IMAGES,3))
  X_full = np.zeros((NUM_IMAGES,IMAGE_LENGTH))
  Y_full = np.zeros((NUM_IMAGES,IMAGE_LENGTH))
  Z_full = np.zeros((NUM_IMAGES,IMAGE_LENGTH))

  # Loop over all images
  for i in range(NUM_IMAGES):
    R,t = cc.getExtrinsics(H_full[i],A)
    R_full[i] = R
    t_full[i] = t
    X_full[i] = X
    Y_full[i] = Y
    Z_full[i] = Z

  # u,v = cc.transformWorld2PixImageUndist(X, Y, Z, R_full[0], t_full[0], A)


  # plt.figure()
  # plt.grid('on')
  # plt.plot(u,v,'go',markerfacecolor='green',markersize=5)
  # plt.plot(u_meas[0],v_meas[0],'go',markerfacecolor='red',markersize=5)
  # plt.ylabel('V')
  # plt.axis('equal')
  # plt.show()

  # pdb.set_trace()


  # cc.plotBoardPixImages(u_meas, v_meas, X_full, Y_full, R_full, t_full, A)

  # cc.plotBoardLocations(X_full, Y_full, R_full, t_full)

  k = cc.estimateLensDistortion(u_meas, v_meas, X_full, Y_full, Z_full, R_full, t_full, A)

  # cc.plotBoardPixImages(u_meas, v_meas, X_full, Y_full, R_full, t_full, A, k)


  cc.undistortImages(A,k)


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        import traceback
        traceback.print_exc()