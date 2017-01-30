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

  display_flag = True
  cc.loadImages(cal_img_path, name, n_corners, square_length, display_flag)

  u_meas, v_meas = cc.getMeasuredPixImageCoord()
  
  # Add your code here!



if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        import traceback
        traceback.print_exc()