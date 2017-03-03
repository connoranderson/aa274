import numpy as np
from numpy import sin, cos
import scipy.linalg    # you may find scipy.linalg.block_diag useful
from ExtractLines import ExtractLines, normalize_line_parameters, angle_difference
from maze_sim_parameters import LineExtractionParams, NoiseParams, MapParams

import pdb


class EKF(object):

    def __init__(self, x0, P0, Q):
        self.x = x0    # Gaussian belief mean
        self.P = P0    # Gaussian belief covariance
        # Gaussian control noise covariance (corresponding to dt = 1 second)
        self.Q = Q

    # Updates belief state given a discrete control step (Gaussianity preserved by linearizing dynamics)
    # INPUT:  (u, dt)
    #       u - zero-order hold control input
    #      dt - length of discrete time step
    # OUTPUT: none (internal belief state (self.x, self.P) should be updated)
    def transition_update(self, u, dt):
        g, Gx, Gu = self.transition_model(u, dt)

        #### TODO ####
        # update self.x, self.P
        ##############

        self.x = g
        self.P = Gx.dot(self.P).dot(Gx.T) + dt * Gu.dot(self.Q).dot(Gu.T)

    # Propagates exact (nonlinear) state dynamics; also returns associated Jacobians for EKF linearization
    # INPUT:  (u, dt)
    #       u - zero-order hold control input
    #      dt - length of discrete time step
    # OUTPUT: (g, Gx, Gu)
    #      g  - result of belief mean self.x propagated according to the system dynamics with control u for dt seconds
    #      Gx - Jacobian of g with respect to the belief mean self.x
    #      Gu - Jacobian of g with respect to the control u
    def transition_model(self, u, dt):
        raise NotImplementedError(
            "transition_model must be overriden by a subclass of EKF")

    # Updates belief state according to a given measurement (with associated uncertainty)
    # INPUT:  (rawZ, rawR)
    #    rawZ - raw measurement mean
    #    rawR - raw measurement uncertainty
    # OUTPUT: none (internal belief state (self.x, self.P) should be updated)
    def measurement_update(self, rawZ, rawR):
        z, R, H = self.measurement_model(rawZ, rawR)
        # don't update if measurement is invalid (e.g., no line matches for
        # line-based EKF localization)
        if z is None:
            return

        #### TODO ####
        # update self.x, self.P
        ##############

        Sigma = H.dot(self.P).dot(H.T) + R
        K = self.P.dot(H.T).dot(np.linalg.inv(Sigma))
        self.x = self.x + K.dot(z).reshape(3,)
        self.P = self.P - K.dot(Sigma).dot(K.T)

    # Converts raw measurement into the relevant Gaussian form (e.g., a dimensionality reduction);
    # also returns associated Jacobian for EKF linearization
    # INPUT:  (rawZ, rawR)
    #    rawZ - raw measurement mean
    #    rawR - raw measurement uncertainty
    # OUTPUT: (z, R, H)
    #       z - measurement mean (for simple measurement models this may = rawZ)
    #       R - measurement covariance (for simple measurement models this may = rawR)
    #       H - Jacobian of z with respect to the belief mean self.x
    def measurement_model(self, rawZ, rawR):
        raise NotImplementedError(
            "measurement_model must be overriden by a subclass of EKF")


class Localization_EKF(EKF):

    def __init__(self, x0, P0, Q, map_lines, tf_base_to_camera, g):
        # 2xJ matrix containing (alpha, r) for each of J map lines
        self.map_lines = map_lines
        # (x, y, theta) transform from the robot base to the camera frame
        self.tf_base_to_camera = tf_base_to_camera
        self.g = g                                    # validation gate
        super(self.__class__, self).__init__(x0, P0, Q)

    # Unicycle dynamics (Turtlebot 2)
    def transition_model(self, u, dt):
        v, om = u
        x, y, th = self.x

        #### TODO ####
        # compute g, Gx, Gu
        ##############

        g = np.copy(self.x)
        Gx = np.eye(self.x.size)
        Gu = np.zeros((self.x.size, 2))

        # More Accurate Model
        DIVIDE_BY_ZERO_THRESHOLD = 10**(-15)

        if np.absolute(om) >= DIVIDE_BY_ZERO_THRESHOLD:
            th_t = om * dt + th
            x_t = (v / om) * (np.sin(om * dt + th) - np.sin(th)) + x
            y_t = (-v / om) * (np.cos(om * dt + th) - np.cos(th)) + y
            g = np.array([x_t, y_t, th_t])

            Gx[0, 2] = (v / om) * (np.cos(om * dt + th) - np.cos(th))
            Gx[1, 2] = (v / om) * (np.sin(om * dt + th) - np.sin(th))

            Gu[0, 0] = (1 / om) * (np.sin(om * dt + th) - np.sin(th))
            Gu[1, 0] = (-1 / om) * (np.cos(om * dt + th) - np.cos(th))
            Gu[0, 1] = (v / om**2) * (np.sin(th)) + (-v / om**2) * \
                np.sin(om * dt + th) + (v / om) * dt * np.cos(om * dt + th)
            Gu[1, 1] = (-v / om**2) * (np.cos(th)) + (v / om**2) * \
                np.cos(om * dt + th) + (v / om) * dt * np.sin(om * dt + th)
            Gu[2, 1] = dt
        else:
            # Use simpler model to handle singularity
            th_t = om * dt + th
            x_t = v * np.cos(th) * dt + x
            y_t = v * np.sin(th) * dt + y
            g = np.array([x_t, y_t, th_t])
            Gx = np.eye(self.x.size)
            Gu = np.zeros((self.x.size, 2))
            Gx[:, 2] = [-v * np.sin(th) * dt, v * np.cos(th) * dt, 1]
            Gu[0, :] = [np.cos(th) * dt, 0]
            Gu[1, :] = [np.sin(th) * dt, 0]
            Gu[2, :] = [0, dt]

        return g, Gx, Gu

    # Given a single map line m in the world frame, outputs the line parameters in the scanner frame so it can
    # be associated with the lines extracted from the scanner measurements
    # INPUT:  m = (alpha, r)
    #       m - line parameters in the world frame
    # OUTPUT: (h, Hx)
    #       h - line parameters in the scanner (camera) frame
    #      Hx - Jacobian of h with respect to the the belief mean self.x
    def map_line_to_predicted_measurement(self, m):
        alpha, r = m

        # Use mean expected position to compute line location
        x, y, theta = self.x
        # (x, y, theta) transform from the robot base to the camera frame
        xcam, ycam, theta_cam = self.tf_base_to_camera

        #### TODO ####
        # compute h, Hx
        ##############

        alpha_cam = alpha - theta_cam - theta

        # rotation matrix from robot reference frame to world frame
        R_cam = np.matrix([[np.cos(theta), -np.sin(theta), 0],
                           [np.sin(theta), np.cos(theta), 0],
                           [0, 0, 1]])
        # rotation matrix to line reference frame from world frame
        R_line = np.matrix([[np.cos(-alpha), -np.sin(-alpha)],
                            [np.sin(-alpha), np.cos(-alpha)]])

        # Find camera coordinates in robot frame
        # (x_cam_w, y_cam_w, theta_cam_w)
        cam_world_coords = np.squeeze(np.asarray(
            np.dot(R_cam, self.tf_base_to_camera)))
        # Offset for camera coords in world frame
        cam_w = np.array([x + cam_world_coords[0], y + cam_world_coords[1]])

        r_proj = np.squeeze(np.asarray(np.dot(R_line, cam_w)))
        r_cam = r - r_proj[0]

        # Store results in h array
        h = np.array([alpha_cam, r_cam])

        # Create and populate Hx matrix
        Hx = np.zeros((2, 3))
        Hx[0, 2] = -1
        Hx[1, 0] = -np.cos(alpha)
        Hx[1, 1] = -np.sin(alpha)
        Hx[1, 2] = -(np.cos(alpha) * (-np.sin(theta) * xcam - np.cos(theta) *
                                      ycam) + np.sin(alpha) * (np.cos(theta) * xcam - np.sin(theta) * ycam))

        flipped, h = normalize_line_parameters(h)
        if flipped:
            Hx[1, :] = -Hx[1, :]

        return h, Hx

    # Given lines extracted from the scanner data, tries to associate to each one the closest map entry
    # measured by Mahalanobis distance
    # INPUT:  (rawZ, rawR)
    #    rawZ - 2xI matrix containing (alpha, r) for each of I lines extracted from the scanner data (in scanner frame)
    #    rawR - list of I 2x2 covariance matrices corresponding to each (alpha, r) column of rawZ
    # OUTPUT: (v_list, R_list, H_list)
    #  v_list - list of at most I innovation vectors (predicted map measurement - scanner measurement)
    #  R_list - list of len(v_list) covariance matrices of the innovation vectors (from scanner uncertainty)
    # H_list - list of len(v_list) Jacobians of the innovation vectors with
    # respect to the belief mean self.x
    def associate_measurements(self, rawZ, rawR):

        #### TODO ####
        # compute v_list, R_list, H_list
        ##############

        v_list = []
        R_list = []
        H_list = []

        I = rawZ.shape[1]
        J = self.map_lines.shape[1]

        # If nothing is passed in, or map is empty, don't continue
        if I==0 or J==0:
            return v_list,R_list,H_list

        # Loop over all observed lines
        for i in range(I):

            # Reset min to infinity for next test
            d_min = float("inf")

            # Loop over all maps
            for j in range(J):
                # Compute H for each map line (j)
                h, Hx = self.map_line_to_predicted_measurement(self.map_lines[:,j])
                # Compute Mahalanobis distance
                v_ij = rawZ[:,i] - h
                S_ij = Hx.dot(self.P).dot(Hx.T) + rawR[i]
                d_ij = v_ij.T.dot(np.linalg.inv(S_ij)).dot(v_ij)  # Mahalanobis distance

                # If this distance is the smallest we've seen, replace the min variables
                if d_ij < d_min:
                    d_min = d_ij
                    v_min = v_ij
                    R_min = rawR[i]
                    H_min = Hx
            
            # Add the smallest mahalanobis distance line to v_list
            if d_min < self.g**2:
                v_list.append(v_min)
                R_list.append(R_min)
                H_list.append(H_min)

        return v_list, R_list, H_list

    # Assemble one joint measurement, covariance, and Jacobian from the individual values corresponding to each
    # matched line feature
    def measurement_model(self, rawZ, rawR):
        v_list, R_list, H_list = self.associate_measurements(rawZ, rawR)
        if not v_list:
            print "Scanner sees", rawZ.shape[1], "line(s) but can't associate them with any map entries"
            return None, None, None

        #### TODO ####
        # compute z, R, H
        ##############
        N = len(v_list)
        
        z = np.array([]).reshape(0,1)
        R = np.array([]).reshape(0,R_list[0].shape[1])
        H = np.array([]).reshape(0,H_list[0].shape[1])
        
        for i in range(N):
            z = np.vstack((z,v_list[i].reshape(2,1))) 
            if R.shape[0] != 0:
                R = scipy.linalg.block_diag(R.copy(),R_list[i])
            else:
                R = R_list[i]    
            H = np.vstack((H,H_list[i])) 

        return z, R, H


class SLAM_EKF(EKF):

    def __init__(self, x0, P0, Q, tf_base_to_camera, g):
        # (x, y, theta) transform from the robot base to the camera frame
        self.tf_base_to_camera = tf_base_to_camera
        self.g = g                                    # validation gate
        super(self.__class__, self).__init__(x0, P0, Q)

    # Combined Turtlebot + map dynamics
    # Adapt this method from Localization_EKF.transition_model.
    def transition_model(self, u, dt):
        v, om = u
        x, y, th = self.x[:3]

        #### TODO ####
        # compute g, Gx, Gu (some shape hints below)
        g = np.copy(self.x)
        Gx = np.eye(self.x.size)
        Gu = np.zeros((self.x.size, 2))
        ##############

        return g, Gx, Gu

    # Combined Turtlebot + map measurement model
    # Adapt this method from Localization_EKF.measurement_model.
    #
    # The ingredients for this model should look very similar to those for Localization_EKF.
    # In particular, essentially the only thing that needs to change is the computation
    # of Hx in map_line_to_predicted_measurement and how that method is called in
    # associate_measurements (i.e., instead of getting world-frame line parameters from
    # self.map_lines, you must extract them from the state self.x)
    def measurement_model(self, rawZ, rawR):
        v_list, R_list, H_list = self.associate_measurements(rawZ, rawR)
        if not v_list:
            print "Scanner sees", rawZ.shape[1], "line(s) but can't associate them with any map entries"
            return None, None, None

        #### TODO ####
        # compute z, R, H (should be identical to Localization_EKF.measurement_model above)
        ##############

        return z, R, H

    # Adapt this method from Localization_EKF.map_line_to_predicted_measurement.
    #
    # Note that instead of the actual parameters m = (alpha, r) we pass in the map line index j
    # so that we know which components of the Jacobian to fill in.
    def map_line_to_predicted_measurement(self, j):
        # j is zero-indexed! (yeah yeah I know this doesn't match the pset
        # writeup)
        alpha, r = self.x[3 + 2 * j, 3 + 2 * j + 2]

        #### TODO ####
        # compute h, Hx (you may find the skeleton for computing Hx below
        # useful)

        Hx = np.zeros((2, self.x.size))
        Hx[:, :3] = FILLMEIN
        # First two map lines are assumed fixed so we don't want to propagate
        # any measurement correction to them
        if j > 1:
            Hx[0, 3 + 2 * j] = FILLMEIN
            Hx[1, 3 + 2 * j] = FILLMEIN
            Hx[0, 3 + 2 * j + 1] = FILLMEIN
            Hx[1, 3 + 2 * j + 1] = FILLMEIN

        ##############

        flipped, h = normalize_line_parameters(h)
        if flipped:
            Hx[1, :] = -Hx[1, :]

        return h, Hx

    # Adapt this method from Localization_EKF.associate_measurements.
    def associate_measurements(self, rawZ, rawR):

        #### TODO ####
        # compute v_list, R_list, H_list
        ##############

        return v_list, R_list, H_list
