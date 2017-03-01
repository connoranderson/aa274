# 2/22/2017 - stay tuned for a more comprehensive validation script

import numpy as np
from numpy.linalg import norm
import scipy.linalg
import cPickle as pickle
import matplotlib.pyplot as plt
from ExtractLines import ExtractLines
from maze_sim_parameters import LineExtractionParams, NoiseParams, MapParams
from ekf import Localization_EKF, SLAM_EKF

### PROBLEM 1

def validate_localization_transition_update(fname = "validation_run.p", show_plot = True):
    validation_run = pickle.load(open(fname, "rb"))

    EKF = Localization_EKF(validation_run["states"][0][1], NoiseParams["P0"], NoiseParams["Q"],
                           MapParams, validation_run["tf_base_to_camera"], NoiseParams["g"])
    open_loop_states = [EKF.x]
    for i in range(len(validation_run["controls"]) - 1):
        u = validation_run["controls"][i][1]
        dt = validation_run["controls"][i+1][0] - validation_run["controls"][i][0]
        EKF.transition_update(u, dt)
        open_loop_states.append(EKF.x)

    ground_truth_x = [s[0] for t, s in validation_run["states"]]
    ground_truth_y = [s[1] for t, s in validation_run["states"]]
    open_loop_x = [s[0] for s in open_loop_states]
    open_loop_y = [s[1] for s in open_loop_states]
    plt.plot(ground_truth_x, ground_truth_y, label="ground truth", color="black")
    plt.plot(open_loop_x, open_loop_y, label="open loop", color="green")
    plt.axis("equal")
    if show_plot:
        plt.legend(loc=0)
        plt.show()

def validate_localization_EKF(fname = "validation_run.p", show_plot = True):
    validate_localization_transition_update(fname, False)

    validation_run = pickle.load(open(fname, "rb"))
    EKF = Localization_EKF(validation_run["states"][0][1], NoiseParams["P0"], NoiseParams["Q"],
                           MapParams, validation_run["tf_base_to_camera"],  NoiseParams["g"])
    EKF_states = [EKF.x]
    scan_states = []
    scan_idx = 0
    for i in range(len(validation_run["controls"]) - 1):
        u = validation_run["controls"][i][1]
        t1 = validation_run["controls"][i+1][0]
        t0 = validation_run["controls"][i][0]
        while scan_idx < len(validation_run["scans"]) and validation_run["scans"][scan_idx][0] < t1:
            EKF.transition_update(u, validation_run["scans"][scan_idx][0] - t0)
            t0 = validation_run["scans"][scan_idx][0]
            alpha, r, C_AR, _, _ = ExtractLines(validation_run["scans"][scan_idx][1],
                                                validation_run["scans"][scan_idx][2],
                                                LineExtractionParams,
                                                NoiseParams["var_theta"],
                                                NoiseParams["var_rho"])
            Z = np.vstack((alpha, r))
            EKF.measurement_update(Z, C_AR)
            scan_states.append(EKF.x)
            scan_idx = scan_idx + 1
        EKF.transition_update(u, t1 - t0)
        EKF_states.append(EKF.x)

    EKF_x = [s[0] for s in EKF_states]
    EKF_y = [s[1] for s in EKF_states]
    plt.plot(EKF_x, EKF_y, label="EKF (known map)", color="red")
    scan_x = [s[0] for s in scan_states]
    scan_y = [s[1] for s in scan_states]
    plt.scatter(scan_x, scan_y, marker = "x", label="measurement update", color="blue")
    if show_plot:
        plt.legend(loc=0)
        plt.show()

def validate_localization_transition_model(fname = "validation_run.p"):
    validation_run = pickle.load(open(fname, "rb"))

    EKF = Localization_EKF(validation_run["states"][0][1], NoiseParams["P0"], NoiseParams["Q"],
                           MapParams, validation_run["tf_base_to_camera"], NoiseParams["g"])

    for i, tc in enumerate(validation_run["controls"]):
        timestamp, control = tc
        g, Gx, Gu = EKF.transition_model(control, .1)
        g_ref, Gx_ref, Gu_ref = validation_run["transition_model_validation"][i]
        if norm(g - g_ref) + norm(Gx - Gx_ref) + norm(Gu - Gu_ref) > 1e-6:
            print "At state x = {0} with u = {1} and dt = {2} got Localization_EKF.transition_model output:\n".format(EKF.x, control, .1)
            print g
            print Gx
            print Gu
            print "\nvs. the expected values\n"
            print g_ref
            print Gx_ref
            print Gu_ref
            return False
    
    print "Localization_EKF.transition_model seems to be correct"
    return True

def validate_localization_map_line_to_predicted_measurement(fname = "validation_run.p"):
    validation_run = pickle.load(open(fname, "rb"))

    EKF = Localization_EKF(validation_run["states"][0][1], NoiseParams["P0"], NoiseParams["Q"],
                           MapParams, validation_run["tf_base_to_camera"], NoiseParams["g"])

    for j in range(EKF.map_lines.shape[1]):
        h, Hx = EKF.map_line_to_predicted_measurement(EKF.map_lines[:,j])
        h_ref, Hx_ref = validation_run["map_line_to_predicted_measurement_validation"][j]
        if norm(h - h_ref) + norm(Hx - Hx_ref) > 1e-6:
            print "At state x = {0} with m = {1} got Localization_EKF.map_line_to_predicted_measurement_validation output:\n".format(EKF.x, EKF.map_lines[:,j])
            print h
            print Hx
            print "\nvs. the expected values\n"
            print h_ref
            print Hx_ref
            return False

    print "Localization_EKF.map_line_to_predicted_measurement seems to be correct"
    return True

def validate_localization_associate_measurements(fname = "validation_run.p"):
    if not validate_localization_transition_model(fname) or not validate_localization_map_line_to_predicted_measurement(fname):
        print "Validation of associate_measurements cannot proceed."
        return False

    validation_run = pickle.load(open(fname, "rb"))

    EKF = Localization_EKF(validation_run["states"][0][1], NoiseParams["P0"], NoiseParams["Q"],
                           MapParams, validation_run["tf_base_to_camera"], NoiseParams["g"])
    scan_idx = 0
    for i in range(len(validation_run["controls"]) - 1):
        u = validation_run["controls"][i][1]
        t1 = validation_run["controls"][i+1][0]
        t0 = validation_run["controls"][i][0]
        while scan_idx < len(validation_run["scans"]) and validation_run["scans"][scan_idx][0] < t1:
            EKF.transition_update(u, validation_run["scans"][scan_idx][0] - t0)
            t0 = validation_run["scans"][scan_idx][0]
            alpha, r, C_AR, _, _ = ExtractLines(validation_run["scans"][scan_idx][1],
                                                validation_run["scans"][scan_idx][2],
                                                LineExtractionParams,
                                                NoiseParams["var_theta"],
                                                NoiseParams["var_rho"])
            Z = np.vstack((alpha, r))

            v_list, R_list, H_list = EKF.associate_measurements(Z, C_AR)
            v_list_ref, R_list_ref, H_list_ref = validation_run["associate_measurements_validation"][scan_idx]
            if len(v_list) != len(v_list_ref) or len(R_list) != len(R_list_ref) or len(H_list) != len(H_list_ref):
                print "You may have an error in Localization_EKF.associate_measurements."
                return False
            permutation = [np.argmin([norm(R - R_ref) for R_ref in R_list_ref]) for R in R_list]
            v_error = sum([norm(v_list[j] - v_list_ref[k]) for j, k in enumerate(permutation)])
            R_error = sum([norm(R_list[j] - R_list_ref[k]) for j, k in enumerate(permutation)])
            H_error = sum([norm(H_list[j] - H_list_ref[k]) for j, k in enumerate(permutation)])
            if v_error + R_error + H_error > 1e-6:
                print "You may have an error in Localization_EKF.associate_measurements."
                return False

            EKF.measurement_update(Z, C_AR)
            scan_idx = scan_idx + 1
        EKF.transition_update(u, t1 - t0)

    print "Localization_EKF.associate_measurements seems to be correct"
    return True


### PROBLEM 2

def validate_SLAM_EKF(fname = "validation_run.p"):
    validate_localization_transition_update(fname, False)

    validation_run = pickle.load(open(fname, "rb"))

    x0_pose = validation_run["states"][0][1]
    P0_pose = NoiseParams["P0"]

    N_map_lines = MapParams.shape[1]
    x0_map = MapParams.T.flatten()
    x0_map[4:] = x0_map[4:] + np.vstack((NoiseParams["std_alpha"]*np.random.randn(N_map_lines-2),    # first two lines fixed
                                         NoiseParams["std_r"]*np.random.randn(N_map_lines-2))).T.flatten()
    P0_map = np.diag(np.concatenate((np.zeros(4),
                                     np.array([[NoiseParams["std_alpha"]**2 for i in range(N_map_lines-2)],
                                               [NoiseParams["std_r"]**2 for i in range(N_map_lines-2)]]).T.flatten())))

    EKF = SLAM_EKF(np.concatenate((x0_pose, x0_map)), scipy.linalg.block_diag(P0_pose, P0_map), NoiseParams["Q"],
                   validation_run["tf_base_to_camera"],  NoiseParams["g"])
    EKF_states = [EKF.x]
    scan_states = []
    scan_idx = 0
    for i in range(len(validation_run["controls"]) - 1):
        u = validation_run["controls"][i][1]
        t1 = validation_run["controls"][i+1][0]
        t0 = validation_run["controls"][i][0]
        while scan_idx < len(validation_run["scans"]) and validation_run["scans"][scan_idx][0] < t1:
            EKF.transition_update(u, validation_run["scans"][scan_idx][0] - t0)
            t0 = validation_run["scans"][scan_idx][0]
            alpha, r, C_AR, _, _ = ExtractLines(validation_run["scans"][scan_idx][1],
                                                validation_run["scans"][scan_idx][2],
                                                LineExtractionParams,
                                                NoiseParams["var_theta"],
                                                NoiseParams["var_rho"])
            Z = np.vstack((alpha, r))
            EKF.measurement_update(Z, C_AR)
            scan_states.append(EKF.x)
            scan_idx = scan_idx + 1
        EKF.transition_update(u, t1 - t0)
        EKF_states.append(EKF.x)

    EKF_x = [s[0] for s in EKF_states]
    EKF_y = [s[1] for s in EKF_states]
    plt.plot(EKF_x, EKF_y, label="EKF (noisy map)", color="orange")
    scan_x = [s[0] for s in scan_states]
    scan_y = [s[1] for s in scan_states]
    plt.scatter(scan_x, scan_y, marker = "x", label="measurement update", color="blue")
    plt.legend(loc=0)
    plt.show()


if __name__ == '__main__':
    ### PROBLEM 1
    # validate_localization_transition_update()
    # validate_localization_EKF()

    ## Subcomponent validation
    # validate_localization_transition_model()
    # validate_localization_map_line_to_predicted_measurement()
    # validate_localization_associate_measurements()

    ### PROBLEM 2

    # validate_SLAM_EKF()