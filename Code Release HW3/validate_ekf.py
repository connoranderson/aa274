# 2/22/2017 - stay tuned for a more comprehensive validation script

import numpy as np
import cPickle as pickle
import matplotlib.pyplot as plt
from ExtractLines import ExtractLines
from maze_sim_parameters import LineExtractionParams, NoiseParams, MapParams
from ekf import Localization_EKF, SLAM_EKF

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

def validate_localization_EKF(fname = "validation_run.p"):
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
    plt.plot(EKF_x, EKF_y, label="EKF", color="red")
    scan_x = [s[0] for s in scan_states]
    scan_y = [s[1] for s in scan_states]
    plt.scatter(scan_x, scan_y, marker = "x", label="measurement update", color="blue")
    plt.legend(loc=0)
    plt.show()

if __name__ == '__main__':
    validate_localization_transition_update()
    # validate_localization_EKF()