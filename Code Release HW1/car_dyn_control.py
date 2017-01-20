import numpy as np
from scipy import linalg
import math
import pdb

def car_dyn(x, t, ctrl, noise):

    u_0 = ctrl[0] + noise[0]
    u_1 = ctrl[1] + noise[1]

    dxdt = [u_0*np.cos(x[2]), u_0*np.sin(x[2]), u_1]

    return dxdt

# TODO Implement this part
def ctrl_traj(x,y,th,dyn_state,ctrl_prev,x_d,y_d,xd_d,yd_d,xdd_d,ydd_d,x_g,y_g,th_g):
    #(x,y,th): current state
    #dyn_state: compensator internal dynamic state
    #ctrl_prev: previous control input (V,om)
    #(xd_d, yd_d): desired Velocity
    #(xdd_d, ydd_d): desired acceleration
    #(x_g,y_g,th_g): desired final state

    # Timestep
    dt = 0.0025

    # Gains
    # kpx =
    # kpy =
    # kdx =
    # kdy =

    #Code trajectory controller (Switch to pose controller once "close" enough)
    #...FILL...#

    #Define control inputs (V,om) - without saturation constraints

    #Define accel = dV/dt

    #Integrate dynamic state with anti-windup
    if (V > 0.5 or om > 1.0) and (accel > 0.0): #integration will only make things worse
        if (V > 0.5): dyn_state_up = 0.5 #cap-off integrator at max
        else: dyn_state_up = dyn_state #or just freeze integration
    else:
        dyn_state_up = dyn_state + accel*dt

    # Apply saturation limits
    V = np.sign(V)*min(0.5, np.abs(V))
    om = np.sign(om)*min(1, np.abs(om))

    return np.array([V, om, dyn_state_up])


def wrapToPi(a):
    if isinstance(a, list):    # backwards compatibility for lists (distinct from np.array)
        return [(x + np.pi) % (2*np.pi) - np.pi for x in a]
    return (a + np.pi) % (2*np.pi) - np.pi

def ctrl_pose (x,y,th,x_g,y_g,th_g):
    #(x,y,th): current state
    #(x_g,y_g,th-g): desired final state

    # Define Controller Gains
    k1 = 1.0
    k2 = 2.0
    k3 = 1.0

    # Define relevant control parameters
    rho = math.sqrt((x-x_g)**2 + (y-y_g)**2)
    alpha = wrapToPi(math.atan2(y_g-y,x_g-x) - th)
    delta = wrapToPi(alpha + th)

    # Control Law
    V = k1*rho*math.cos(alpha)
    om = wrapToPi(k2*alpha + k1*(math.cos(alpha)*math.sin(alpha)/alpha)*(alpha + k3*delta))

    # For debugging, uncomment following line
    # pdb.set_trace()

    # Apply saturation limits
    V = np.sign(V)*min(0.5, np.abs(V))
    om = np.sign(om)*min(1, np.abs(om))

    return np.array([V, om])
