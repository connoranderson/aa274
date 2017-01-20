import numpy as np
import math
from scipy import linalg
import scikits.bvp_solver
import matplotlib.pyplot as plt


def q1_ode_fun(tau, y):

    #Code in the BVP ODEs


    return #...FILL...#


def q1_bc_fun(ya, yb):

    #lambda
    lambda_test = 20

    #goal pose
    x_g = 5
    y_g = 5
    th_g = -np.pi/2.0
    xf = [x_g, y_g, th_g]

    #initial pose
    x0 = [0, 0, -np.pi/2.0]

    #Code boundary condition residuals

    return #...FILL...#

#Define solver state: y = [x, y, th, ...? ]
problem = scikits.bvp_solver.ProblemDefinition(#...FILL...#
                                                )

soln = scikits.bvp_solver.solve(problem, solution_guess = (#...FILL...#
                                                            ))

dt = 0.005

# Test if solution flips
y_0 = soln(0)
flip = 0
if y_0[-1] < 0:
    t_f = -y_0[-1]
    flip = 1
else:
    t_f = y_0[-1]

t = np.arange(0,t_f,dt)
y = soln(t/t_f)
if flip:
    y[3:7,:] = -y[3:7,:]
y = y.T # solution arranged column-wise

V = #...FILL...#
om = #...FILL...#

V = np.array([V]).T # Convert to 1D column matrices
om = np.array([om]).T

#Save Data
data = #...FILL...#
np.save('traj_opt_data',data)

# Plots
plt.figure()
plt.plot(y[:,0], y[:,1],'k-',linewidth=2)
plt.quiver(y[1:-1:200,0],y[1:-1:200,1],np.cos(y[1:-1:200,2]),np.sin(y[1:-1:200,2]))
plt.grid('on')
plt.plot(0,0,'go',markerfacecolor='green',markersize=15)
plt.plot(5,5,'ro',markerfacecolor='red', markersize=15)
plt.xlabel('X'); plt.ylabel('Y')

plt.figure()
plt.plot(t, V,linewidth=2)
plt.plot(t, om,linewidth=2)
plt.grid('on')
plt.xlabel('Time [s]')
plt.legend(['V [m/s]', '$\omega$ [rad/s]'],loc='center left', bbox_to_anchor=(1,0.5))

plt.show()
