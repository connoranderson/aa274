import numpy as np
import math
from scipy import linalg
import scikits.bvp_solver
import matplotlib.pyplot as plt

import pdb


def q1_ode_fun(tau,z):

    #Return array containing RHS of ODEs
    #z = [x y th p1 p2 p3 r]

    # Use dHdu eqtns to define w and V
    om = -z[5]/2
    V = (-z[3]*math.cos(z[2]) - z[4]*math.sin(z[2]))/2

    # ODE Equations
    x_dot = V*math.cos(z[2])
    y_dot = V*math.sin(z[2])
    th_dot = om
    dHdth = z[3]*V*math.sin(z[2]) - z[4]*V*math.cos(z[2])
    p_dot = np.hstack((0,0,dHdth))
    r_dot = 0 #dummy state ode

    outputArr = z[6]*np.hstack((x_dot,y_dot,th_dot,p_dot,r_dot))

    return outputArr

def q1_bc_fun(ya, yb):
    #z = [x y th p1 p2 p3 r]

    #lambda
    lambda_test = 0.2

    #goal pose
    x_g = 5
    y_g = 5
    th_g = -np.pi/2.0
    xf = [x_g, y_g, th_g]

    #initial pose
    x0 = [0, 0, -np.pi/2.0]

    # Find V and w using right BCs
    w = -yb[5]/2
    V = (-yb[3]*math.cos(yb[2]) - yb[4]*math.sin(yb[2]))/2

    #Code boundary condition residuals

    #Return a tuple containing 2 arrays - left and right side BC residuals
    #Note: len(left BCs) + len(right BCs) = num of ODEs

    #Left BCs
    BC_left = np.array([ya[0]-x0[0], ya[1]-x0[1], ya[2]-x0[2]])

    #Free final time constraint
    H_f = lambda_test + V**2 + w**2 + yb[3]*V*math.cos(yb[2]) + yb[4]*V*math.sin(yb[2]) + yb[5]*w 
    # CHECK - Should only be in terms of yb!

    #Right BCs
    BC_right = np.array([yb[0]-xf[0], yb[1]-xf[1], yb[2]-xf[2], H_f])

    return (BC_left, BC_right)

def States_Init():
    #z = [x y th p1 p2 p3 r]
    #(2.5,2.5,np.pi/2.0,1,-2,1,5.0)
    return (2.0,2.0,-np.pi/2.0,-2,-2,0.5,20.0)

#Define solver state: y = [x, y, th, ...? ]
problem = scikits.bvp_solver.ProblemDefinition(num_ODE=7, #Number of ODes
                                            num_parameters = 0, #Number of parameters
                                            num_left_boundary_conditions = 3, #Number of left BCs
                                            boundary_points = (0,1), #Boundary points of independent coordinate
                                            function = q1_ode_fun, #ODE function
                                            boundary_conditions = q1_bc_fun) #BC function

#Defne initial guess as a tuple (constant solution)
guess = States_Init()

soln = scikits.bvp_solver.solve(problem, solution_guess = guess)

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

# Extract solution vectors
xtraj = y[:,0]
ytraj = y[:,1] 
thtraj = y[:,2]

xdot = np.diff(xtraj)/dt
ydot = np.diff(ytraj)/dt
om = np.diff(thtraj)/dt

xdot = np.append(0,xdot)
ydot = np.append(0,ydot)
om = np.append(0,om)


# Apply a sign to V to account for reverse direction
N = xdot.size
V = np.sqrt(np.power(xdot,2) + np.power(ydot,2))
for i in range(1, N):
    if math.atan2(ydot[i],xdot[i]) != math.atan2(V[i]*math.sin(thtraj[i]),(V[i]*math.cos(thtraj[i]))):
        V[i] = -V[i]


# Save Data
# Save the state and control histories as a .npy file with the format (x; y; theta; V; w).
data = np.column_stack((xtraj, ytraj, thtraj, V, om))
np.save('traj_opt_data',data)

pdb.set_trace()

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
