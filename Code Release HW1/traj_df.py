import numpy as np
import math
import scipy as Sci
from scipy import linalg
import scipy.integrate as integrate
import matplotlib.pyplot as plt
import pdb

# Constants
t_f = 15
V_max = 0.5
om_max = 1

# Initial conditions
x_0 = 0
y_0 = 0
V_0 = V_max;
th_0 = -np.pi/2;

# Final conditions

x_f = 5;
y_f = 5;
V_f = V_max;
th_f = -np.pi/2;

# Solve Linear equations:
A = np.matrix([[1, 0, 0, 0, 0, 0, 0, 0],  # X(0)
	[1, t_f, t_f**2, t_f**3, 0, 0, 0, 0], # X(tf)
	[0, 0, 0, 0, 1, 0, 0, 0], # Y(0)
	[0, 0, 0, 0, 1, t_f, t_f**2, t_f**3], # Y(tf)
	[0, 1, 0, 0, 0, 0, 0, 0], # V(0)
	[0, 1, 2*t_f, 3*t_f**2, 0, 0, 0, 0],
	[0, 0, 0, 0, 0, 1, 0, 0],
	[0, 0, 0, 0, 0, 1, 2*t_f, 3*t_f**2]])

b = np.matrix([x_0,x_f,y_0,y_f,0,0,-0.5,-0.5])
b = b.T

x_sol = np.linalg.solve(A,b)


# Compute traj
dt = 0.005
N = int (t_f/dt)
t = dt*np.array(range(N+1)) # t[0],....,t[N]
t = t.T
data = np.zeros((N+1,9))

#Compute trajectory, store in data, format: [x,y,th,V,om,xd,yd,xdd,ydd]
xtraj = x_sol.item(0) + x_sol.item(1)*t + x_sol.item(2)*np.power(t,2) + x_sol.item(3)*np.power(t,3)
ytraj = x_sol.item(4) + x_sol.item(5)*t + x_sol.item(6)*np.power(t,2) + x_sol.item(7)*np.power(t,3)

# Preallocate Arrays and set initial conditions
xd_traj = np.copy(xtraj)
yd_traj = np.copy(xtraj)
xdd_traj = np.copy(xtraj)
ydd_traj = np.copy(xtraj)
th_traj = np.copy(xtraj)
V_traj = np.copy(xtraj)
w_traj = np.copy(xtraj)

xdd_traj[0] = 0
ydd_traj[0] = 0
xd_traj[0] = 0
yd_traj[0] = -1
th_traj[0] = -np.pi/2
V_traj[0] = -1
w_traj[0] = 0

# Solve for xdot, ydot, thdot, w, and V
for i in range(1, N+1):
    xd_traj[i] = (xtraj[i]-xtraj[i-1])/dt
    yd_traj[i] = (ytraj[i]-ytraj[i-1])/dt
    xdd_traj[i] = (xd_traj[i] - xd_traj[i-1])/dt
    ydd_traj[i] = (yd_traj[i] - yd_traj[i-1])/dt
    if xd_traj[i] != 0:
    	th_traj[i] = math.atan(yd_traj[i]/xd_traj[i])
    else:
    	th_traj[i] = math.asin(yd_traj[i]/np.absolute(yd_traj[i]))

    w_traj[i] = (th_traj[i]-th_traj[i-1])/dt
    V_traj = np.sqrt(np.power(xd_traj,2) + np.power(yd_traj,2))

# Store in data (sorry about memory inefficiencies :P) [x,y,th,V,om,xd,yd,xdd,ydd]
data[:,0] = xtraj
data[:,1] = ytraj
data[:,2] = th_traj
data[:,3] = V_traj
data[:,4] = w_traj
data[:,5] = xd_traj
data[:,6] = yd_traj
data[:,7] = xd_traj
data[:,8] = yd_traj


# Re-scaling - Compute scaled trajectory, store in data_scaled

# Find S variable at 5ms increments
S = integrate.cumtrapz(V_traj, t, initial=0)

# plt.figure()
# plt.plot(S,V_traj,'k-',linewidth=2)
# plt.grid('on')
# plt.xlabel('S'); plt.ylabel('V')

# Apply constraints to velocity vector
V_tilde = np.copy(V_traj)

for i in range(1, N+1):
	V_tilde[i] = min(0.5, V_traj[i])
	if w_traj[i] != 0:
		V_tilde[i] = min(V_tilde[i], np.absolute(V_traj[i]/w_traj[i]))

ws = w_traj/V_traj
plt.figure()
plt.plot(S,V_tilde,'k-',linewidth=2)
plt.grid('on')
plt.xlabel('S'); plt.ylabel('V_tilde')

# Get rescaled time 
funct = np.asarray(np.power(V_tilde,-1)).squeeze()
t_tilde = integrate.cumtrapz(funct, S, initial=0)
w_tilde = ws*V_tilde

# Rescale t to 5ms increments
t_tilde_5ms = np.arange(0,max(t_tilde),dt)

# Rescale s to 5ms increments
s_tilde = integrate.cumtrapz(V_tilde, t_tilde, initial=0)
s_tilde_5ms = np.interp(t_tilde_5ms,t_tilde,s_tilde)

# Get w, V, x, y, theta, xdot, ydot, xddot, and yddot

V_5ms = np.interp(t_tilde_5ms,t_tilde,V_tilde)

w_5ms = np.interp(t_tilde_5ms,t_tilde,w_tilde)

theta = integrate.cumtrapz(w_5ms,t_tilde_5ms, initial=0)

x_5ms = np.interp(s_tilde_5ms,s_tilde,xtraj)
y_5ms = np.interp(s_tilde_5ms,s_tilde,ytraj)
th_5ms = np.interp(s_tilde_5ms,s_tilde,th_traj)

x_dot_5ms = V_5ms*np.cos(th_5ms)
y_dot_5ms = V_5ms*np.sin(th_5ms)

x_ddot_5ms = np.diff(x_dot_5ms)/dt
y_ddot_5ms = np.diff(y_dot_5ms)/dt

x_ddot_5ms = np.append(x_ddot_5ms,[0])
y_ddot_5ms = np.append(y_ddot_5ms,[0])

# Store solution values [x,y,th,V,om,xd,yd,xdd,ydd]
N = x_5ms.size
data_scaled = np.zeros((N,9))

data_scaled[:,0] = x_5ms
data_scaled[:,1] = y_5ms
data_scaled[:,2] = th_5ms
data_scaled[:,3] = V_5ms
data_scaled[:,4] = w_5ms
data_scaled[:,5] = x_dot_5ms
data_scaled[:,6] = y_dot_5ms
data_scaled[:,5] = x_ddot_5ms
data_scaled[:,6] = y_ddot_5ms

#Save data
np.save('traj_df_data',data_scaled)

# pdb.set_trace()

t_scaled = t_tilde_5ms
s_scaled = s_tilde_5ms

# Plots
plt.figure()
plt.plot(data_scaled[:,0], data_scaled[:,1],'k-',linewidth=2)
plt.grid('on')
plt.plot(x_0,y_0,'go',markerfacecolor='green',markersize=15)
plt.plot(x_f,y_f,'ro',markerfacecolor='red', markersize=15)
plt.xlabel('X'); plt.ylabel('Y')

plt.figure()
plt.subplot(2,1,1)
plt.plot(t, data[:,3:5],linewidth=2)
plt.grid('on')
plt.xlabel('Time [s]')
plt.legend(['V [m/s]', '$\omega$ [rad/s]'],loc='center left', bbox_to_anchor=(1,0.5))
plt.title('Original')

plt.subplot(2,1,2)
plt.plot(t_scaled,data_scaled[:,3:5],linewidth=2)
plt.grid('on')
plt.xlabel('Time [s]')
plt.legend(['V [m/s]', '$\omega$ [rad/s]'],loc='center left', bbox_to_anchor=(1,0.5))
plt.title('Scaled')

plt.figure()
plt.plot(t,S,'b-',linewidth=2)
plt.grid('on')
plt.plot(t_scaled,s_scaled,'r-',linewidth=2)
plt.xlabel('Time [s]')
plt.ylabel('Arc-length [m]')

plt.show()


