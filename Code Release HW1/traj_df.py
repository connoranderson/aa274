import numpy as np
import math
import scipy as Sci
from scipy import linalg
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
# pdb.set_trace()

x_sol = np.linalg.solve(A,b)

# pdb.set_trace()

# Compute traj
dt = 0.005
N = int (t_f/dt)
t = dt*np.array(range(N+1)) # t[0],....,t[N]
t = t.T
data = np.zeros((N+1,9))

#Compute trajectory, store in data, format: [x,y,th,V,om,xd,yd,xdd,ydd]
xtraj = x_sol.item(0) + x_sol.item(1)*t + x_sol.item(2)*np.power(t,2) + x_sol.item(3)*np.power(t,3)
ytraj = x_sol.item(4) + x_sol.item(5)*t + x_sol.item(6)*np.power(t,2) + x_sol.item(7)*np.power(t,3)



xd_traj = xtraj[:]
yd_traj = ytraj[:]

xd_traj[0] = 0
yd_traj[0] = -1

# th_traj = xtraj
# th_traj[0] = -np.pi/2
# V_traj = xtraj
# w_traj = xtraj

# V_traj[0] = 1
# w_traj[0] = 0

# pdb.set_trace()



# for i in range(1, N+1):
#     xd_traj[i] = (xtraj[i]-xtraj[i-1])/dt
#     yd_traj[i] = (ytraj[i]-ytraj[i-1])/dt
#     th_traj[i] = math.atan(yd_traj[i]/xd_traj[i])
#     w_traj[i] = (th_traj[i]-th_traj[i-1])/dt

# data[:,0] = xtraj
# data[:,1] = ytraj
# data[:,2] = th_traj
# data[:,3] = xd_traj
# data[:,4] = yd_traj
# data[:,5] = V_traj
# data[:,6] = w_traj


# Plots
plt.figure()
plt.plot(xtraj, ytraj,'k-',linewidth=2)
plt.grid('on')
plt.plot(x_0,y_0,'go',markerfacecolor='green',markersize=15)
plt.plot(x_f,y_f,'ro',markerfacecolor='red', markersize=15)
plt.xlabel('X'); plt.ylabel('Y')


# Re-scaling - Compute scaled trajectory, store in data_scaled
#...FILL...#

# Plots
# plt.figure()
# plt.plot(data_scaled[:,0], data_scaled[:,1],'k-',linewidth=2)
# plt.grid('on')
# plt.plot(x_0,y_0,'go',markerfacecolor='green',markersize=15)
# plt.plot(x_f,y_f,'ro',markerfacecolor='red', markersize=15)
# plt.xlabel('X'); plt.ylabel('Y')

# plt.figure()
# plt.subplot(2,1,1)
# plt.plot(t, data[:,3:5],linewidth=2)
# plt.grid('on')
# plt.xlabel('Time [s]')
# plt.legend(['V [m/s]', '$\omega$ [rad/s]'],loc='center left', bbox_to_anchor=(1,0.5))
# plt.title('Original')

# plt.subplot(2,1,2)
# plt.plot(t_scaled,data_scaled[:,3:5],linewidth=2)
# plt.grid('on')
# plt.xlabel('Time [s]')
# plt.legend(['V [m/s]', '$\omega$ [rad/s]'],loc='center left', bbox_to_anchor=(1,0.5))
# plt.title('Scaled')

# plt.figure()
# plt.plot(t,s,'b-',linewidth=2)
# plt.grid('on')
# plt.plot(t_scaled,s_scaled,'r-',linewidth=2)
# plt.xlabel('Time [s]')
# plt.ylabel('Arc-length [m]')

plt.show()

# #Save data
# np.save('traj_df_data',data_scaled)
