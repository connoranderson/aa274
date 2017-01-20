import numpy as np
import math
import scipy as Sci
from scipy import linalg
import matplotlib.pyplot as plt

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
#...FILL...#

# Compute traj
dt = 0.005
N = int (t_f/dt)
t = dt*np.array(range(N+1)) # t[0],....,t[N]
t = t.T
data = np.zeros((N+1,9))

#Compute trajectory, store in data, format: [x,y,th,V,om,xd,yd,xdd,ydd]
#...FILL...#

# Re-scaling - Compute scaled trajectory, store in data_scaled
#...FILL...#

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
plt.plot(t,s,'b-',linewidth=2)
plt.grid('on')
plt.plot(t_scaled,s_scaled,'r-',linewidth=2)
plt.xlabel('Time [s]')
plt.ylabel('Arc-length [m]')

plt.show()

#Save data
np.save('traj_df_data',data_scaled)
