import numpy as np
import matplotlib.pyplot as plt
import os 

os.environ['DISPLAY'] = ':23'


trajectory_pth = './dataset/seq_1/sequence.csv'
data = np.loadtxt(trajectory_pth, delimiter = ',', skiprows=1)
headers = {'idx':0, 't':1, 'x':2, 'y':3, 'z':4, 'qx':5, 'qy':6, 'qz':7, 'qw':8}
print(data.shape)

plt.figure()
plt.plot(data[:, headers['x']], data[:, headers['y']])
plt.title('Trajectory')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.minorticks_on()
plt.grid(True, which='both', linestyle='--', linewidth=0.5)
plt.show()

plt.figure()
plt.plot(data[:, headers['t']], data[:, headers['z']])
plt.title('Depth')
plt.xlabel('t [s]')
plt.ylabel('z [m]')
plt.minorticks_on()
plt.grid(True, which='both', linestyle='--', linewidth=0.5)
plt.show()