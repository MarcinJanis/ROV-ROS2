import numpy as np
import matplotlib.pyplot as plt
import os 

os.environ['DISPLAY'] = ':23'

show_samples = 1.0

trajectory_pth = './dataset/seq_3/sequence.csv'
data = np.loadtxt(trajectory_pth, delimiter = ',', skiprows=1)
samples_num = data.shape[0]


print(f'imported {data.shape[0]} samples with {data.shape[1]} categories.')
headers = {'idx':0, 't':1, 'x':2, 'y':3, 'z':4, 'qx':5, 'qy':6, 'qz':7, 'qw':8}


plt.figure()
plt.plot(data[:int(show_samples*samples_num), headers['x']], data[:int(show_samples*samples_num), headers['y']])
plt.title('Trajectory')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.minorticks_on()
plt.grid(True, which='both', linestyle='--', linewidth=0.5)
plt.show()

plt.figure()
plt.plot(data[:int(show_samples*samples_num), headers['t']], data[:int(show_samples*samples_num), headers['z']])
plt.title('Depth')
plt.xlabel('t [s]')
plt.ylabel('z [m]')
plt.minorticks_on()
plt.grid(True, which='both', linestyle='--', linewidth=0.5)
plt.show()