import numpy as np
import traceback
import abr_jaco2
from abr_control.controllers import OSC, path_planners
import nengo
from abr_control.utils import DataHandler, Target

dat = DataHandler(use_cache=True)

print('-----MAIN TIER GROUPS-----')
loc = 'testing/joint_space_training/'
keys = dat.get_keys(loc)
saved_params = dat.load(params=keys,
        save_location=loc)

for key in saved_params:
    print('%s: %s' %(key, saved_params[key]))

print('\n')
print('-----SESSION 0 DATA-----')
loc = 'testing/joint_space_training/session0/'
keys = dat.get_keys(loc)
saved_params = dat.load(params=keys,
        save_location=loc)

for key in saved_params:
    print('%s: %s' %(key, saved_params[key]))

print('\n')
print('-----SESSION 0 RUN0 dq-----')
loc = 'testing/joint_space_training/session0/run0/'
keys = dat.get_keys(loc)
#keys = ['dq']
data = dat.load(params=keys,
        save_location=loc)
#key = 'dq'
#print('%s: %s' %(key, saved_params[key]))

import matplotlib
matplotlib.use("TKAgg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d

# control signals and inputs
fig = plt.figure()
ax1 = fig.add_subplot(2,3,1)
ax1.set_title('input signal')
ax1.plot(data['input_signal'])

ax2 = fig.add_subplot(2,3,2)
ax2.set_title('training_signal')
ax2.plot(data['training_signal'])

ax3 = fig.add_subplot(2,3,3)
ax3.set_title('u adapt')
ax3.plot(data['u_adapt'])

ax4 = fig.add_subplot(2,3,4)
ax4.set_title('u osc')
ax4.plot(data['u_base'])

ax5 = fig.add_subplot(2,3,5)
ax5.set_title('u avoid')
ax5.plot(data['u_avoid'])

plt.tight_layout()
plt.show()

# feedback
fig = plt.figure()
ax1 = fig.add_subplot(2,3,1)
ax1.set_title('q')
ax1.plot(data['q'])

ax2 = fig.add_subplot(2,3,2)
ax2.set_title('dq')
ax2.plot(data['dq'])

ax3 = fig.add_subplot(2,3,3)
ax3.set_title('error')
ax3.plot(data['error'])

ax4 = fig.add_subplot(2,3,4)
ax4.set_title('time')
ax4.plot(data['time'])

ax5 = fig.add_subplot(2,3,5)
ax5.set_title('filter')
ax5.plot(data['filter'])

plt.tight_layout()
plt.show()

# weights
weights = np.squeeze(np.array(data['weights']))
fig = plt.figure()
ax1 = fig.add_subplot(121)
ax1.set_title('weights')
ax1.plot(weights)

# target and traj
ax2 = fig.add_subplot(122, projection='3d')

targets = data['target']
x = targets[:,0]
y = targets[:,1]
z = targets[:,2]
ax2.scatter(x,y,z,c='r',marker='o')

traj = data['ee_xyz']
tx = traj[:,0]
ty = traj[:,1]
tz = traj[:,2]
ax2.scatter(tx,ty,tz,c='k')
plt.tight_layout()
plt.show()
