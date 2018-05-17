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
keys = ['dq']
saved_params = dat.load(params=keys,
        save_location=loc)
key = 'dq'
print('%s: %s' %(key, saved_params[key]))

# import matplotlib
# matplotlib.use("TKAgg")
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import axes3d
#
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
#
# targets = np.array(target_xyz)
# x = targets[:,0]
# y = targets[:,1]
# z = targets[:,2]
# ax.scatter(x,y,z,c='r',marker='o')
# plt.show()
