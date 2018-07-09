import abr_jaco2
from abr_control.utils import DataHandler, ProcessData
import numpy as np

import matplotlib
matplotlib.use("TKAgg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
test_group = 'base_control_tuning'
test_name = 'pd34'
session = 0
run = 1
dt = 0.004
dat = DataHandler(use_cache=True, db_name='dewolf2018neuromorphic')
data = dat.load(params=['q', 'error', 'target', 'ee_xyz', 'filter', 'osc_dx',
    'time'],
        save_location='%s/%s/session%03d/run%03d'%
        (test_group, test_name, session, run))
osc = dat.load(params=['kp', 'kv', 'vmax'],
        save_location='%s/%s/parameters/OSC/'%(test_group, test_name))
kp = osc['kp']
kv = osc['kv']
vmax = osc['vmax']
q_t = data['q']
error_t = data['error']
ee_xyz_t = data['ee_xyz']
targets_t = data['target']
filter_t = data['filter']
dx = data['osc_dx']
time = data['time']

dt = np.mean(time)
ee_dxyz = np.vstack([np.zeros((1, 3)),
    np.diff(ee_xyz_t, axis=0) / np.array(time[1:, None])])
# print(targets_t[0])
# print([np.array(targets_t[0])])
# print(ee_xyz_t[0])
proc = ProcessData()
ideal_t,ideal = proc.generate_ideal_path(reaching_time=4,
        target_xyz=[np.array(targets_t[0])],
        start_xyz=ee_xyz_t[0], kp=kp, kv=kv, vmax=vmax, dt=dt)

#interpolate
ideal = proc.interpolate_data(data=ideal, time_intervals=ideal_t, n_points=200)
filter_t = proc.interpolate_data(data=filter_t, time_intervals=time, n_points=200)
dx = proc.interpolate_data(data=dx, time_intervals=time, n_points=200)
ee_dxyz = proc.interpolate_data(data=ee_dxyz, time_intervals=time, n_points=200)
# q_t = proc.interpolate(data=q_t, time_intervals=time, n_point=200)
# error_t = proc.interpolate(data=error_t, time_intervals=time, n_points=200)
# ee_xyz_t = proc.interpolate(data=ee_xyz_t, time_intervals=time, n_points=200)


plt.figure()
label = ['X','Y','Z']
for ii in range(0,3):
    plt.subplot(3,1,ii+1)
    if ii == 0:
        plt.title('Velocity Profiles: %s\nkp:%i kv:%i vmax:%i'
                %(test_name, kp, kv, vmax))
    plt.plot(dx[:,ii], 'b',label='dx')
    plt.plot(filter_t[:, 3+ii],'g', label='filter')
    plt.plot(filter_t[:, 3+ii]-dx[:,ii],'k', label='filter-dx')
    plt.plot(ideal[:, 3+ii], '--r', label='ideal')
    plt.plot(ee_dxyz[:,ii], 'orange', label='ee_vel')
    plt.legend(loc=1)
    plt.ylabel(label[ii])
plt.tight_layout()
plt.show()

# rc = abr_jaco2.Config(use_cython=True, hand_attached=True, init_all=True,
#         offset=None)
# offset = rc.OFFSET
# rc.init_all()
#
# print(len(q_t))
# for ii in range(0,len(q_t),10):
#     print('%.2f complete'%(ii/len(q_t)*100), end='\r')
#     q = q_t[ii]
#     targets = targets_t[ii]
#     error = error_t[ii]
#     ee_xyz = ee_xyz_t[ii]
#     filter_xyz = filter_t[ii, 0:3]
#
#     joint0 = rc.Tx('joint0', q=q, x=offset)
#     joint1 = rc.Tx('joint1', q=q, x=offset)
#     joint2 = rc.Tx('joint2', q=q, x=offset)
#     joint3 = rc.Tx('joint3', q=q, x=offset)
#     joint4 = rc.Tx('joint4', q=q, x=offset)
#     joint5 = rc.Tx('joint5', q=q, x=offset)
#
#
#
#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection='3d')
#
#     joint0 = np.array(joint0)
#     joint1 = np.array(joint1)
#     joint2 = np.array(joint2)
#     joint3 = np.array(joint3)
#     joint4 = np.array(joint4)
#     joint5 = np.array(joint5)
#     joints = [joint0, joint1, joint2, joint3, joint4, joint5, ee_xyz]
#     joints = np.stack(joints)
#
#     ax.scatter(targets[0], targets[1], targets[2], c='r',marker='o')
#     ax.scatter(joint0[0], joint0[1], joint0[2], c='b', marker='o')
#     ax.scatter(joint1[0], joint1[1], joint1[2], c='b', marker='o')
#     ax.scatter(joint2[0], joint2[1], joint2[2], c='b', marker='o')
#     ax.scatter(joint3[0], joint3[1], joint3[2], c='b', marker='o')
#     ax.scatter(joint4[0], joint4[1], joint4[2], c='b', marker='o')
#     ax.scatter(joint5[0], joint5[1], joint5[2], c='b', marker='o')
#     ax.scatter(ee_xyz[0], ee_xyz[1], ee_xyz[2], c='b', marker='o')
#     ax.scatter(filter_xyz[0], filter_xyz[1], filter_xyz[2], c='g', marker='o')
#     ax.plot(joints[:,0], joints[:,1], joints[:,2])
#     ax.set_xlim3d(-0.35,0.35)
#     ax.set_ylim3d(-0.35,0.35)
#     ax.set_zlim3d(0,0.7)
#     plt.title('%s \nDistance to Target: %.3f m'%(test_name, error))
#     plt.savefig('figures/gif_figs/%05d'%ii)
#     if len(q_t)-ii > 10:
#         plt.close()
#     else:
#         print('\n')
#         print(joints)
#
# bashCommand = ("convert -delay 5 -loop 0 -deconstruct -quantize"
#                + " transparent -layers optimize -resize 1200x2000"
#                + " figures/gif_figs/*.png figures/gif_figs/%s.gif"%test_group)
# print('100.00% complete')
# print('Creating gif...')
# import subprocess
# process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
# output, error = process.communicate()
# print('Finished')
# plt.show()
