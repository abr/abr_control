import abr_jaco2
from abr_control.utils import DataHandler
import numpy as np
import matplotlib
matplotlib.use("TKAgg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d

show_traj = True
test_group = '1lb_random_target'
test_name = 'nengo_cpu_16_0'
session = 0
runs=[0,1,2,3,4,5,6,7,8,9]
rc = abr_jaco2.Config(use_cython=True, hand_attached=True, init_all=True,
        offset=None)
#offset = rc.OFFSET
offset=[0,0,0]
rc.init_all()

q_t = []
error_t = []
ee_xyz_t = []
targets_t = []
filter_t = []

for run in runs:
    dat = DataHandler(use_cache=True, db_name='dewolf2018neuromorphic')
    data = dat.load(params=['q', 'error', 'target', 'ee_xyz', 'filter'],
            save_location='%s/%s/session%03d/run%03d'%
            (test_group, test_name, session, run))
    q_t.append(data['q'])
    error_t.append(data['error'])
    ee_xyz_t.append(data['ee_xyz'])
    targets_t.append(data['target'])
    filter_t.append(data['filter'])

q_t = np.array(q_t)
error_t = np.array(error_t)
ee_xyz_t = np.array(ee_xyz_t)
targets_t = np.array(targets_t)
filter_t = np.array(filter_t)

length = min(min(len(q_t[0]), len(q_t[1])),len(q_t[2]))
print(length)
for ii in range(0,length,10):
    fig = plt.figure(figsize=(15, np.ceil(len(runs)/3)*5))
    # ax = [fig.add_subplot(1,3,1, projection='3d'),
    #       fig.add_subplot(1,3,2, projection='3d'),
    #       fig.add_subplot(1,3,3, projection='3d')]
    print('%.2f complete'%(ii/length*100), end='\r')
    for run in runs:
        ax = fig.add_subplot(np.ceil(len(runs)/3),3,run+1, projection='3d')
        q = q_t[run][ii]
        targets = targets_t[run][ii]
        error = error_t[run][ii]
        ee_xyz = ee_xyz_t[run][ii]
        filter_xyz = filter_t[run][ii, 0:3]

        joint0 = rc.Tx('joint0', q=q, x=offset)
        joint1 = rc.Tx('joint1', q=q, x=offset)
        joint2 = rc.Tx('joint2', q=q, x=offset)
        joint3 = rc.Tx('joint3', q=q, x=offset)
        joint4 = rc.Tx('joint4', q=q, x=offset)
        joint5 = rc.Tx('joint5', q=q, x=offset)



        joint0 = np.array(joint0)
        joint1 = np.array(joint1)
        joint2 = np.array(joint2)
        joint3 = np.array(joint3)
        joint4 = np.array(joint4)
        joint5 = np.array(joint5)
        joints = [joint0, joint1, joint2, joint3, joint4, joint5, ee_xyz]
        joints = np.stack(joints)

        ax.scatter(targets[0], targets[1], targets[2], c='r',marker='o')
        ax.scatter(joint0[0], joint0[1], joint0[2], c='b', marker='o')
        ax.scatter(joint1[0], joint1[1], joint1[2], c='b', marker='o')
        ax.scatter(joint2[0], joint2[1], joint2[2], c='b', marker='o')
        ax.scatter(joint3[0], joint3[1], joint3[2], c='b', marker='o')
        ax.scatter(joint4[0], joint4[1], joint4[2], c='b', marker='o')
        ax.scatter(joint5[0], joint5[1], joint5[2], c='b', marker='o')
        ax.scatter(ee_xyz[0], ee_xyz[1], ee_xyz[2], c='b', marker='o')
        ax.scatter(filter_xyz[0], filter_xyz[1], filter_xyz[2], c='g', marker='*')
        ax.plot(joints[:,0], joints[:,1], joints[:,2])
        ax.set_xlim3d(-0.35,0.35)
        ax.set_ylim3d(-0.35,0.35)
        ax.set_zlim3d(0,0.7)
        plt.title('%s \nDistance to Target %i: %.3f m'%(test_name, run, error))

        if show_traj:
            ax.plot(ee_xyz_t[0][:ii, 0], ee_xyz_t[0][:ii,1], ee_xyz_t[0][:ii,2],
                    color='b')
            ax.plot(filter_t[0][:ii, 0], filter_t[0][:ii,1], filter_t[0][:ii,2],
                    c='g')
    plt.savefig('figures/gif_figs/%05d.png'%ii)
    plt.close()

bashCommand = ("convert -delay 5 -loop 0 -deconstruct -quantize"
               + " transparent -layers optimize -resize 1200x2000"
               + " figures/gif_figs/*.png figures/gif_figs/gifs/%s-%s.gif"
               %(test_group,test_name))
print('100.00% complete')
print('Creating gif...')
import subprocess
process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
output, error = process.communicate()
print('Finished')
#plt.show()
