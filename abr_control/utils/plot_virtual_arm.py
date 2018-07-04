import abr_jaco2
from abr_control.utils import DataHandler
import numpy as np
test_group = '1lb_random_target'
test_name = 'pd_2'
session = 0
run = 20
dat = DataHandler(use_cache=True, db_name='dewolf2018neuromorphic')
data = dat.load(params=['q', 'error', 'target', 'ee_xyz'],
        save_location='%s/%s/session%03d/run%03d'%
        (test_group, test_name, session, run))
q_t = data['q']
error_t = data['error']
ee_xyz_t = data['ee_xyz']
targets_t = data['target']

rc = abr_jaco2.Config(use_cython=True, hand_attached=True, init_all=True,
        offset=None)
offset = rc.OFFSET
rc.init_all()

print(len(q_t))
for ii in range(0,len(q_t),10):
    print('%.2f complete'%(ii/len(q_t)*100), end='\r')
    q = q_t[ii]
    targets = targets_t[ii]
    error = error_t[ii]
    ee_xyz = ee_xyz_t[ii]

    joint0 = rc.Tx('joint0', q=q, x=offset)
    joint1 = rc.Tx('joint1', q=q, x=offset)
    joint2 = rc.Tx('joint2', q=q, x=offset)
    joint3 = rc.Tx('joint3', q=q, x=offset)
    joint4 = rc.Tx('joint4', q=q, x=offset)
    joint5 = rc.Tx('joint5', q=q, x=offset)


    import matplotlib
    matplotlib.use("TKAgg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import axes3d

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

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
    ax.plot(joints[:,0], joints[:,1], joints[:,2])
    ax.set_xlim3d(-0.35,0.35)
    ax.set_ylim3d(-0.35,0.35)
    ax.set_zlim3d(0,0.7)
    plt.title('%s \nDistance to Target: %.3f m'%(test_name, error))
    plt.savefig('figures/gif_figs/%05d'%ii)
    if len(q_t)-ii > 10:
        plt.close()
    else:
        print('\n')
        print(joints)

bashCommand = ("convert -delay 5 -loop 0 -deconstruct -quantize"
               + " transparent -layers optimize -resize 1200x2000"
               + " figures/gif_figs/*.png figures/gif_figs/%s.gif"%test_group)
print('100.00% complete')
print('Creating gif...')
import subprocess
process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
output, error = process.communicate()
print('Finished')
plt.show()
