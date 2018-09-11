from abr_control.utils import DataHandler, make_gif
import numpy as np
import matplotlib
matplotlib.use("TKAgg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import subprocess
import os

#TODO: turn into module
# remove old figures used for previous gif so we don't get overlap for tests of
# different lengths

# bashCommand = ("rm figures/gif_figs/*.png")
# print('Removing old figures from "gif_figs" folder...')
# process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)#,
# output, error = process.communicate()

show_traj = True
run_num=49
use_cache=True
db_name = 'dewolf2018neuromorphic'
test_groups = [
                #'simulations',
                #'1lb_random_target',
                'simulations',
                'simulations',
                # '1lb_random_target',
              ]
tests = [
        # 'ur5_sim_no_weight_6',
        # 'ur5_sim_no_weight_7',
        'jaco_sim_no_weight_6',
        'jaco_sim_no_weight_7',
        # 'pd_no_weight_47',
        # 'pd_no_weight_50',
        # 'jaco_sim_no_weight_5',
        ]
use_offset = False
# test_name = 'nengo_cpu_%i_19'%run_num
# test_name0 = 'nengo_cpu_%i_0'%run_num

# test_name = 'nengo_cpu_53_9'
# test_name0 = 'nengo_cpu_48_9'

# test_name = 'pd_no_weight_20'
# test_name0 = 'pd_no_weight_22'

label_name = '%s : %s'%(tests[0], tests[1])
session = 0
runs=[0,1,2,3,4,5,6,7,8,9]
if 'simulations' in test_groups[0]:
    if 'ur5' in tests[0]:
        from abr_control.arms import ur5 as arm
        rc = arm.Config(use_cython=True)
        print('Using UR5 sim config')
    elif 'jaco' in tests[0]:
        from abr_control.arms import jaco2 as arm
        rc = arm.Config(use_cython=True, hand_attached=True)
        print('Using jaco2 sim config')
    offset=[0,0,0]
else:
    import abr_jaco2
    # only the first test needs the transforms since the virtual arm is for
    # that test. The offset is based on the first test listed
    if use_offset:
        rc = abr_jaco2.ConfigOffset(use_cython=True, hand_attached=True, init_all=True,
                offset=None)
        print('using jaco2 config with offset')
    else:
        rc = abr_jaco2.Config(use_cython=True, hand_attached=True, init_all=True,
                offset=None)
        print('using jaco2 config without offset')
    offset = rc.OFFSET
    #offset=[0,0,0]
    rc.init_all()

q_t = []
error_t = []
ee_xyz_t = []
targets_t = []
filter_t = []
ee_xyz_0 = []
error_0 = []
time_t = []

if use_cache:
    from abr_control.utils.paths import cache_dir
    save_loc = '%s/figures'%cache_dir
else:
    save_loc = 'figures'

if not os.path.exists(save_loc):
    os.makedirs(save_loc)

if not os.path.exists('%s/gif_fig_cache'%save_loc):
    os.makedirs('%s/gif_fig_cache'%save_loc)

files = [f for f in os.listdir('%s/gif_fig_cache'%save_loc) if f.endswith(".png") ]
for ii, f in enumerate(files):
    if ii == 0:
        print('Deleting old temporary figures for gif creation...')
    os.remove(os.path.join('%s/gif_fig_cache'%save_loc, f))
    #print(os.path.join('%s/gif_fig_cache'%save_loc, f))

for run in runs:
    dat = DataHandler(use_cache=use_cache, db_name=db_name)
    data = dat.load(params=['q', 'error', 'target', 'ee_xyz', 'filter', 'time'],
            save_location='%s/%s/session%03d/run%03d'%
            (test_groups[0], tests[0], session, run))
    q_t.append(data['q'])
    error_t.append(data['error'])
    ee_xyz_t.append(data['ee_xyz'])
    targets_t.append(data['target'])
    filter_t.append(data['filter'])
    time_t.append(data['time'])

    data_0 = dat.load(params=['ee_xyz', 'error'],
            save_location='%s/%s/session%03d/run%03d'%
            (test_groups[1], tests[1], session, run))
    ee_xyz_0.append(data_0['ee_xyz'])
    error_0.append(data_0['error'])

q_t = np.array(q_t)
error_t = np.array(error_t)
ee_xyz_t = np.array(ee_xyz_t)
targets_t = np.array(targets_t)
filter_t = np.array(filter_t)

min_len = []
for nn, q in enumerate(q_t):
    min_len.append(len(q))
    #print('%i: %i'%(nn, len(q)))
length = np.min(min_len)


fig2 = plt.figure(figsize=(15, np.ceil(len(runs)/3)*5))
ee_xyz_1 = np.array(ee_xyz_t)
ee_xyz_2 = np.array(ee_xyz_0)
for run in runs:
    ax = fig2.add_subplot(np.ceil(len(runs)/3),3,run+1)
    plt.title(run)
    ax.plot(ee_xyz_1[run].T[0], 'r', label='%s X'%tests[0])
    ax.plot(ee_xyz_1[run].T[1], 'b', label='%s Y'%tests[0])
    ax.plot(ee_xyz_1[run].T[2], 'g', label='%s Z'%tests[0])
    ax.plot(ee_xyz_2[run].T[0], 'y--', label='%s X'%tests[1])
    ax.plot(ee_xyz_2[run].T[1], 'c--', label='%s Y'%tests[1])
    ax.plot(ee_xyz_2[run].T[2], 'm--', label='%s Z'%tests[1])
    plt.legend()
plt.savefig('2d_trajectory')
plt.show()

for ii in range(0,length,10):
    fig = plt.figure(figsize=(15, np.ceil(len(runs)/3)*5))
    # ax = [fig.add_subplot(1,3,1, projection='3d'),
    #       fig.add_subplot(1,3,2, projection='3d'),
    #       fig.add_subplot(1,3,3, projection='3d')]
    print('%.2f%% complete'%(ii/length*100), end='\r')
    for jj, run in enumerate(runs):
        ax = fig.add_subplot(np.ceil(len(runs)/3),3,run+1, projection='3d')
        q = q_t[run][ii]
        targets = targets_t[run][ii]
        error = error_t[run][ii]
        ee_xyz = ee_xyz_t[run][ii]
        filter_xyz = filter_t[run][ii, 0:3]

        joint0 = rc.Tx('joint0', q=q)#, x=offset)
        joint1 = rc.Tx('joint1', q=q)#, x=offset)
        joint2 = rc.Tx('joint2', q=q)#, x=offset)
        joint3 = rc.Tx('joint3', q=q)#, x=offset)
        joint4 = rc.Tx('joint4', q=q)#, x=offset)
        joint5 = rc.Tx('joint5', q=q)#, x=offset)
        ee_recalc = rc.Tx('EE', q=q, x=offset)


        link0 = np.array(rc.Tx('link0', q=q))#, x=offset)
        link1 = np.array(rc.Tx('link1', q=q))#, x=offset)
        link2 = np.array(rc.Tx('link2', q=q))#, x=offset)
        link3 = np.array(rc.Tx('link3', q=q))#, x=offset)
        link4 = np.array(rc.Tx('link4', q=q))#, x=offset)
        link5 = np.array(rc.Tx('link5', q=q))#, x=offset)
        link6 = np.array(rc.Tx('link6', q=q))#, x=offset)
        links = np.stack([link0, link1, link2, link3, link4, link5, link6])

        joint0 = np.array(joint0)
        joint1 = np.array(joint1)
        joint2 = np.array(joint2)
        joint3 = np.array(joint3)
        joint4 = np.array(joint4)
        joint5 = np.array(joint5)
        ee_recalc = np.array(ee_recalc)
        joints = [joint0, joint1, joint2, joint3, joint4, joint5, ee_xyz]
        joints = np.stack(joints)

        colors = ['k', 'k', 'k', 'k', 'k', 'k', 'k']
        marker_size = [2**5, 2**5, 2**5, 2**5, 2**5, 2**5, 2**5]
        marker = ['o', 'o', 'o', 'o', 'o', 'o', 'o']
        # if any joint drops below the origin, change its color to red
        for kk, j in enumerate(joints):
            # 0.04m == radius of elbow joint
            if j[2] < 0.04:
                colors[kk] = 'r'
                marker_size[kk] = 2**9
                marker[kk] = '*'
            else:
                colors[kk] = 'k'
                marker_size[kk] = 2**5
                marker[kk] = 'o'
        # plot target location
        ax.scatter(targets[0], targets[1], targets[2], c='r',marker='o', s=2**5)
        # plot COM locations
        ax.scatter(link0[0], link0[1], link0[2], c='y',
                marker='o', s=2**5)
        ax.scatter(link1[0], link1[1], link1[2], c='y',
                marker='o', s=2**5)
        ax.scatter(link2[0], link2[1], link2[2], c='y',
                marker='o', s=2**5)
        ax.scatter(link3[0], link3[1], link3[2], c='y',
                marker='o', s=2**5)
        ax.scatter(link4[0], link4[1], link4[2], c='y',
                marker='o', s=2**5)
        ax.scatter(link5[0], link5[1], link5[2], c='y',
                marker='o', s=2**5)
        ax.scatter(link6[0], link6[1], link6[2], c='y',
                marker='o', s=2**5)

        # plot joint locations
        ax.scatter(joint0[0], joint0[1], joint0[2], c=colors[0],
                marker=marker[0], s=marker_size[0])
        ax.scatter(joint1[0], joint1[1], joint1[2], c=colors[1],
                marker=marker[1], s=marker_size[1])
        ax.scatter(joint2[0], joint2[1], joint2[2], c=colors[2],
                marker=marker[2], s=marker_size[2])
        ax.scatter(joint3[0], joint3[1], joint3[2], c=colors[3],
                marker=marker[3], s=marker_size[3])
        ax.scatter(joint4[0], joint4[1], joint4[2], c=colors[4],
                marker=marker[4], s=marker_size[4])
        ax.scatter(joint5[0], joint5[1], joint5[2], c=colors[5],
                marker=marker[5], s=marker_size[5])
        ax.scatter(ee_xyz[0], ee_xyz[1], ee_xyz[2], c=colors[6],
                marker=marker[6], s=marker_size[6])
        # plot current filtered path planner target
        ax.scatter(filter_xyz[0], filter_xyz[1], filter_xyz[2], c='g', marker='*')
        # plot lines joining joints
        points = [None]*(len(joints)+len(links))
        points[::2] = links
        points[1::2] = joints
        points = np.array(points)
        ax.plot(points.T[0], points.T[1], points.T[2], 'k')
        ax.set_xlim3d(-0.35,0.35)
        ax.set_ylim3d(-0.35,0.35)
        ax.set_zlim3d(0,0.7)
        plt.title('Target %i: %s \n'%(jj, label_name))
        # time_t = np.array(time_t)
        # print(time_t.shape)
        # print(time_t[run].shape)
        # print(ii)
        # print(np.squeeze(time_t[run])[:ii+1])
        plt.xlabel('Time: %.2f sec'%(np.sum(time_t[run][:ii])))
        ax.text(-0.5, -0.5, 0.4, 'Avg: %.3f m'%np.mean(error_t[jj]), color='b')
        ax.text(-0.5, -0.5, 0.5, 'Final: %.3f m'%(error_t[jj][-1]), color='b')
        ax.text(-0.5, -0.5, 0.6, 'Error: %.3f m'%(error), color='b')
        ax.text(-0.5, -0.5, 0.7, tests[0], color='b')

        # plot the recalulated EE pos to see if it matches
        ax.scatter(ee_recalc[0], ee_recalc[1], ee_recalc[2], c='m', marker='*')

        if show_traj:
            # plot ee trajectory line
            ax.plot(ee_xyz_t[run][:ii, 0], ee_xyz_t[run][:ii,1], ee_xyz_t[run][:ii,2],
                    color='b', label=tests[0])
            # plot filtered path planner trajectory line
            ax.plot(filter_t[run][:ii, 0], filter_t[run][:ii,1], filter_t[run][:ii,2],
                    c='g')
            # plot ee trajectory line of starting run
            ax.plot(ee_xyz_0[run][:ii, 0], ee_xyz_0[run][:ii,1],
                    ee_xyz_0[run][:ii, 2], c='tab:purple', linestyle='dashed',
                    label=tests[1])
            ax.text(-0.5, -0.5, 0.0, 'Avg: %.3f m'%np.mean(error_0[jj]),
                    color='tab:purple')
            ax.text(-0.5, -0.5, 0.1, 'Final: %.3f m'%(error_0[jj][-1]),
                    color='tab:purple')
            if ii >= len(error_0[run]):
                iii = len(error_0[run])-1
            else:
                iii = ii
            ax.text(-0.5, -0.5, 0.2, 'Error: %.3f m'%(error_0[run][iii]),
                    color='tab:purple')
            ax.text(-0.5, -0.5, 0.3, tests[1], color='tab:purple')
            if jj == 0:
                ax.legend(bbox_to_anchor=[-0.55, 0.5], loc='center left')
    plt.savefig('%s/gif_fig_cache/%05d.png'%(save_loc,ii))
    plt.close()


make_gif.create(fig_loc='%s/gif_fig_cache'%save_loc,
                save_loc='%s/%s/virtual_arm'%(save_loc, db_name),
                save_name='%s-%s'%(tests[0], tests[1]),
                delay=5)

