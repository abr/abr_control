import abr_jaco2
from abr_control.utils import DataHandler
import numpy as np
import matplotlib
matplotlib.use("TKAgg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import subprocess

# remove old figures used for previous gif so we don't get overlap for tests of
# different lengths

# bashCommand = ("rm figures/gif_figs/*.png")
# print('Removing old figures from "gif_figs" folder...')
# process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)#,
# output, error = process.communicate()

show_traj = True
run_num=49
test_group = '1lb_random_target'
# test_name = 'nengo_cpu_%i_19'%run_num
# test_name0 = 'nengo_cpu_%i_0'%run_num

test_name = 'nengo_cpu_53_9'
test_name0 = 'nengo_cpu_48_9'

# test_name = 'pd_no_weight_12'
# test_name0 = 'pd_no_weight_9'

label_name = '%s : %s'%(test_name, test_name0)
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
ee_xyz_0 = []
error_0 = []
time_t = []

for run in runs:
    dat = DataHandler(use_cache=True, db_name='dewolf2018neuromorphic')
    data = dat.load(params=['q', 'error', 'target', 'ee_xyz', 'filter', 'time'],
            save_location='%s/%s/session%03d/run%03d'%
            (test_group, test_name, session, run))
    q_t.append(data['q'])
    error_t.append(data['error'])
    ee_xyz_t.append(data['ee_xyz'])
    targets_t.append(data['target'])
    filter_t.append(data['filter'])
    time_t.append(data['time'])

    data_0 = dat.load(params=['ee_xyz', 'error'],
            save_location='%s/%s/session%03d/run%03d'%
            (test_group, test_name0, session, run))
    ee_xyz_0.append(data_0['ee_xyz'])
    error_0.append(data_0['error'])

q_t = np.array(q_t)
error_t = np.array(error_t)
ee_xyz_t = np.array(ee_xyz_t)
targets_t = np.array(targets_t)
filter_t = np.array(filter_t)

min_len = []
for q in q_t:
    min_len.append(len(q))
length = np.min(min_len)

for ii in range(0,length,10):
    fig = plt.figure(figsize=(15, np.ceil(len(runs)/3)*5))
    # ax = [fig.add_subplot(1,3,1, projection='3d'),
    #       fig.add_subplot(1,3,2, projection='3d'),
    #       fig.add_subplot(1,3,3, projection='3d')]
    print('%.2f complete'%(ii/length*100), end='\r')
    for jj, run in enumerate(runs):
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
        ax.scatter(targets[0], targets[1], targets[2], c='r',marker='o')
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
        ax.plot(joints[:,0], joints[:,1], joints[:,2], 'k')
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
        ax.text(-0.5, -0.5, 0.7, test_name, color='b')

        if show_traj:
            # plot ee trajectory line
            ax.plot(ee_xyz_t[run][:ii, 0], ee_xyz_t[run][:ii,1], ee_xyz_t[run][:ii,2],
                    color='b', label=test_name)
            # plot filtered path planner trajectory line
            ax.plot(filter_t[run][:ii, 0], filter_t[run][:ii,1], filter_t[run][:ii,2],
                    c='g')
            # plot ee trajectory line of starting run
            ax.plot(ee_xyz_0[run][:ii, 0], ee_xyz_0[run][:ii,1],
                    ee_xyz_0[run][:ii, 2], c='tab:purple', linestyle='dashed',
                    label=test_name0)
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
            ax.text(-0.5, -0.5, 0.3, test_name0, color='tab:purple')
            if jj == 0:
                ax.legend(bbox_to_anchor=[-0.55, 0.5], loc='center left')
    plt.savefig('figures/gif_figs/%05d.png'%ii)
    plt.close()


#"convert -delay 5 -loop 0 -deconstruct -quantize transparent -layers optimize -resize 1200x2000 figures/gif_figs/*.png figures/gif_figs/gifs/%s-%s.gif"
bashCommand = ("convert -delay 5 -loop 0 -deconstruct -quantize"
               + " transparent -layers optimize -resize 1200x2000"
               + " figures/gif_figs/*.png figures/gif_figs/gifs/"
               + "%s-%s-%s.gif"
               %(test_group,test_name, test_name0))
print('command: ', bashCommand)
print('100.00% complete')
print('Creating gif...')
process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)#,
        #stdin=subprocess.PIPE, shell=True)
output, error = process.communicate()
print('Finished')
