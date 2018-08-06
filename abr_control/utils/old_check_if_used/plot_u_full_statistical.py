from abr_control.utils import DataHandler, ProcessData
import matplotlib
matplotlib.use('TkAgg')
from matplotlib import pyplot as plt
import numpy as np

dat = DataHandler(use_cache=True, db_name='dewolf2018neuromorphic')
proc = ProcessData()

test_group = 'friction_tests'
#test_group = 'gradual_friction_tests'
#test_group = 'weighted_tests'

if test_group == 'gradual_friction_tests':
    test_list = [
                  # 'pd_no_friction',
                  # 'pd',
                  # 'pid',
                  'nengo_cpu20k',
                  'nengo_gpu20k',
                  'nengo_loihi20k'
                  ]

elif test_group == 'friction_tests':
    test_list = [
                  # 'pd_no_friction',
                  # 'pd',
                  # 'pid',
                  'nengo_cpu1k',
                  'nengo_gpu1k',
                  'nengo_loihi1k'
                  ]

elif test_group == 'weighted_tests':
    test_list = [
                  # 'pd_no_weight',
                  # 'pd',
                  # 'pid',
                  'nengo_cpu1k',
                  'nengo_gpu1k',
                  'nengo_loihi1k'
                  ]

for test in test_list:
    print('TEST: ', test)
    ee_xyz0 = []
    ee_xyz49 = []
    n_sessions = 5
    # load raw data
    for ii in range(0,n_sessions):
        dat0 = dat.load(params=['adapt', 'time'],
                save_location='%s/%s/session00%i/run000'%
                (test_group, test, ii))
        dat49 = dat.load(params=['adapt', 'time'],
                save_location='%s/%s/session00%i/run049'%
                (test_group, test, ii))
        ee0 = dat0['adapt'][:, 1:3]
        ee49 = dat49['adapt'][:,1:3]
        print(np.array(ee0).shape)
        t0 = dat0['time']
        t49 = dat49['time']
        # interpolate data
        ee0 = proc.interpolate_data(data=ee0, time_intervals=t0, n_points=400)
        ee49 = proc.interpolate_data(data=ee49, time_intervals=t49, n_points=400)
        ee_xyz0.append(ee0)
        ee_xyz49.append(ee49)


    ee_xyz0 = np.array(ee_xyz0)
    ee_xyz49 = np.array(ee_xyz49)



    print('eexyz0 loaded: ', ee_xyz0.shape)
    ee_xyz0_2norm = np.zeros((n_sessions, len(ee_xyz0[0])))
    ee_xyz49_2norm = np.zeros((n_sessions, len(ee_xyz49[0])))
    print('eexyz0_2norm_ init: ', ee_xyz0_2norm.shape)

    # take the norm of the trajectory
    # for ii in range(0, n_sessions):
    #     # ee_xyz0_2norm[ii] = np.sqrt(np.sum(np.squeeze(ee_xyz0[ii,:,:])**2, axis=1))
        # ee_xyz49_2norm[ii] = np.sqrt(np.sum(np.squeeze(ee_xyz49[ii,:,:])**2, axis=1))
    ee_xyz0_2norm = np.linalg.norm(ee_xyz0, axis=2)
    ee_xyz49_2norm = np.linalg.norm(ee_xyz49, axis=2)

    print('eexyz0_2norm filled: ', ee_xyz0_2norm.shape)
    # # take the average between sessions
    # ee_xyz0_mean = np.mean(ee_xyz0_2norm, axis=0)
    # ee_xyz49_mean = np.mean(ee_xyz49_2norm, axis=0)
    # print('ee_xyz0_mean: ', ee_xyz0_mean.shape)
    n_targets = 5

    # break up between targets
    samples_per_target = int(len(ee_xyz0_2norm[0])/n_targets)
    ee_xyz0_split = np.zeros((n_targets*n_sessions,samples_per_target))
    ee_xyz49_split = np.zeros((n_targets*n_sessions,samples_per_target))
    print('eexyz0 split init: ', ee_xyz0_split.shape)

    # split up into the trajectories to each target
    for mm in range(0, n_sessions):
        for jj in range(0, n_targets):
            ee_xyz0_split[mm*n_sessions+jj] = (ee_xyz0_2norm[mm,
                    samples_per_target*jj:samples_per_target*(jj+1)])
            ee_xyz49_split[mm*n_sessions+jj] = (ee_xyz49_2norm[mm,
                    samples_per_target*jj:samples_per_target*(jj+1)])

    print('eexyz0_split: ', ee_xyz0_split.shape)

    data0 = proc.get_mean_and_ci(raw_data=ee_xyz0_split,
            n_runs=samples_per_target)
    data49 = proc.get_mean_and_ci(raw_data=ee_xyz49_split,
            n_runs=samples_per_target)

    #data = {'mean': data[0], 'lower_bound': data[1], 'upper_bound': data[2]}
    plt.figure()
    color = ['r', 'b']
    color_shading = color
    color_mean = color
    plt.fill_between(range(np.array(data0[0]).shape[0]),
                     data0[2],
                     data0[1],
                     color=color_shading[0],
                     alpha=.5)
    plt.plot(data0[0], color=color_mean[0], label='run0')
    plt.fill_between(range(np.array(data49[0]).shape[0]),
                     data49[2],
                     data49[1],
                     color=color_shading[1],
                     alpha=.5)
    plt.plot(data49[0], color=color_mean[1], label='run49')
    plt.ylabel('2norm u full')
    plt.xlabel('run')
    plt.title('%s: %s'%(test_group, test))
    #plt.show()
    plt.legend()
    plt.savefig('figures/%s_2norm-u-adapt-profile_%s.pdf'%(test_group, test))

    #ee_xyz0 = ee_xyz0[0].T
    #ee_xyz49 = ee_xyz49[0].T
    #print(ee_xyz0.shape)
    #plt.figure()
    #plt.subplot(411)
    #plt.title(test)
    #plt.plot(ee_xyz0[0], 'k', label='run0')
    #plt.ylabel('X')
    #plt.plot(ee_xyz49[0], 'y', label='run49')
    #plt.legend()
    #plt.subplot(412)
    #plt.plot(ee_xyz0[1], 'k', label='run0')
    #plt.ylabel('Y')
    #plt.plot(ee_xyz49[1], 'y', label='run49')
    #plt.legend()
    #plt.subplot(413)
    #plt.plot(ee_xyz0[2], 'k', label='run0')
    #plt.ylabel('Z')
    #plt.plot(ee_xyz49[2], 'y', label='run49')
    #plt.legend()
    #plt.subplot(414)
    #plt.plot(np.sqrt(np.sum(ee_xyz0**2, axis=0)), label='2norm')
    #plt.legend
    #plt.tight_layout()
    #plt.show()
    ##plt.savefig('figures/%s_vel_profile_%s.pdf'%(test_group, test))
