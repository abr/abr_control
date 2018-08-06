from abr_control.utils import DataHandler
import matplotlib
matplotlib.use('TkAgg')
from matplotlib import pyplot as plt
import numpy as np

dat = DataHandler(use_cache=True, db_name='dewolf2018neuromorphic')

# test_group = 'gradual_friction_tests'
test_group = '1lb_random_target'
test_list = [
             'nengo_cpu_40_9'
            ]
# test_list = [
#               'pd_no_friction',
#               'pd',
#               'pid',
#               'nengo_cpu20k',
#               'nengo_gpu20k',
#               'nengo_loihi20k'
#               ]

# test_group = 'friction_tests'
# test_list = [
#               'pd_no_friction',
#               'pd',
#               'pid',
#               'nengo_cpu1k',
#               'nengo_gpu1k',
#               'nengo_loihi1k'
#               ]

# test_group = 'weighted_tests'
# test_list = [
#               'pd_no_weight',
#               'pd',
#               'pid',
#               'nengo_cpu1k',
#               'nengo_gpu1k',
#               'nengo_loihi1k'
#               ]
for test in test_list:
    # load data
    data = dat.load(params=['ee_xyz', 'time'], save_location='%s/%s/session000/run000'%
            (test_group, test))
    # ee_xyz49 = dat.load(params=['ee_xyz'], save_location='%s/%s/session000/run049'%
    #         (test_group, test))['ee_xyz']
    ee_xyz0 = data['ee_xyz']
    time = data['time']
    time = np.squeeze(time)
    time = time[:-1]
    time = np.vstack(((np.vstack((time, time)), time))).T

    ee_xyz0 = (np.diff(ee_xyz0, axis=0)/time).T
    # ee_xyz49 = (np.diff(ee_xyz49, axis=0)/0.0075).T
    #
    plt.figure()
    plt.subplot(411)
    plt.title(test)
    plt.plot(ee_xyz0[0], 'k', label='run0')
    plt.ylabel('X')
    # plt.plot(ee_xyz49[0], 'y', label='run49')
    # plt.legend()
    plt.subplot(412)
    plt.plot(ee_xyz0[1], 'k', label='run0')
    plt.ylabel('Y')
    # plt.plot(ee_xyz49[1], 'y', label='run49')
    # plt.legend()
    plt.subplot(413)
    plt.plot(ee_xyz0[2], 'k', label='run0')
    plt.ylabel('Z')
    # plt.plot(ee_xyz49[2], 'y', label='run49')
    # plt.legend()
    plt.subplot(414)
    plt.plot(np.sqrt(np.sum(ee_xyz0**2, axis=0)), label='2norm')
    plt.ylabel('2norm')
    plt.legend
    plt.tight_layout()
    #plt.show()
    plt.savefig('figures/%s_vel_profile_%s.pdf'%(test_group, test))
