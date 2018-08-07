import abr_jaco2
from abr_control.utils import DataHandler, ProcessData
import os
import numpy as np

import matplotlib
matplotlib.use("TKAgg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d

def plot_data(test_group, test_name, session, run, use_cache=True, db_name=None):
    dat = DataHandler(use_cache=use_cache, db_name=db_name)
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
    reaching_time = dat.load(params=['reaching_time'],
        save_location='%s/%s/parameters/test_parameters/'
        %(test_group, test_name))['reaching_time']

    dt = np.mean(time)
    # calculate ee velocity by differentiating ee position
    ee_dxyz = np.vstack([np.zeros((1, 3)),
        np.diff(ee_xyz_t, axis=0) / np.array(time[1:, None])])
    proc = ProcessData()
    ideal_t,ideal = proc.generate_ideal_path(reaching_time=reaching_time,
            target_xyz=[np.array(targets_t[0])],
            start_xyz=ee_xyz_t[0], kp=kp, kv=kv, vmax=vmax, dt=dt)

    #interpolate
    ideal = proc.interpolate_data(data=ideal, time_intervals=ideal_t, n_points=200)
    filter_t = proc.interpolate_data(data=filter_t, time_intervals=time, n_points=200)
    dx = proc.interpolate_data(data=dx, time_intervals=time, n_points=200)
    ee_dxyz = proc.interpolate_data(data=ee_dxyz, time_intervals=time, n_points=200)


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
    if use_cache:
        from abr_control.utils.paths import cache_dir
        save_loc = '%s/figures/%s/velocity_profile'%(cache_dir, db_name)
    else:
        save_loc = 'figures/%s/velocity_profile'%db_name

    if not os.path.exists(save_loc):
        os.makedirs(save_loc)
    fig_title = 'velocity_comparison-%s'%(test_name)
    plt.savefig('%s/%s.pdf'%(save_loc,fig_title))

    plt.show()
