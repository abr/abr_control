from abr_control.utils import DataHandler
import os
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

def load_signals(test_group, test, session, run, dat):
    data = dat.load(params = ['u_base', 'q_torque', 'u_adapt'],
            save_location='%s/%s/session%03d/run%03d'%(test_group, test,
                session, run))
    osc = dat.load(params=['kp', 'kv', 'vmax'],
            save_location='%s/%s/parameters/OSC/'%(test_group, test))
    kp = osc['kp']
    kv = osc['kv']
    vmax = osc['vmax']
    gains = 'kp: %s\nkv: %s\nvmax: %s' %(kp, kv, vmax)
    u = data['u_base'].T
    T = data['q_torque'].T
    if 'nengo' in test:
        u_adapt = data['u_adapt'].T
        return [u_adapt, u, T, gains]
    else:
        return [u, T, gains]

def plot_data(test_group, tests, session, runs, use_cache=True, db_name=None):
    dat = DataHandler(use_cache=use_cache, db_name=db_name)
    plt.figure(figsize=(20,20))

    for jj, test in enumerate(tests):
        vlines = []
        for mm, run in enumerate(runs):#range(0, n_runs):
            if 'nengo' in test:
                [u_ad, uu, TT, gains] = load_signals(test_group=test_group,
                        test=test, session=session, run=run, dat=dat)
            else:
                [uu, TT, gains] = load_signals(test_group=test_group,
                        test=test, session=session, run=run, dat=dat)
            if mm == 0:
                u = uu
                T = TT
                vlines.append(len(uu.T))
                if 'nengo' in test:
                    u_adapt = u_ad

            else:
                u = np.hstack((u,uu))
                T = np.hstack((T,TT))
                vlines.append(len(u.T))
                if 'nengo' in test:
                    u_adapt = np.hstack((u_adapt, u_ad))

        N_JOINTS = len(u)
        for ii in range(0, N_JOINTS):
            plt.subplot(N_JOINTS,len(tests), 1 + len(tests)*ii + jj)
            if ii == 0:
                plt.title(test)
            if ii == N_JOINTS-1:
                plt.xlabel('%s'%gains)
            if jj == 0:
                plt.ylabel('J%i Torque'%ii)
            if 'nengo' in test:
                plt.plot(u[ii] + u_adapt[ii], label='OSC+adapt %s'%test)
                plt.plot(u[ii], label='OSC %s'%test)
                plt.plot(T[ii], label='Feedback %s'%test)
            else:
                plt.plot(u[ii], label='OSC %s'%test)
                plt.plot(T[ii], label='Feedback %s'%test)
            for line in vlines:
                plt.axvline(x=line, color='k')
            #plt.ylim(20, -20)
            plt.legend()

    if use_cache:
        from abr_control.utils.paths import cache_dir
        save_loc = '%s/figures/%s/torque_profile'%(cache_dir, db_name)
    else:
        save_loc = 'figures/%s/torque_profile'%db_name

    if not os.path.exists(save_loc):
        os.makedirs(save_loc)
    fig_title = 'torque_comparison-%s-vs%i'%(tests[0], len(tests)-1)
    plt.savefig('%s/%s.pdf'%(save_loc,fig_title))
    plt.show()
