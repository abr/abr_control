""" This script is calculating the error by comparing the trajectory of the
end-effector to the trajectory the hand could take under perfect OSC control.
The perfect trajectory is stored in the pd_x.npz file, with keys x, dx. """

import os
import numpy as np
from abr_control.utils.paths import cache_dir

class ProcessData():
    def __init__(self):
        pass

    def bootstrapci(self, data, func, n=3000, p=0.95):
        index=int(n*(1-p)/2)
        samples = np.random.choice(data, size=(n, len(data)))
        r = [func(s) for s in samples]
        r.sort()
        return r[index], r[-index]

    def proc_saved_data(self, folder, name, n_sessions, n_runs, run_time,
            parameter, dt=0.005, regenerate=False):

        # account for imperfect test lengths
        run_time -= 0.03
        save_file = ('%s_%i_x_%iruns_dt%0.4f_%itargets_x_%isec_%s' %
                     (parameter, n_sessions, n_runs, dt, self.n_targets,
                         self.reach_time_per_target, name))
        print('searching for: ', save_file)
        saved_data = [sf for sf in os.listdir(os.getcwd())
                      if sf == (save_file + '.npz')]
        if len(saved_data) > 0 and not regenerate:
            print('Loading data from ', saved_data[0])
        else:
            raw_data = np.zeros((n_sessions, n_runs))
            param_sessions = []
            for ii in range(n_sessions):
                param_runs = []
                for jj in range(n_runs):
                    filename_param = (
                        folder + 'session%i/run%i_data/%s%i.npz' % (ii, jj,
                            parameter, jj))
                    filename_timeintervals = (
                        folder + 'session%i/run%i_data/time%i.npz' % (ii, jj, jj))
                    param = np.load(filename_param)['%s'%parameter].squeeze()
                    time_intervals = np.cumsum(np.hstack([
                        0, np.load(filename_timeintervals)['time'].squeeze()[:-1]]))
                    print(ii, ' ', jj, ' ', time_intervals[-1], ': Using first'
                          + ' %f sec'%run_time)

                    # interpolate to have even sampling between tests
                    import scipy.interpolate
                    param_entries = []
                    for kk in range(param.shape[1]):
                        interp = scipy.interpolate.interp1d(time_intervals,
                                param[:, kk])
                        param_entries.append(np.array([
                            interp(t) for t in np.arange(0.001, run_time, dt)]))
                    param_runs.append(np.copy(param_entries))
                param_sessions.append(np.copy(param_runs))

            # Save interpolated data
            # add dimension if only 1 session so each saved file will be
            # 4 dimensional
            # if n_sessions == 1:
            #     np.savez_compressed(save_file, parameter=[param_sessions])
            # else:
            np.savez_compressed(save_file, parameter=param_sessions)
            print('PARAMETER SAVED: ', parameter)


    def proc_traj_err(self, folder, name, n_sessions, n_runs, run_time=3, dt=0.005, regenerate=False):

        # account for imperfect test lengths
        run_time-=0.03
        save_file = ('trajectory_error_%i_x_%i_runs_dt%0.4f_%itargets_x_%isec_%s' %
                     (n_sessions, n_runs, dt, self.n_targets,
                         self.reach_time_per_target, name))
        print('searching for: ', save_file)
        saved_data = [sf for sf in os.listdir(os.getcwd())
                      if sf == (save_file + '.npz')]
        if len(saved_data) > 0 and not regenerate:
            print('Loading data from ', saved_data[0])
            saved_data = np.load(saved_data[0])
            mean = saved_data['mean']
            upper_bound = saved_data['upper_bound']
            lower_bound = saved_data['lower_bound']
        else:

            # load in perfect trajectory, sampled every 1ms for 4s
            pd_xyz = np.load('../pd_x_%isec_x_%itargets.npz'
                    % (self.reach_time_per_target, self.n_targets))
            pd_xyz_x = pd_xyz['x']
            pd_xyz_dx = pd_xyz['dx']

            raw_data = np.zeros((n_sessions, n_runs))
            for ii in range(n_sessions):
                for jj in range(n_runs):
                    filename_ee_xyz = (
                        folder + 'session%i/run%i_data/ee_xyz%i.npz' % (ii, jj, jj))
                    filename_timeintervals = (
                        folder + 'session%i/run%i_data/time%i.npz' % (ii, jj, jj))
                    ee_xyz = np.load(filename_ee_xyz)['ee_xyz'].squeeze()
                    time_intervals = np.cumsum(np.hstack([
                        0, np.load(filename_timeintervals)['time'].squeeze()[:-1]]))
                    print('Session %i run %i ran for %f seconds ' %
                          (ii, jj, time_intervals[-1]))

                    # interpolate to even samples out
                    ee_xyz_interp = []
                    import scipy.interpolate
                    for kk in range(ee_xyz.shape[1]):
                        interp = scipy.interpolate.interp1d(time_intervals, ee_xyz[:, kk])
                        ee_xyz_interp.append(np.array([
                            interp(t) for t in np.arange(0.001, run_time, dt)]))
                    ee_xyz_x = np.array(ee_xyz_interp).T
                    # calculate the velocity of the end-effector
                    ee_xyz_dx = np.vstack([np.zeros((1, 3)), np.diff(ee_xyz_x, axis=0) / dt])

                    # compare perfect and actual trajectories to generate error
                    error = (
                        # position error
                        np.sqrt(np.sum((pd_xyz_x[:len(ee_xyz_x),:] - ee_xyz_x)**2, axis=1)) +
                        # velocity error
                        np.sqrt(np.sum((pd_xyz_dx[:len(ee_xyz_dx),:] - ee_xyz_dx)**2, axis=1)))
                    raw_data[ii, jj] = np.copy(np.sum(error) * dt)

            mean = np.mean(raw_data, axis=0)

            sample = []
            upper_bound = []
            lower_bound = []

            for i in range(n_runs):
                data = raw_data[:,i]
                ci = self.bootstrapci(data, np.mean)
                sample.append(np.mean(data))
                lower_bound.append(ci[0])
                upper_bound.append(ci[1])

            np.savez_compressed(
                save_file,
                mean=mean, upper_bound=upper_bound, lower_bound=lower_bound)
            print('saved to: ', save_file)

        return mean, lower_bound, upper_bound

    def proc_tot_err(self, folder, name, n_sessions, n_runs, dt=0.005, regenerate=False,
                    run_time=3):

        # account for imperfect test lengths
        run_time-=0.03
        save_file = ('error_to_target_%i_x_%i_runs_dt%0.4f_%itargets_x_%isec_%s' %
                     (n_sessions, n_runs, dt, self.n_targets,
                         self.reach_time_per_target, name))
        print('searching for: ', save_file)
        saved_data = [sf for sf in os.listdir(os.getcwd())
                      if sf == (save_file + '.npz')]
        if len(saved_data) > 0 and not regenerate:
            print('Loading data from ', saved_data[0])
            saved_data = np.load(saved_data[0])
            mean = saved_data['mean']
            upper_bound = saved_data['upper_bound']
            lower_bound = saved_data['lower_bound']
        else:
            raw_data = np.zeros((n_sessions, n_runs))
            for ii in range(n_sessions):
                for jj in range(n_runs):
                    filename_error = (
                        folder + 'session%i/run%i_data/error%i.npz' % (ii, jj, jj))
                    filename_timeintervals = (
                        folder + 'session%i/run%i_data/time%i.npz' % (ii, jj, jj))
                    error = np.load(filename_error)['error'].squeeze()
                    time_intervals = np.cumsum(np.hstack([
                        0, np.load(filename_timeintervals)['time'].squeeze()[:-1]]))
                    print(ii, ' ', jj, ' ', time_intervals[-1], ': Using first '
                          + '%f sec'%run_time)
                    # np.sum(error * dt)
                    # raw_data[ii, jj] = np.dot(error.squeeze(),
                    #                           time_intervals.squeeze())

                    # everything ran for 10s, so it's an interval problem
                    # so we're going to interpolate to even samples out
                    import scipy.interpolate
                    interp = scipy.interpolate.interp1d(time_intervals, error)
                    error = np.array([
                        interp(t) for t in np.arange(0.001, run_time, dt)])# 10-5*dt, dt)])
                        #interp(t) for t in np.arange(0.001, 19.99, dt)])# 10-5*dt, dt)])
                    raw_data[ii, jj] = np.sum(error) * dt


            mean = np.mean(raw_data, axis=0)

            sample = []
            upper_bound = []
            lower_bound = []

            for i in range(n_runs):
                data = raw_data[:,i]
                ci = self.bootstrapci(data, np.mean)
                sample.append(np.mean(data))
                lower_bound.append(ci[0])
                upper_bound.append(ci[1])

            np.savez_compressed(
                save_file,
                mean=mean, upper_bound=upper_bound, lower_bound=lower_bound)
            print('saved to: ', save_file)

        return mean, lower_bound, upper_bound


    def get_set(self, test_info, n_targets, reach_time_per_target, regenerate=False):
        test_info = np.array(test_info)
        self.n_targets = n_targets
        self.reach_time_per_target = reach_time_per_target
        run_time = n_targets * reach_time_per_target
        self.reach_time_per_target = reach_time_per_target
        self.n_target = n_targets

        parameters = ['q', 'dq', 'u', 'ee_xyz']
        os.chdir('proc_data')

        for ii in range(0,len(test_info)):
            weighted = test_info[ii,0]
            backend = test_info[ii,1]
            test_name = test_info[ii,2]
            n_runs = test_info[ii,3]
            n_sessions = test_info[ii,4]

            if not os.path.exists(test_name):
                os.makedirs(test_name)
            os.chdir(test_name)

            data_location = (cache_dir + '/saved_weights/dewolf2017/'
                             + weighted + '/' + backend + '/' + test_name + '/')

            # get error compared to ideal trajectory
            mean_data, lower_bound_data, upper_bound_data = self.proc_traj_err(
                data_location, test_name,
                n_sessions=int(n_sessions), n_runs=int(n_runs),
                regenerate=regenerate, run_time=run_time)

            # get cumulative error to target
            mean_data, lower_bound_data, upper_bound_data = self.proc_tot_err(
                data_location, test_name,
                n_sessions=int(n_sessions), n_runs=int(n_runs),
                regenerate=regenerate, run_time=run_time)

            for jj in range(0,len(parameters)):
                # process q, dq, u and ee_xyz to have consistent sampling
                self.proc_saved_data(
                    data_location, test_name,
                    n_sessions=int(n_sessions), n_runs=int(n_runs),
                    regenerate=regenerate, run_time=run_time,
                    parameter=parameters[jj])

            os.chdir('../')
        os.chdir('../')

