"""
Class for processing data including: interpolating for even sampling,
calculating average and confidence intervals, scaling data, filtering data, and
comparing to an ideal trajectory

Process for comparing to ideal trajectory
1. interpolate data for even sampling
2. generate ideal trajectory with the same sampling (auto generated in step3)
3. calculate error from recorded path to ideal
4. filter data if desired (recommended for 2nd and 3rd order error)
5. scale to a baseline if desired
"""
import numpy as np
import os
import scipy.interpolate

from abr_control.utils.paths import cache_dir
from abr_control.utils import DataHandler

class ProcessData():
    def __init__(use_cache=True):
        """

        Parameters
        ----------
        use_cache: Boolean, Optional (Default:True)
            True to prepend the abr_control cache folder to the directory
            provided for saving. This location is specified in
            abr_control/utils/paths.py
            False to use the directory passed in as is
        """
        #TODO: fix the case if user doesn't want to use the cache dir
        self.dat = DataHandler(use_cache=use_cache)

    def get_mean_and_ci(raw_data, n_runs)
        sample = []
        upper_bound = []
        lower_bound = []

        for i in range(n_runs):
            data = raw_data[:,i]
            ci = self.bootstrapci(data, np.mean)
            sample.append(np.mean(data))
            lower_bound.append(ci[0])
            upper_bound.append(ci[1])

        return(mean=sample, upper_bound=upper_bound, lower_bound=lower_bound)

    def bootstrapci(self, data, func, n=3000, p=0.95):
        index=int(n*(1-p)/2)
        samples = np.random.choice(data, size=(n, len(data)))
        r = [func(s) for s in samples]
        r.sort()
        return r[index], r[-index]

    def interpolate_data(data, time_intervals, run_time, dt):
        # interpolate to even samples out
        data_interp = []
        for kk in range(data.shape[1]):
            interp = scipy.interpolate.interp1d(time_intervals, data[:, kk])
            data_interp.append(np.array([
                interp(t) for t in np.arange(0.001, run_time, dt)]))
        data_interp = np.array(data_interp).T

        return data_interp

    def scale_data(input_data, baseline_low, baseline_high, scaling_factor=1):
        """
        scale data to some baseline to get values from 0-1 relative
        to baseline times the scaling factor

        PARAMETERS
        input_data: list of floats
            the data to be scaled
        baseline_low: list of floats
            the lower error baseline that will be the zero
            reference
        baseline_high: list of floats
            the higher error baseline that will be the one
            reference
        """
        scaled_data = ((input_data - baseline_low)
                       / (baseline_high - baseline_low))
        scaled_data *= scaling_factor

        return scaled_data

    def generate_ideal_path():
        # ideal acceleration
        pd_xyz_ddx = np.diff(pd_xyz_dx, axis=0) / dt
        # ideal jerk
        pd_xyz_dddx = np.diff(pd_xyz_ddx, axis=0) / dt
        return ideal_path

    def filter_data(data, alpha=0.2):
        data_filtered = []
        for nn in range(0,len(data)):
            if nn == 0:
                data_filtered.append((alpha*data[nn]
                                          + (1-alpha)*0))
            else:
                data_filtered.append((alpha*data[nn]
                                          + (1-alpha)*data_filtered[nn-1]))
        data_filtered = np.array(data_filtered)

        return data_filtered

    def path_error_to_ideal(self, folder, name, n_sessions, n_runs, error_type,
                      run_time=3, dt=0.005, regenerate=False, order_of_error=0):
        """
        Function for passing already interpolated data in to compare to an
        ideal path (of the same interpolation)

        Parameters
        ----------
        order_of_error: int, Optional (Default: 0)
            the order of error to calculate
            0 == position error
            1 == velocity error
            2 == acceleration error
            3 == jerk error

        """

        # account for imperfect test lengths
        run_time-=0.03

        # check database location for how many runs and sessions we have if
        # none are provided

        # check if processed data already exists, if so check if user wants to
        # regenerate

        # generate the ideal path and save it to the session level group, if
        # one exists overwrite it so we have the actual ideal that was used for
        # the current calculation saved
        ideal_path = generate_ideal_path()
        # only need the ideal path for the order of error we are interested in
        ideal_path = ideal_path[order_of_error]

        # instantiate arrays for saving error
        raw_data = np.zeros((n_sessions, n_runs))
        error = np.zeros((n_sessions, n_runs))

        # cycle through the sessions
        for ii in range(n_sessions):

            # cycle through the runs
            for jj in range(n_runs):
                # load data from run
                recorded_path = np.load(filename_recorded_path)['ee_xyz'].squeeze()
                times = np.load(filename_timeintervals)['time'].squeeze()[:-1]
                time_intervals = np.cumsum(np.hstack([0, times]))
                print('Session %i run %i ran for %f seconds ' %
                      (ii, jj, time_intervals[-1]))

                # Calculate the correct order of error
                for ii in range(0, order_of_error):
                    recorded_path = np.diff(recorded_path, axis=0) / dt]

                # error relative to ideal path
                error_to_ideal = (np.sum(np.sqrt(np.sum(
                    (ideal_path - recorded_path)**2,
                    axis=1))))*dt

                # save the error for this run
                error[ii,jj] = np.copy(error_to_ideal)

        return error
