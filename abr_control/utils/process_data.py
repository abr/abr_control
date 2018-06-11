"""
Class for processing data including: interpolating for even sampling,
calculating average and confidence intervals, scaling data, filtering data, and
comparing to an ideal trajectory

Process for comparing to ideal trajectory
1. interpolate data for even sampling
2. generate ideal trajectory with the same sampling
3. calculate error from recorded path to ideal
4. filter data if desired (recommended for 2nd and 3rd order error)
5. scale to a baseline if desired
"""
import numpy as np
import os
import scipy.interpolate

from abr_control.utils.paths import cache_dir
from abr_control.utils import DataHandler
from abr_control.controllers import path_planners

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

    def interpolate_data(data, time_intervals, n_points):
        run_time = sum(time_intervals)
        dt = run_time/n_points
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

    def generate_ideal_path(reaching_time, target_xyz, start_xyz):

        if target_xyz is None:
            print('ERROR: Must provide target(s)')
        x_track = []
        u_track = []
        # create our point mass system dynamics dx = Ax + Bu
        x = np.hstack([self.start_xyz, np.zeros(3)])  # [x, y, z, dx, dy, dz]
        A = np.array([
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0]])
        u = np.array([0, 0, 0])  # [u_x, u_y, u_z]
        B = np.array([
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]])

        # create our state and control cost matrices
        Qs = np.ones(6) * 10
        # Rs = [0.35, 0.225, 0.5, 0.2, 0.225]
        Rs = np.ones(6) * 1e-3

        # interpolation sampling rate
        dt = 0.001
        timesteps = int(reaching_time / dt)
        print('time steps: ', timesteps)
        continuous = False

        vmax = 1
        kp = 20
        kv = 6
        lamb = kp / kv

        path = path_planners.SecondOrder(None)
        w = 1e4 / timesteps

        for ii, target in enumerate(target_xyz):
            u = np.zeros(3)
            Q = Qs[ii] * np.eye(6)
            R = Rs[ii] * np.eye(3)
            print('II: ', ii)

            for t in range(timesteps):
                # track trajectory
                x_track.append(np.copy(x))
                u_track.append(np.copy(u))

                temp_target = path.step(
                    y=target, dy=np.zeros(3),
                    target=target, w=w,
                    zeta=2, dt=0.003, threshold=0.05)

                # calculate the position error
                x_tilde = np.array(x[:3] - temp_target[:3])

                # implement velocity limiting
                sat = vmax / (lamb * np.abs(x_tilde))
                if np.any(sat < 1):
                    index = np.argmin(sat)
                    unclipped = kp * x_tilde[index]
                    clipped = kv * vmax * np.sign(x_tilde[index])
                    scale = np.ones(3, dtype='float32') * clipped / unclipped
                    scale[index] = 1
                else:
                    scale = np.ones(3, dtype='float32')

                u = -kv * (x[3:] - temp_target[3:] -
                                    np.clip(sat / scale, 0, 1) *
                                    -lamb * scale * x_tilde)

                # move simulation one time step forward
                dx = np.dot(A, x) + np.dot(B, u)
                x += dx * dt

        u_track = np.array(u_track)
        x_track = np.array(x_track)

        return [u_track, x_track]

    def filter_data(self, data, alpha=0.2):
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

    def calc_path_error_to_ideal(ideal_path, recorded_path, order_of_error,
            alpha=0.2):
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
        # only need the ideal path for the order of error we are interested in
        ideal_path = ideal_path[:3]

        # Calculate the correct order of error
        for ii in range(0, order_of_error):
            recorded_path = np.diff(recorded_path, axis=0) / dt
            ideal_path = np.diff(ideal_path, axis=0) /dt

        # ----- APPLY FILTERS -----
        # if we're using acceleration or jerk, add a filter to
        # clean up the signal
        if order_of_error > 1:
            recorded_path = self.filter_data(data=recorded_path,
                    alpha=alpha)
            ideal_path = self.filter_data(data=ideal_path,
                    alpha=alpha)


        # error relative to ideal path
        error_to_ideal = (np.sum(np.sqrt(np.sum(
            (ideal_path - recorded_path)**2,
            axis=1))))*dt

        return error_to_ideal
