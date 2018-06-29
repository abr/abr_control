import numpy as np

from abr_control.utils import DataHandler, ProcessData

class PathErrorToIdeal():
    """

    """

    def __init__(self):
        pass

    def get_unique_targets(self, target_xyz):
        #TODO find a better way to do this
        unique_target_xyz = []
        for target in target_xyz:
            match = False
            for saved in unique_target_xyz:
                if target[0] != saved[0]:
                    if target[1] != saved[1]:
                        if target[2] != saved[2]:
                            match = False
                        else: match = True
                    else: match = True
                else: match = True
            if not match:
                unique_target_xyz.append(np.copy(target))

        return unique_target_xyz


    def process(self, test_group, test_list, regenerate=False,
            n_interp_pts=400, order_of_error=1, alpha=0.2, use_cache=True,
            scaling_factor=1, n_runs=None, n_sessions=None, upper_baseline_loc=None,
            lower_baseline_loc=None, d_thres=0.03, t_thres=0.03, db_name=None):

        """
        A function that takes the provided tests from the specified test group and
        interpolates the data for even sampling, compares to an ideal generated
        trajectory, applies a filter for higher order error, and then scales to any
        provided baseline.
:
        test_group: string
            the test group location in the hdf5 database
        test_list: list of strings
            the tests in the above specified test group to plot
        regenerate: Boolean, Optional(Default: False)
            whether to regenerate data if it already exists
        n_interp_pts: int, Optional (Default: 400)
            how many points to interpolate the provided data to
        order_of_error: int, Optional (Default: 1)
            the order of error to process
            0 == position error
            1 == velocity error
            2 == acceleration error
            3 == jerk error
        use_cache: Boolean, Optional (Default: True)
            True if saving database to the abr_control .cache location
            False if saving elsewhere, in this case the entire location prior to
            test_group must be prepended to the test_group parameter
        db_name: string, Optional (Default: abr_control_db if left as None)
            name of the database being used
        alpha: float, Optional (Default: 0.2)
            the filter constant to use when filtering higher order error
        scaling_factor: int, Optional (Default: 1)
            the scaling factor to apply after scaling to a baseline.
        n_sessions: int, Optional (Default:None)
            if left as None, then the database will be checked for how many sessions
            are in the provided test
        n_runs: int, Optional (Default:None)
            if left as None, then the database will be checked for the lowest
            number of runs in the sessions of the test provided
        upper_baseline_loc: string, Optional (Default: None)
            location in database to the test that will be used as the upper bound
            of the baseline calculation. If None no baseline adjustment will be
            made. Test has to be in the same test_group as passed in.
        lower_baseline_loc: string, Optional (Default: None)
            location in database to the test that will be used as the lower bound
            of the baseline calculation. If None no baseline adjustment will be
            made. Test has to be in the same test_group as passed in.
        """
        dat = DataHandler(use_cache=use_cache, db_name=db_name)
        proc = ProcessData()
        orders = ['position', 'velocity', 'acceleration', 'jerk']
        baseline = False

        # if a baseline is provided, make sure we have both an upper and lower
        if upper_baseline_loc is None and lower_baseline_loc is not None:
            Exception('An upper baseline must be provided along with the lower'
                      + ' baseline')

        elif lower_baseline_loc is None and upper_baseline_loc is not None:
            Exception('A lower baseline must be provided along with the upper'
                      + ' baseline')

        elif upper_baseline_loc is not None and lower_baseline_loc is not None:
            # prepend the baseline tests to the test list
            # if we are using baselines, they will be the first two tests
            # processed
            tests = []
            tests.append(lower_baseline_loc)
            tests.append(upper_baseline_loc)
            for test in test_list:
                tests.append(test)
            test_list = tests
            baseline = True

        print('--------------------')
        print('Calculating %s error' % orders[order_of_error])
        print('Lower Error Baseline: %s' % lower_baseline_loc)
        print('Upper Error Baseline: %s' % upper_baseline_loc)
        print('Test List: ', test_list)
        print('--------------------')

        # Cycle through the list of tests
        for nn, test in enumerate(test_list):
            # update location to the current test
            loc = '%s/%s/'%(test_group, test)
            keys = dat.get_keys(loc)

            # get the number of runs and sessions if not provided
            if n_sessions is None:
                n_sessions = max([key for key in keys if 'session' in key])
                # add 1 since base 0
                n_sessions = int(n_sessions[7:]) + 1

            if n_runs is None:
                max_runs = []
                for key in keys:
                    if 'session' in key:
                        #print('cur key: ', key)
                        session_keys = dat.get_keys('%s%s'%(loc,key))
                        #print('ses keys: ', session_keys)
                        runs = max([session_key for session_key in session_keys if
                            'run' in session_key])
                        max_runs.append(int(runs[3:]))
                # add 1 since base 0
                n_runs = min(max_runs) + 1

            print('%i/%i: %s: %s runs %s sessions' %(nn, len(test_list), test,
                n_runs, n_sessions))

            # save the base test group location
            base_loc = loc
            ideal_path_error = np.zeros((n_sessions, n_runs))

            # cycle through the sessions
            for ii in range(n_sessions):

                # cycle through the runs
                for jj in range(n_runs):
                    loc = '%ssession%03d/run%03d'%(base_loc, ii, jj)
                    keys = dat.get_keys(loc)

                    # reset error print
                    print_list = ['\n', 'ERROR: the tests must have the same '
                            'starting position and targets as the ideal '
                            'trajectory, along with a total test length '
                            'within a %fsec threshold'%t_thres,
                            ' TEST: %s' % base_loc]

                    # ----- GENERATE IDEAL PATH -----
                    # only generate at the start before we process any tests
                    if nn + ii + jj == 0:
                        print('Ideal path not found, generating...')
                        # use the lower baseline parameters for generating the
                        # ideal trajectory, the other tests should have the same
                        # starting point, targets, and test length, otherwise an
                        # error will be thrown
                        loaded_data = dat.load(params=['ee_xyz', 'time',
                            'target'],
                                save_location=loc)
                        reaching_time = loaded_data['time']
                        # our path planner creates 4000 steps so we need to
                        # create the corresponding timesteps for interpolation
                        target_xyz = loaded_data['target']
                        ideal_start_xyz = loaded_data['ee_xyz'][0]

                        # only save the unique target locations
                        #target_set = set()
                        ideal_target_xyz = self.get_unique_targets(target_xyz)
                        # print('Ideal Reaching Time: ',
                                # np.sum(reaching_time))
                        # print('Ideal Start XYZ: ', ideal_start_xyz)
                        # print('Ideal Target XYZ: ', ideal_target_xyz)

                        [ideal_reaching_time, ideal_path] = proc.generate_ideal_path(
                                reaching_time=np.sum(reaching_time)/len(ideal_target_xyz),
                                target_xyz=ideal_target_xyz, start_xyz=ideal_start_xyz)

                        # interpolate the ideal path to the same sampling rate as
                        # the tests
                        # print('ideal path shape: ', ideal_path.shape)
                        print('Interpolating ideal path for even sampling')
                        ideal_path = proc.interpolate_data(
                                data=ideal_path[:, :3],
                                time_intervals=ideal_reaching_time,
                                n_points=n_interp_pts)

                    # ----- INTERPOLATE END-EFFECTOR POSITION DATA -----
                    # print the progress as a percentage
                    print('%.3f processing complete...'
                            %(100*(((n_runs+1)*ii
                                + jj)/((n_sessions+1)*(n_runs+1)))),
                            end='\r')
                    # check if we have interpolated data saved
                    # load test data
                    loaded_data = dat.load(params=['ee_xyz', 'time',
                        'target'], save_location=loc)
                    ee_xyz = loaded_data['ee_xyz']
                    time = loaded_data['time']
                    target_xyz = loaded_data['target']

                    # only save the unique target locations
                    target_xyz = self.get_unique_targets(target_xyz)

                    # check to make sure that the reaching time and targets
                    # match the ones used in the ideal trajectory, otherwise
                    # throw an error
                    # 'start_xyz', 'target', 'reaching_time'
                    param_check = [False, False, False]

                    # Check if our starting position is within tolerance of
                    # the ideal path
                    if (abs(np.array(ee_xyz[0]) - np.array(ideal_start_xyz))
                            > d_thres).any():
                        param_check[0] = True
                        print_list.append('Starting positions do not match...')
                        print_list.append('Ideal: %s'% ideal_start_xyz)
                        print_list.append('Test: %s'% ee_xyz[0])
                        print('!!!start_xyz test triggered!!!')

                    # Check if our target locations are within tolerance of
                    # the ones used for the ideal path
                    if (abs(np.array(target_xyz) - np.array(ideal_target_xyz))
                            > d_thres).any():
                        param_check[1] = True
                        print_list.append('Targets do not match...')
                        print_list.append('Ideal: %s'% ideal_target_xyz)
                        print_list.append('Test: %s'% target_xyz)
                        print('!!!target_xyz test triggered!!!')

                    # Check if our test length is within tolerance of the
                    # ideal path test length
                    if np.sum(time)>(np.sum(ideal_reaching_time)+t_thres)or \
                            np.sum(time)<(np.sum(ideal_reaching_time)-t_thres):
                        param_check[2] = True
                        print_list.append('Test lengths do not match within'
                                          + ' threshold...')
                        print_list.append('Ideal: %f +/-%f sec'%
                                (np.sum(ideal_reaching_time),
                                 t_thres))
                        print_list.append('Test: %f'% np.sum(time))
                        print('!!!reaching_time test triggered!!!')

                    if any(param_check):
                        to_print = '\n'.join(print_list)
                        raise ValueError('\n'.join(print_list))

                    # used for differentiating higher orders of error
                    run_time = sum(time)
                    time_intervals = np.cumsum(time)
                    dt = (run_time-time_intervals[0])/n_interp_pts

                    # if our parameters match the ideal path interpolate data for even sampling

                    if 'ee_xyz_interp_%s'%orders[order_of_error] in keys and not regenerate:
                        # data exists and user does not want to regenerate
                        #print('%s ee_xyz_interp exists, passing...'%loc)
                        ee_xyz_interp = dat.load(
                                params=['ee_xyz_interp_%s'%orders[order_of_error]],
                                save_location='%ssession%03d/interp_data/run%03d'%(base_loc,
                                    ii, jj))
                        ee_xyz_interp = ee_xyz_interp['ee_xyz_interp_%s'%orders[order_of_error]]

                    else:
                        #print('4: Interpolating Data')
                        ee_xyz_interp = proc.interpolate_data(data=ee_xyz,
                                time_intervals=time, n_points=n_interp_pts)

                        # Calculate the correct order of error for the recorded path
                        for mm in range(0, order_of_error):
                            ee_xyz_interp = np.vstack([np.zeros((1, 3)),
                                np.diff(ee_xyz_interp, axis=0) / dt])

                        # save the interpolated error data
                        #print('5: Saving interpolated data')
                        #TODO: do we want to save this?
                        dat.save(data={'ee_xyz_interp_%s'%orders[order_of_error] : ee_xyz_interp},
                                save_location='%ssession%03d/interp_data/run%03d'%(base_loc,
                                    ii, jj), overwrite=True, timestamp=False)

                    # Calculate the correct order of error for the ideal path
                    # set ideal_path_diff to ideal_path in case we are at order
                    # 0 and the for loop does not trigger
                    ideal_path_diff = ideal_path
                    for mm in range(0, order_of_error):
                        ideal_path_diff = np.vstack([np.zeros((1, 3)), np.diff(ideal_path, axis=0) / dt])

                    # ----- FILTER AND COMPARE TO IDEAL PATH -----
                    #print('6: Calculating path error to ideal')
                    path_error = proc.calc_path_error_to_ideal(
                            dt=dt,
                            ideal_path=ideal_path_diff, recorded_path=ee_xyz_interp,
                            order_of_error=order_of_error, alpha=alpha)

                    #print('7: Storing path error to ideal for session %i : run %i'
                            #%(ii, jj))
                    ideal_path_error[ii, jj] = np.copy(path_error)
                    #TODO add status printing to see progress
                    #TODO: use progress bar that was used in gif creator

                # ----- SCALE TO SPECIFIED BASELINE -----
                # if we're passed processing the lower and upper baseline
                if baseline and nn > 1:
                    #print('8a: Doing Baseline Calculation')
                    session_error = proc.scale_data(input_data=ideal_path_error[ii],
                        baseline_low=lower_baseline, baseline_high=upper_baseline,
                        scaling_factor=scaling_factor)
                else:
                    session_error = ideal_path_error[ii] * scaling_factor
                    #print('8b: Applying scaling factor')

                # save the error to ideal path data
                #TODO: should we even save at this step?
                # print('9: Saving final processed session error')
                # dat.save(data={'ideal_path_error': session_error},
                #         save_location = '%ssession%03d'%(base_loc, ii),
                #         overwrite=True, timestamp=False)

                # save the final session error
                ideal_path_error[ii] = session_error

            # ----- CI AND MEAN -----
            print('100.00000% processing complete   ')
            #print('10: Getting CI and bounds')
            stat_data = proc.get_mean_and_ci(raw_data=ideal_path_error,
                                             n_runs=n_runs)
            mean_data = stat_data[0]
            lower_bound = stat_data[1]
            upper_bound = stat_data[2]

            final_error = {'mean': mean_data,
                           'upper_bound': upper_bound,
                           'lower_bound': lower_bound,
                           'n_runs': n_runs,
                           'n_sessions': n_sessions,
                           'upper_baseline': upper_baseline_loc,
                           'lower_baseline': lower_baseline_loc}

            # save the mean error and bounds relative to ideal
            #print('11: Saving CI and Bound Data')
            dat.save(data=final_error,
                    save_location = base_loc
                    + 'proc_data/%s'%orders[order_of_error],
                    overwrite=True, timestamp=False)

            # if we're using a baseline save our baseline data if they are done being processed
            if baseline and nn == 0:
                lower_baseline = mean_data
                # lower_baseline = np.load('pd_5pt_baseline_traj_error.npz')['mean']
            if baseline and nn == 1:
                upper_baseline = mean_data
                # upper_baseline = np.load('pd_5pt_baseline_traj_error_friction.npz')['mean']
