from abr_control.utils import DataHandler, ProcessData


def process(test_group, test_list, regenerate=False,
        n_interp_pts=400, order_of_error=1, use_cache=True, alpha=0.2,
        scaling_factor=1, n_runs=None, n_sessions=None, upper_baseline_loc=None,
        lower_baseline_loc=None)
    """
    A function that takes the provided tests from the specified test group and
    interpolates the data for even sampling, compares to an ideal generated
    trajectory, applies a filter for higher order error, and then scales to any
    provided baseline.

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
    dat = DataHandler(use_cache=use_cache)
    proc = ProcessData(use_cache=use_cache)
    orders = ['pos', 'vel', 'acc', 'jerk']
    baseline = False

    # if a baseline is provided, make sure we have both an upper and lower
    if upper_baseline_loc is None and lower_baseline_loc is not None:
        Exception('An upper baseline must be provided along with the lower'
                  + ' baseline')

    if lower_baseline_loc is None and upper_baseline_loc is not None:
        Exception('A lower baseline must be provided along with the upper'
                  + ' baseline')

    if upper_baseline_loc is not None and lower_baseline_loc is not None:
        # prepend the baseline tests to the test list
        test_list = (([].append(lower_baseline_loc)).append(upper_baseline_loc)).append(test_list)
        baseline = True

    # Cycle through the list of tests
    for nn, test in enumerate(test_list):
        # update location to the current test
        loc = '%s/%s/'%(test_group, test_list[test])
        keys = dat.get_keys(loc)

        # get the number of runs and sessions if not provided
        if n_sessions is None:
            n_sessions = max([key for key in keys if 'session' in key])
            n_sessions = int(n_sessions[7:])

        if n_runs is None:
            max_runs = []
            for key in keys if 'session' in key:
                session_keys = dat.get_keys('%s%s'%(loc,key))
                runs = max([session_key for session_key in session_keys if
                    'run' in session_key])
                max_runs.append(int(run[3:]))
            n_runs = min(max_runs)

        print('Processing up to run%s of session%s in test %s'
                % (n_runs, n_sessions, test))

        # save the base test group location
        base_loc = loc
        ideal_path_error = np.zeros(n_sessions, n_runs)

        # set thresholds for test lengths and distances to start position and
        # targets
        t_thres = 0.03 # in sec
        d_thres = 0.01 # in meters

        # cycle through the sessions
        for ii, session in enumerate(n_sessions):

            # cycle through the runs
            for jj, run in enumerate(n_runs):
                loc = '%ssession%03d/run%03d'%(base_loc, ii, jj)
                keys = dat.get_keys(loc)

                # reset error print
                print_list = ['ERROR: the tests must have the same
                        starting position and targets as the ideal
                        trajectory, along with a total test length
                        within a %fsec threshold'%t_thres, 'TEST:
                        %s' % base_loc]

                # ----- GENERATE IDEAL PATH -----
                # only generate at the start before we process any tests
                if nn==0:
                    # use the lower baseline parameters for generating the
                    # ideal trajectory, the other tests should have the same
                    # starting point, targets, and test length, otherwise an
                    # error will be thrown
                    loaded_data = dat.load(params=['ee_xyz', 'time', 'target_xyz'],
                            save_location=loc)
                    ideal_reaching_time = loaded_data['time']
                    target_xyz = loaded_data['target_xyz']
                    ideal_start_xyz = loaded_data['ee_xyz'][0]

                    # only save the unique target locations
                    target_set = set()
                    ideal_target_xyz = []
                    for item in target_xyz:
                        if item not in target_set:
                            target_set.add(item)
                            ideal_target_xyz.append(item)
                        else:
                            pass

                    ideal_path = proc.generate_ideal_path(
                            reaching_time=np.sum(ideal_reaching_time),
                            target_xyz=ideal_target_xyz, start_xyz=ideal_start_xyz)
                    # interpolate the ideal path to the same sampling rate as
                    # the tests
                    ideal_path = proc.interpolate_data(data=ideal_path,
                            time_intervals=ideal_reaching_time, n_points=n_interp_pts)

                # ----- INTERPOLATE END-EFFECTOR POSITION DATA -----
                # check if we have interpolated data saved
                if 'ee_xyz_interp_%s'%orders[order_of_error] in keys and not regenerate:
                    # data exists and user does not want to regenerate
                    pass

                else:
                    # load test data
                    loaded_data = dat.load(params=['ee_xyz', 'time',
                        'target_xyz'], save_location=loc)
                    ee_xyz = loaded_data['ee_xyz']
                    time = loaded_data['time']
                    target_xyz_list = target_xyz['target_xyz']

                    # only save the unique target locations
                    target_set = set()
                    target_xyz = []
                    for item in target_xyz_list:
                        if item not in target_set:
                            target_set.add(item)
                            target_xyz.append(item)
                        else:
                            pass

                    # check to make sure that the reaching time and targets
                    # match the ones used in the ideal trajectory, otherwise
                    # throw an error
                    params_to_check = ['start_xyz', 'target_xyz',
                            'reaching_time']
                    param_check = [False, False, False]

                    #TODO check that this works
                    if (abs(np.array(ee_xyz[0]) - np.array(ideal_start_xyz)) > d_thres).any() :
                        param_check[0] = True
                        print_list.append('Starting positions do not match...')
                        print_list.append('Ideal: ', ideal_start_position)
                        print_list.append('Test: ', ee_xyz[0])

                    #TODO check that this works
                    if (abs(np.array(target_xyz[0]) - np.array(ideal_target_xyz)) > d_thres).any() :
                        param_check[1] = True
                        print_list.append('Targets do not match...')
                        print_list.append('Ideal: ', ideal_target_xyz)
                        print_list.append('Test: ', target_xyz)

                    if (np.sum(time) > np.sum(ideal_reaching_time) + t_thres or
                        np.sum(time) < np.sum(ideal_reaching_time) - t_thres)
                        param_check[2] = True
                        print_list.append('Test lengths do not match within'
                                          + ' threshold...')
                        print_list.append('Ideal: ',
                                np.sum(ideal_reaching_time),
                                '+/- %fsec'%t_thres)
                        print_list.append('Test: ', np.sum(time))

                    if any(param_check):
                        to_print = '\n'.join(print_list)
                        Exception(to_print)
                        
                    # if our parameters match the ideal path interpolate data for even sampling
                    ee_xyz_interp = proc.interpolate_data(data=ee_xyz,
                            time_intervals=time, n_points=n_interp_pts)

                    # save the interpolated error data
                    dat.save(data={'ee_xyz_interp_%s'%orders[order_of_error] : ee_xyz_interp},
                            save_location=loc, overwrite=True, timestamp=False)

                # ----- FILTER AND COMPARE TO IDEAL PATH -----
                # Check if we have comparative error data saved
                #if 'ideal_path_error_%s'%orders[order_of_error] in keys and not regenerate:
                    # data exists and user does not want to regenerate
                #    pass

                #else:
                    # calculate the error relative to an ideal path
                path_error = proc.calc_path_error_to_ideal(
                        ideal_path=ideal_path, recorded_path=ee_xyz_interp,
                        order_of_error=order_of_error, alpha=alpha)

                ideal_path_error[ii, jj] = np.copy(path_error)
                #TODO add status printing to see progress
                #TODO: use progress bar that was used in gif creator

            # ----- SCALE TO SPECIFIED BASELINE -----
            if baseline:
                # if we're passed processing the lower and upper baseline
                if nn > 1:
                session_error = proc.scale_data(input_data=ideal_path_error[ii],
                    baseline_low=lower_baseline, baseline_high=upper_baseline,
                    scaling_factor=scaling_factor)
            else:
                session_error = ideal_path_error[ii] * scaling_factor

            # save the error to ideal path data
            dat.save(data={'ideal_path_error': session_error},
                    save_location = '%ssession%03d'%(base_loc, ii),
                    overwrite=True, timestamp=False)

        # ----- CI AND MEAN -----
        sample = []
        upper_bound = []
        lower_bound = []

        for i in range(n_runs):
            data = raw_data[:,i]
            ci = self.bootstrapci(data, np.mean)
            sample.append(np.mean(data))
            lower_bound.append(ci[0])
            upper_bound.append(ci[1])

        final_error = {'mean': sample,
                       'upper_bound': upper_bound,
                       'lower_bound': lower_bound,
                       'n_runs': n_runs,
                       'n_sessions': n_sessions,
                       'upper_baseline': upper_baseline_loc,
                       'lower_baseline': lower_baseline_loc}
                       
        # save the mean error and bounds relative to ideal
        dat.save(data=final_error,
                save_location = base_loc,
                overwrite=True, timestamp=False)

        # if we're using a baseline save our baseline data if they are done being processed
        if nn == 0:
            lower_baseline = sample
        if nn == 1:
            upper_baseline = sample
