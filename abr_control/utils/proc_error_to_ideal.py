from abr_control.utils import DataHandler, ProcessData


def process(test_group, test_list, regenerate=False,
        n_interp_pts=400, order_of_error=1, use_cache=True, alpha=0.2,
        scaling_factor=100, n_runs=None, n_sessions=None)
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
    scaling_factor: int, Optional (Default: 100)
        the scaling factor to apply after scaling to a baseline. Default is 100
        to allow for plotting a percent-error
    n_sessions: int, Optional (Default:None)
        if left as None, then the database will be checked for how many sessions
        are in the provided test
    n_runs: int, Optional (Default:None)
        if left as None, then the database will be checked for the lowest
        number of runs in the sessions of the test provided
    """
    dat = DataHandler(use_cache=use_cache)
    proc = ProcessData(use_cache=use_cache)
    #TODO Need a clean way to specify the baseline used for scaling, how to
    # find and point to the test easily

    # ----- GENERATE IDEAL PATH -----
    #TODO pass parameters in to generate ideal path
    ideal_path = proc.generate_ideal_path()

    for test in test_list:
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

        base_loc = loc

        for ii, session in enumerate(n_sessions):


#TODO!!!!!!!!!!!!!!!!!!!!!!!!!! DECIDE HOW TO LOOP THROUGH RUNS AND SESSIONS


            for jj, run in enumerate(n_runs):
                loc = '%s%s/%s'%(base_loc, session, run())
                keys = dat.get_keys(loc)

                # ----- INTERPOLATE END-EFFECTOR POSITION DATA -----
                # check if we have interpolated data saved
                if 'ee_xyz_interp' in keys and not regenerate:
                    # data exists and user does not want to regenerate
                    pass

                else:
                    # load ee_xyz and time
                    [ee_xyz, time, target_xyz] = dat.load(params=['ee_xyz', 'time',
                        'target_xyz'], save_location=loc)
                    print('ee_xyz loaded: ', ee_xyz)
                    print('time_loaded: ', time)
                    print('target_xyz loaded: ', target_xyz)
                    starting_xyz = ee_xyz['ee_xyz'][0]
                    time = time['time']
                    #TODO Find way of getting only unique target locations since these
                    # will repeat for the length of reaching time
                    target_xyz = target_xyz['target_xyz']

                    # interpolate data for even sampling
                    # 400 points for a 20 second run is ~ a point every 5ms
                    ee_xyz_interp = proc.interpolate_data(data=ee_xyz,
                            time_intervals=time, n_points=n_interp_pts)

                    # save the interpolated data
                    dat.save(data={'ee_xyz_interp': ee_xyz_interp}, save_location=loc,
                        overwrite=True, timestamp=False)

                # ----- COMPARE TO IDEAL PATH -----
                # Check if we have comparative error data saved
                if 'ideal_path_error' in keys and not regenerate:
                    # data exists and user does not want to regenerate
                    pass

                else:
                    # calculate the error relative to an ideal path
                    ideal_path_error = proc.calc_path_error_to_ideal(
                            ideal_path=ideal_path, recorded_path=ee_xyz_interp,
                            n_session=n_sessions, n_runs=n_runs,
                            order_of_error=order_of_error)

                    # ----- APPLY FILTERS -----
                    if order_of_error > 1:
                        # Apply a filter if using second or third order error as it gets noisy
                        ideal_path_error = proc.filter_data(data=ideal_path_error,
                                alpha=alpha)

                    # save the new error data
                    dat.save(data={'ideal_path_error': ideal_path_error}, save_location=loc,
                        overwrite=True, timestamp=False)

                # ----- SCALE TO SPECIFIED BASELINE -----
                if 'scaled_ideal_path_error' in keys and not regenerate:
                    # scaled data exists and user does not want to regenerate
                    pass
                else:
                    scaled_error = proc.scale_data(input_data=ideal_path_error,
                        baseline_low=baseline_low, baseline_high=baseline_high,
                        scaling_factor=scaling_factor)

                    # save the scaled data
                    dat.save(data={'scaled_ideal_path_error': scaled_error}, save_location=loc,
                        overwrite=True, timestamp=False)
