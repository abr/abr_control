import glob
import redis
import struct
import os
import time
import numpy as np
import scipy.special
import logging
r = redis.StrictRedis(host='127.0.0.1')

# import abr_control.utils.os_utils
from abr_control.utils.paths import cache_dir

class DataHandler():
    """
    Data handler for saving and loading data

    This class is meant to help simplify running automated tests. This is
    specifically helpful when running consecutive runs of learning where it is
    desired to pick up where the last run ended off.

    The naming convention used is as follows:
    - runs are consecutive tests, usually where the previous run is used as
    a starting point for the following run
    - a group of runs is called a session, multiple sessions are used for
      averaging and other statistical purposes like getting confidence
      intervals
    - a test_name is the user specified name of the test being run, which is
      made up of sessions that consist of several runs
    - by default the test_name data is saved in the abr_control .cache folder.
      This folder can be found in abr_control/utils/paths.py. However, the user
      can prevent prepending the test_name folder with the abr_cache by setting
      use_cache to false
    """

    def __init__(self):
        pass

    def last_save_location(self, session=None, run=None, test_name='test',
            use_cache=True):
        """ Search for most recent saved weights

        If the user sets session or run to None, the function searches the
        specified test_name for the highest numbered run in the
        highest numbered session, otherwise the specified values are used.
        Returns the file location and the number of the most recent run.

        If the user specifies a test_name, session, or run that do not exist,
        they will be created.

        Parameters
        ----------
        session: int, Optional (Default: None)
            the session number of the current set of runs
            if set to None, then the latest session in the test_name folder
            will be use, based off the highest numbered session
        run: int, Optional (Default: None)
            the run number under which to save data
            if set to None, then the latest run in the test_name/session#
            folder will be used, based off the highest numbered run
        test_name: string, Optional (Default: 'test')
            the folder name that will hold the session and run folders
            the convention is abr_cache_folder/test_name/session#/run#
            The abr cache folder can be found in abr_control/utils/paths.py
        use_cache: Boolean, Optional (Default:True)
            True to prepend the abr_control cache folder to the directory
            provided. This location is specified in abr_control/utils/paths.py
            False to use the directory pass in as is
        """

        if use_cache:
            test_name = cache_dir + '/' + test_name

        # if session is not None, then the user has specified what session to save the
        # current data in
        if session is not None:
            test_name += '/session%i' % session
        # If no session is specified, check if there is a session in the
        # directory provided, if not then save the run to 'session0'
        else:
            prev_sessions = glob.glob(test_name + '/session*')
            run_cache = []
            if prev_sessions:
                test_name = max(prev_sessions)
            else:
                test_name += '/session0'

        # check if the provided test_name exists, if not, create it
        self.check_folder_exists(directory=test_name, use_cache=False)

        # check what the last entry is based off the largest run number, unless
        # one is specified by the user. If none is found then return None
        if run is None:
            prev_runs = glob.glob(test_name + '/run*')
            run_cache = []
            if prev_runs:
                for entry in prev_runs:
                    # extract the numbers appened to the runs found and get the
                    # largest entry
                    extracted_num = re.search(r'\d+$', entry)
                    run_cache.append(int(extracted_num.group()))
                run_num = max(run_cache)
            else:
                run_num = None
        else:
            # return the run specified by the user
            run_num = run

        return [test_name, run_num]

    def check_folder_exists(self, directory, use_cache=True):
        """
        Checks if the provided directory exists.

        Parameters
        ----------
        directory: string
            The directory that the function checks for, if it does not exists
            it is created
        use_cache: Boolean, Optional (Default:True)
            True to prepend the abr_control cache folder to the directory
            provided. This location is specified in abr_control/utils/paths.py
            False to use the directory pass in as is
        """
        if use_cache:
            directory = cache_dir + '/' + directory

        if not os.path.exists(directory):
            print("The provided directory: '%s' does not exist, creating folder..."
                % directory)
            os.makedirs(directory)

    def save_data(self, tracked_data, session=None, run=None,
            test_name='test', use_cache=True):
        """ Save the data to the specified test_name folder

        Uses the naming structure of a session being made up of several runs.
        This allows the user to automate scripts for saving and loading data
        between consecutive runs. These sets of runs are saved in a session, so
        multiple sessions can be run for averaging and other statistical
        purposes

        Parameters
        ----------
        tracked_data: dictionary of lists to save
            instantiate as
                tracked_data = {'data1': [], 'data2': []}
            append as
                tracked_data['data1'].append(np.copy(data_to_save))
                tracked_data['data2'].append(np.copy(other_data_to_save))
        session: int, Optional (Default: None)
            the session number of the current set of runs
            if set to None, then the latest session in the test_name folder
            will be use, based off the highest numbered session
        run: int, Optional (Default: None)
            the run number under which to save data
            if set to None, then the latest run in the test_name/session#
            folder will be used, based off the highest numbered run
        test_name: string, Optional (Default: 'test')
            the folder name that will hold the session and run folders
            the convention is abr_cache_folder/test_name/session#/run#
            The abr cache folder can be found in abr_control/utils/paths.py
        use_cache: Boolean, Optional (Default:True)
            True to prepend the abr_control cache folder to the directory
            provided. This location is specified in abr_control/utils/paths.py
            False to use the directory pass in as is
        """

        if session or run is None:
            # user did not specify either run or session so we will grab the
            # last entry in the test_name directory based off the highest
            # numbered session and/or run
            [test_name, run_num] = last_save_location(session=session,
                    run=run, test_name=test_name, use_cache=use_cache)

        # if the user specified a run, then save the data under that run,
        # otherwise increment the last saved run by 1 to save data in a new
        # folder
        if run is None:
            run_num += 1

        save_folder ='%s/run%i'%(test_name, run_num)
        # check whether the run folder exists
        check_folder_exists(directory=save_folder, use_cache=False)

        print('Saving the following data to %s'%save_folder)

        for key in tracked_data:
            np.savez_compressed('%s/%s%i'%(save_folder, key, run_num)
                                ,tracked_data[key])
            print(tracked_data[key])
