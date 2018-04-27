"""
Saves data to HDF5 database

Parameters
----------
"""
from abr_control.utils.paths import cache_dir
try:
    import h5py
except ImportError:
    print("You must install h5py to use the database utilities")

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

    def __init__(self, use_cache=True):
        """
        use_cache: Boolean, Optional (Default:True)
            True to prepend the abr_control cache folder to the directory
            provided. This location is specified in abr_control/utils/paths.py
            False to use the directory pass in as is
        """
        # create our database if it does not exist
        self.use_cache = use_cache
        if self.use_cache:
            self.db_loc = '%s/abr_control_db.h5'%cache_dir
        else:
            self.db_loc = 'abr_control_db.h5'

        db = h5py.File(self.db_loc, 'a')
        db.close()

    def check_group_exists(self, location, create=True):
        """
        Checks if the provided group exists in the database.

        Parameters
        ----------
        location: string
            The database group that the function checks for,
        create: boolean, Optional (Default:True)
            true: create group if it does not exist
            false: do not create group if it does not exist
        """
        #TODO: should we add check if location is a dataset?
        db = h5py.File(self.db_loc, 'a')
        try:
            exists = isinstance(db([location], h5py.Group))

        except KeyError:
            if create:
                print('%s group does not exist, creating it...'%location)
                db.create_group(location)
                exists = True
            else:
                print('%s group does not exist, to create it set create=True'%location)
                exists = False

        db.close()
        return exists

    def last_save_location(self, session=None, run=None, test_name='test',
            test_group='test_group'):
        """ Search for most latest run in provided test

        If the user sets session or run to None, the function searches the
        specified test_name for the highest numbered run in the
        highest numbered session, otherwise the specified values are used.
        Returns highest numbered run, session and path, unless a run or session
        was specified by the user to use.

        If the user specifies a session, or run that do not exist, the 0th
        session and/or run will be created. However, if the test_name does not
        exist, an exception will be raised to alert the user

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
        test_group: string, Optional (Default: 'test_group')
            the group that all of the various test_name tests belong to. This
            is helpful for grouping tests together relative to a specific
            feature being tested, a paper that they belong to etc.
        """

        db = h5py.File(self.db_loc, 'a')
        # first check whether the test passed in exists
        exists = self.check_group_exists(location='%s/%s/'%(test_group, test_name), create=False)

        # if the test does not exist, raise an exception
        if exists is False:
            raise ValueError('The %s/%s does not exist'%(test_group, test_name))

        # if not looking for a specific session, check what our highest
        # numbered session is
        if session is None:
            # get all of the session keys
            session_keys = list(db['%s/%s'%(test_group, test_name)].keys())

            if session_keys:
               session = max(session_keys)
            # test_group/test_name exists, but for some reason we do not have
            # a session saved. Alert the user of this and create session0 group
            else:
                print('The group %s/%s exists, but there is no session'
                      + ' saved... \n Creating session0 group' %(test_group,
                          test_name))
                db.create_group('%s/%s/session0'%(test_group, test_name))
                session = 'session0'
        else:
            session = 'session%i' %session

        # usually this will be set to None so that we can start from where we
        # left off in testing, but sometimes it is useful to pick up from
        # a specific run
        if run is None:
            # get all of the run keys
            run_keys = list(db['%s/%s/%s'%(test_group, test_name, session)].keys())

            if run_keys:
               run = max(run_keys)
            # no run exists in this session, so start from run0
            else:
                print('The group %s/%s/%s exists, but there are no run groups'
                      + ' saved...' %(test_group,
                          test_name, session))
                run = None
        else:
            run = 'run%i'%run

        location = '%s/%s/%s/%s'%(test_group, test_name, session, run)

        db.close()
        return [run, session, location]

    def save_data(self, tracked_data, session=None, run=None,
            test_name='test', test_group='test_group'):
        #TODO: add check to see if path already exists
        """ Save the data to the specified test_name group

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
        test_group: string, Optional (Default: 'test_group')
            the group that all of the various test_name tests belong to. This
            is helpful for grouping tests together relative to a specific
            feature being tested, a paper that they belong to etc.
        """

        db = h5py.File(self.db_loc, 'a')
        if session or run is None:
            # user did not specify either run or session so we will grab the
            # last entry in the test_name directory based off the highest
            # numbered session and/or run
            [run, session] = last_save_location(session=session,
                    run=run, test_name=test_name, test_group=test_group)

        # if the user specified a run, then save the data under that run,
        # otherwise increment the last saved run by 1 to save data in a new
        # folder
        if run is None:
            run = 'run0'

        group_path = '%s/%s/%s/%s'%(test_group, test_name, session, run)
        db.create_group(group_path)

        print('Saving data to %s'%group_path)

        for key in tracked_data:
            db[group_path].create_dataset('%s'%key, data=tracked_data[key])

        print('Data saved.')
        db.close()

    def load_data(self, desired_params, session=None, run=None,
            test_name='test', test_group='test_group'):
        """
        Loads the data listed in desired_params from the group provided

        The path to the group is used as 'test_group/test_name/session/run'
        Note that session and run are ints that from the user end, and are
        later used in the group path as ('run%i'%run) and ('session%i'%session)

        desired_params: list of strings
            ex: ['q', 'dq', 'u', 'adapt']
            if you are unsure about what the keys are for the data saved, you
            can use the get_keys() function to list the keys in a provided
            group path
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
        test_group: string, Optional (Default: 'test_group')
            the group that all of the various test_name tests belong to. This
            is helpful for grouping tests together relative to a specific
            feature being tested, a paper that they belong to etc.
        """
        # the keys saved should be the same between all runs in a session, and
        # between sessions of a test_name, so they are not required to be
        # provided
        if session or run is None:
            raise ValueError('A session and run number are required to load'
                             + 'data')
        else:
            session = 'session%i'%session
            run = 'run%i'%run
            group_path = '%s/%s/%s/%s'%(test_group, test_name, session, run)

            # check if the group exists
            exists = self.check_group_exists(location=group_path, create=False)

            # if group path does not exist, raise an exception to alert the
            # user
            if exists is False:
                raise ValueError('The group path %s does not exist'%(group_path))
            # otherwise load the keys
            else:
                db = h5py.File(self.db_loc, 'a')
                saved_data = {}
                for key in desired_params
                    saved_data[key] = np.array(db.get('%s/%s'%(group_path,key)))

                db.close()
        return saved_data

    def get_keys(self, group_path):
        """
        Returns a list of keys from the group path provided

        group_path: string
            path to the group that you want the keys from
            ex: 'my_feature_test/sub_test_group/session0/run3'
        returns a list of keys from group_list
        """
        db = h5py.File(self.db_loc, 'a')
        keys = list(db[group_path].keys())
        db.close()
        return keys
