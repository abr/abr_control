"""
Handler for saving and loading data from HDF5 database

"""
import numpy as np
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

    def __init__(self, use_cache=True, db_name=None):
        """
        use_cache: Boolean, Optional (Default:True)
            True to prepend the abr_control cache folder to the directory
            provided. This location is specified in abr_control/utils/paths.py
            False to use the directory pass in as is
        db_name: string, Optional (Default: abr_control_db if left as None)
            name of the database being used
        """
        # create our database if it does not exist
        self.use_cache = use_cache
        if db_name is None:
            db_name = 'abr_control_db'
        if self.use_cache:
            self.db_loc = '%s/%s.h5'%(cache_dir, db_name)
        else:
            self.db_loc = '%s.h5'%db_name

        db = h5py.File(self.db_loc, 'a')
        db.close()

    def check_group_exists(self, location, create=False):
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
        #try:
            #exists = isinstance(db([location], h5py.Group))
        exists = location in db

        #except KeyError:
        if exists is False:
            if create:
                #print('Creating new group %s...'%location)
                db.create_group(location)
                exists = True
            else:
                #print('%s group does not exist, to create it set create=True'%location)
                exists = False

        db.close()
        return exists

    def last_save_location(self, session=None, run=None, test_name='test',
            test_group='test_group', create=True):
        """ Search for latest run in provided test

        If the user sets session or run to None, the function searches the
        specified test_name for the highest numbered run in the
        highest numbered session, otherwise the specified values are used.
        Returns highest numbered run, session and path, unless a run or session
        was specified by the user to use.

        If the user specifies a session, or run that do not exist, the 0th
        session and/or run will be created. However, if the test_name does not
        exist, an exception will be raised to alert the user

        This function is used for both saving and loading, if used for saving
        then it may be desirable to create a group if the provided one does not
        exist. This can be done by setting create to True. The opposite is true
        for loading, where None should be returned if the group does not exist.
        In this scenario create should be set to False. By default, these are
        the settings for the load and save functions

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
        create: Boolean, Optional (Default: True)
            whether to create the group passed in if it does not exist
        """

        self.db = h5py.File(self.db_loc, 'a')

        # first check whether the test passed in exists
        exists = self.check_group_exists(location='%s/%s/'%(test_group,
            test_name), create=create)

        # if the test does not exist, return None
        if exists is False:
            run = None
            session = None
            location = '%s/%s/'%(test_group, test_name)
            self.db.close()
            return [run, session, location]

        # If a session is provided, check if it exists
        if session is not None:
            # check if the provided session exists before continuing, create it
            # if it does not and create is set to True
            exists = self.check_group_exists(location='%s/%s/session%03d/'%(test_group,
                test_name, session), create=create)
            # if exists, use the value
            if exists:
                session = 'session%03d' %session
                #print('session is: ', session)
            else:
                run = None
                session = None
                location = '%s/%s/'%(test_group, test_name)
                self.db.close()
                return [run, session, location]

        # if not looking for a specific session, check what our highest
        # numbered session is
        elif session is None:
            # get all of the session keys
            session_keys = list(self.db['%s/%s'%(test_group, test_name)].keys())

            if session_keys:
                session = max(session_keys)
                #print('session is: ', session)

            elif create:
                # No session can be found, create it if create is True
                # print('The group %s/%s exists, but there is no session'
                #         % (test_group, test_name)
                #       + ' saved... \nCreating session000 group')
                self.db.create_group('%s/%s/session000'%(test_group, test_name))
                session = 'session000'
                #print('session is: ', session)

            else:
                run = None
                session = None
                location = '%s/%s/'%(test_group, test_name)
                self.db.close()
                return [run, session, location]

        if run is not None:
            # check if the provided run exists before continuing, create it
            # if it does not and create is set to True
            exists = self.check_group_exists(location='%s/%s/%s/run%03d'%(test_group,
                test_name, session, run), create=create)
            # if exists, use the value
            if exists:
                run = 'run%03d' %run
                #print('run is: ', run)
            else:
                run = None
                location = '%s/%s/'%(test_group, test_name)
                self.db.close()
                return [run, session, location]

        # usually this will be set to None so that we can start from where we
        # left off in testing, but sometimes it is useful to pick up from
        # a specific run
        elif run is None:
            # get all of the run keys
            run_keys = list(self.db['%s/%s/%s'%(test_group, test_name, session)].keys())

            if run_keys:
                run = max(run_keys)
                #print('run is: ', run)

            else:
                # print('The group %s/%s/%s exists, but there are no runs saved...'
                #       %(test_group, test_name, session))
                run = None
                #print('run is: ', run)

        location = '%s/%s/'%(test_group, test_name)
        if session is not None:
            session = int(session[7:])
            location += 'session%03d/'%session
        else:
            location += '%s/'%session
        if run is not None:
            run = int(run[3:])
            location += 'run%03d'%run
        else:
            location += '%s/'%run

        self.db.close()
        return [run, session, location]

    def save(self, data, save_location='test', overwrite=False, create=True,
            timestamp=True):
        """ Saves data to a dataset in the provided group

        Parameters
        ----------
        data: dictionary of lists to save
            instantiate as
                data = {'data1': [], 'data2': []}
            append as
                data['data1'].append(np.copy(data_to_save))
                data['data2'].append(np.copy(other_data_to_save))
        save_location: string, Optional (Default: 'test')
            the group that all of the data will be saved to
        overwrite: boolean, Optional (Default: False)
            determines whether or not to overwrite data if a group / dataset
            already exists
        create: boolean, Optional (Default: True)
            determines whether to create the group provided if it does not
            exist, or to warn to the user that it does not
        timestamp: boolean, Optional (Default: True)
            whether to save timestamp with data
        """

        db = h5py.File(self.db_loc, 'a')

        # try to create the group, if it exists a ValueError will be raised. If
        # the user wished to overwrite the dataset that already exists then
        # continue, otherwise raise the ValueError and alert the user to set
        # the overwrite parameter to true
        try:
            db.create_group(save_location)
        except ValueError:
            pass
            #TODO: this should not need an overwrite, could be saving several
            #datasets to the same location, in which case you shouldn't be
            #overwriting. Can't currently think of a time when you would want to
            #overwrite the group itself, just the data
            # if overwrite:
            #     # pass, do not delete and write again incase their is data in
            #     # this group that is not being overwritten
            #     pass
            # else:
            #     raise ValueError('The group %s already exists. If you wish to'
            #                      % save_location
            #                      + ' overwrite the dataset set overwrite=True')

        # print('Saving data to %s'%save_location)

        if timestamp:
            import time
            data['timestamp'] = time.strftime("%H:%M:%S")
            data['datestamp'] = time.strftime("%Y/%m/%d")

        for key in data:
            #print('key:%s | data:'%(key), data[key])
            # to avoid errors, if no data is passed in with the key, set the
            # value to 'no data'
            if data[key] is None:
                data[key] = 'no data'
                #print('key: %s has no data, setting to \'no_data\'' % key)
            try:
                db[save_location].create_dataset('%s'%key, data=data[key])
            except RuntimeError as e:
                if overwrite:
                    # if dataset already exists, then overwrite the data
                    del db[save_location+'/%s'%key]
                    db[save_location].create_dataset('%s'%key, data=data[key])
                else:
                    print(e)
                    raise RuntimeError('Dataset %s already exists in %s, set'
                            %(save_location, key)
                    + ' overwrite=True to overwrite')
                    print('Dataset %s already exists in %s'
                            %(save_location, key)
                            + ' \n Skipping dataset...')
                    print('If you wish to overwrite the old data, set '
                          + 'overwrite to True')
            except TypeError as e:
                print(e)
                print('Trying to save %s to %s'%(key, save_location))

        # print('Data saved.')
        db.close()


    def save_data(self, tracked_data, session=None, run=None,
            test_name='test', test_group='test_group', overwrite=False,
            create=True, timestamp=True):
        #TODO: currently module does not check whether a lower run or session
        # exists if the user provides a number for either parameter, could lead
        # to a case where user provides run to save as 6, but runs 0-5 do not
        # exist, is it worth adding a check for this?
        """ Saves data collected from test trials with a standard naming convention

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
        overwrite: boolean, Optional (Default: False)
            determines whether or not to overwrite data if a group / dataset
            already exists
        timestamp: boolean, Optional (Default: True)
            whether to save timestamp with data
        """

        if run is not None:
            run = 'run%03d'%run
        if session is not None:
            session = 'session%03d'%session
        if session is None or run is None:
            # user did not specify either run or session so we will grab the
            # last entry in the test_name directory based off the highest
            # numbered session and/or run
            [run, session, __] = self.last_save_location(session=session,
                    run=run, test_name=test_name, test_group=test_group,
                    create=create)

            # if no previous run saved, start saving in run0
            if run is None:
                run = 'run000'

        group_path = '%s/%s/%s/%s'%(test_group, test_name, session, run)

        # save the data
        self.save(data=tracked_data, save_location=group_path,
                overwrite=overwrite, create=create, timestamp=timestamp)

    def load(self, params, save_location):
        """
        Loads the data listed in params from the group provided

        The path to the group is used as 'test_group/test_name/session/run'
        Note that session and run are ints that from the user end, and are
        later used in the group path as ('run%i'%run) and ('session%i'%session)

        params: list of strings
            ex: ['q', 'dq', 'u', 'adapt']
            if you are unsure about what the keys are for the data saved, you
            can use the get_keys() function to list the keys in a provided
            group path
        save_location: string
            the location to look for data
            for trial data it is in the form of
            'test_group/test_name/session_num/run_num'
        """
        # check if the group exists
        exists = self.check_group_exists(location=save_location, create=False)

        # if group path does not exist, raise an exception to alert the
        # user
        if exists is False:
            raise ValueError('The path %s does not exist'%(save_location))
        # otherwise load the keys
        else:
            db = h5py.File(self.db_loc, 'a')
            saved_data = {}
            for key in params:
                saved_data[key] = np.array(db.get('%s/%s'%(save_location,key)))

            db.close()
        return saved_data


    def load_data(self, params, session=None, run=None,
            test_name='test', test_group='test_group', create=False):
        """
        Loads the data listed in params from the group provided

        The path to the group is used as 'test_group/test_name/session/run'
        Note that session and run are ints that from the user end, and are
        later used in the group path as ('run%i'%run) and ('session%i'%session)

        params: list of strings
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
        # if the user doesn'r provide either run or session numbers, the
        # highest numbered run and session are searched for in the provided
        # test_group/test_name location
        # print('run: ', run)
        # print('session: ', session)
        if session is None or run is None:
            # if session is not None and run is None:
            #     print('Checking for lastest run in session%03d'%session)
            # elif session is None and run is not None:
            #     print('Checking for run%i data in latest session'%run)
            # else:
            #     print('Checking for lastest session and run...')
            [run, session, group_path] = self.last_save_location(session=session,
                    run=run, test_name=test_name, test_group=test_group,
                    create=create)
            # if run is None:
            #     print('There is no run saved in %s/%s/%s: Returning None'
            #             % (test_group, test_name, session))

        else:
            # print('using user provided run and session')
            # print('run: ', run)
            # print('session: ', session)
            session = 'session%03d'%session
            run = 'run%03d'%run

        if run is None:
            saved_data = None
        else:
            group_path = '%s/%s/%s/%s'%(test_group, test_name, session, run)

            saved_data = self.load(params=params, save_location=group_path)

        return saved_data

    def get_keys(self, group_path):
        """
        Returns a list of keys from the group path provided

        group_path: string
            path to the group that you want the keys from
            ex: 'my_feature_test/sub_test_group/session000/run003'
        returns a list of keys from group_list
        """
        db = h5py.File(self.db_loc, 'a')
        if isinstance(db[group_path], h5py.Dataset):
            # print('The provided path points to a dataset, please provide'
            #         + ' a group to obtain keys')
            keys = None
        else:
            #print('looking for keys in %s' % group_path)
            keys = list(db[group_path].keys())
        db.close()
        return keys

    def delete(self, path):
        #TODO: this needs a lot of work, add checks to see if path exists, some
        # feature to double check user is sure, maybe a backup function?
        #TODO: Add something like a recycling bin so you can undo a delete
        db = h5py.File(self.db_loc, 'a')
        del db[path]

    def rename(self, old, new, delete_old=True):
        db = h5py.File(self.db_loc, 'a')
        db[new] = db[old]
        if delete_old:
            del db[old]
