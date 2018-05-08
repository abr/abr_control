"""
Saves data that is commonly kept from tests and maintains a standard naming structure

"""
from abr_control.utils import DataHandler

class SaveTestData():
    def __init__():
        pass
    def save_data(session, run, test_name, test_group, use_cache=True,
                  q=None, dq=None, u=None, adapt=None, time=None, target=None,
                  error=None, training_signal=None, input_signal=None, ee_xyz=None,
                  int_err=None, friction=None, filtered_target=None, custom_params=None,
                  overwrite=False):
        """
        Saves data to standardized names for use with other scripts

        The parameters listed are to keep a standard naming convention to avoid
        issues with scripts that load data from the database. Note that not all
        of the parameters need to be passed in. Only the ones passed in will be
        saved to the database

        They can all be passed in in the custom_params dictionary, however if
        a different key is passed than what is typically use then some of the
        plotting scripts will not work. The naming convention is to use a key
        that is the same as the parameter name (ie: key for q = 'q', key for
        training_signal is 'training_signal' etc.

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
        use_cache: Boolean, Optional (Default:True)
            True to prepend the abr_control cache folder to the directory
            provided. This location is specified in abr_control/utils/paths.py
            False to use the directory pass in as is
        q: list of floats, Optional (Default: None)
            a list of joint angles over time
        dq: list of floats, Optional (Default: None)
            a list of joint velocities over time
        u: list of floats, Optional (Default: None)
            a list of control signals over time
        adapt: list of floats, Optional (Default: None)
            a list of the adaptive control signals over time
        time: list of floats, Optional (Default: None)
            a list of the loop speed over time
        target: list of floats, Optional (Default: None)
            a list of targets over time
        filtered_target: list of floats, Optional (Default: None)
            a list of the filtered target path
        error: list of floats, Optional (Default: None)
            a list of error from end-effector to target over time
        training_signal: list of floats, Optional (Default: None)
            a list of training signals passed to the adaptive controller over
            time
        input_signal: list of floats, Optional (Default: None)
            a list of input signals passed to the adaptive controller over time
        ee_xyz: list of floats, Optional (Default: None)
            a list of end-effector positions over time
        int_err: list of floats, Optional (Default: None)
            a list of the cumulative errors from the integrative controller
            over time
        friction: list of floats, Optional (Default: None)
            a list of joint frictions artificially added over time
        custom_params: dictionary, Optional (Default: None)
            a dictionary of keys and data to save. This allows the user to
            define parameters that are not listed. The listed parameters can
            also be passed in in this dictionary as long as the same key is
            used (key matches parameter name: training_signal key ->
            'training_signal')
            format: {'key1': key1_data_list, 'key2': key2_data_list.....}
        overwrite: boolean, Optional (Default: False)
            determines whether or not to overwrite data if a group / dataset
            already exists
        """

        #TODO: decided whether it's worth having these parameters predefined
        preset_params = {'q': q,
                         'dq': dq,
                         'u': u,
                         'adapt': adapt,
                         'time': time,
                         'target': target,
                         'filtered_target': filtered_target,
                         'error': error,
                         'training_signal': training_signal,
                         'input_signal': input_signal,
                         'ee_xyz': ee_xyz,
                         'int_err': int_err,
                         'friction': friction,
                        }

        dh = DataHandler(use_cache=use_cache)
        # Save preset params, if they are not passed in the entry
        # will be 'no data'
        print('Saving preset parameters...')
        dh.save_data(tracked_data=preset_params, session=session, run=run,
                     test_name=test_name, test_group=test_group,
                     overwrite=overwrite)

        if custom_params is not None:
            print('Saving custom parameters...')
            dh.save_data(tracked_data=custom_params, session=session, run=run,
                         test_name=test_name, test_group=test_group,
                         overwrite=overwrite)
