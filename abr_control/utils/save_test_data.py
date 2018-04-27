"""
Saves data that is usually of interest during training

"""
class SaveLearningData():
    def __init__():
        pass
    def save_data(q=None, dq=None, u=None, adapt=None, time=None, target=None,
            error=None, training_signal=None, input_signal=None, ee_xyz=None,
            int_err=None, friction=None, custom_params=None):
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
        """
