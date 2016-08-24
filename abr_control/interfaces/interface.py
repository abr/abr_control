import numpy as np


class interface:
    """ Base class for interfaces.
    The purpose of interfaces is to abstract away the API
    and overhead of connection / disconnection etc for each
    of the different systems that can be controlled.
    """

    def __init__(self, robot_config):
        self.robot_config = robot_config

        # for plotting
        self.track_q = []
        self.track_dq = []
        self.track_hand = []

    def connect(self):
        """ All initial setup. """
        raise NotImplementedError

    def disconnect(self):
        """ Any socket closing etc that must be done
        when done with the arm. """
        raise NotImplementedError

    def apply_u(self, u):
        """ Applies the set of torques u to the arm.

        u np.array: An array of joint torques
        """
        raise NotImplementedError

    def get_feedback(self):
        """ Returns a dictionary of relevant feedback to the
        controller. At very least this contains q, dq.
        """
        raise NotImplementedError

    def save_info(self, folder, postfix=''):
        """ Save the stored information to file.

        folder string: name of the folder to save data in
        postfix string: appended to end of files for easier ID
        """
        np.savez_compressed('%s/q%s' % (folder, postfix),
                            q=np.array(self.track_q))
        np.savez_compressed('%s/dq%s' % (folder, postfix),
                            dq=np.array(self.track_dq))
